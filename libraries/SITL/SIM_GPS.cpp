/*
  SITL handling

  This simulates a GPS on a serial port

  Andrew Tridgell November 2011
 */

#include "SIM_GPS.h"

#if HAL_SIM_GPS_ENABLED

#include <time.h>
#include <sys/time.h>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/AP_HAL.h>
#include <SITL/SITL.h>
#include <AP_InternalError/AP_InternalError.h>

#include "SIM_GPS_FILE.h"
#include "SIM_GPS_Trimble.h"
#include "SIM_GPS_MSP.h"
#include "SIM_GPS_NMEA.h"
#include "SIM_GPS_NOVA.h"
#include "SIM_GPS_SBP2.h"
#include "SIM_GPS_SBP.h"
#include "SIM_GPS_UBLOX.h"
#include "SIM_GPS_SBF.h"

#include <GCS_MAVLink/GCS.h>

// the number of GPS leap seconds - copied from AP_GPS.h
#define GPS_LEAPSECONDS_MILLIS 18000ULL

extern const AP_HAL::HAL& hal;

using namespace SITL;

// ensure the backend we have allocated matches the one that's configured:
GPS_Backend::GPS_Backend(GPS &_front, uint8_t _instance) :
    instance{_instance},
    front{_front}
{
    _sitl = AP::sitl();
}

ssize_t GPS_Backend::write_to_autopilot(const char *p, size_t size) const
{
    return front.write_to_autopilot(p, size);
}

ssize_t GPS_Backend::read_from_autopilot(char *buffer, size_t size) const
{
    const ssize_t ret = front.read_from_autopilot(buffer, size);
    front.scan_rtcm(buffer, ret);
    return ret;
}

GPS::GPS(uint8_t _instance) :
    SerialDevice(8192, 2048),
    instance{_instance}
{
}

uint32_t GPS::device_baud() const
{
    if (backend == nullptr) {
        return 0;
    }
    return backend->device_baud();
}

/*
  write some bytes from the simulated GPS
 */
ssize_t GPS::write_to_autopilot(const char *p, size_t size) const
{
    // the second GPS instance fails in a different way to the first;
    // the first will start sending back 3 satellites, the second just
    // stops responding when disabled.  This is not necessarily a good
    // thing.
    if (instance == 1 && _sitl->gps_disable[instance]) {
        return -1;
    }

    const float byteloss = _sitl->gps_byteloss[instance];

    // shortcut if we're not doing byteloss:
    if (!is_positive(byteloss)) {
        return SerialDevice::write_to_autopilot(p, size);
    }

    size_t ret = 0;
    while (size--) {
        float r = ((((unsigned)random()) % 1000000)) / 1.0e4;
        if (r < byteloss) {
            // lose the byte
            p++;
            continue;
        }

        const ssize_t pret = SerialDevice::write_to_autopilot(p, 1);
        if (pret == 0) {
            // no space?
            return ret;
        }
        if (pret != 1) {
            // error has occured?
            return pret;
        }
        ret++;
        p++;
    }

    return ret;
}

/*
  get timeval using simulation time
 */
void GPS_Backend::simulation_timeval(struct timeval *tv)
{
    uint64_t now = AP_HAL::micros64();
    static uint64_t first_usec;
    static struct timeval first_tv;
    if (first_usec == 0) {
        first_usec = now;
        first_tv.tv_sec = AP::sitl()->start_time_UTC;
    }
    *tv = first_tv;
    tv->tv_sec += now / 1000000ULL;
    uint64_t new_usec = tv->tv_usec + (now % 1000000ULL);
    tv->tv_sec += new_usec / 1000000ULL;
    tv->tv_usec = new_usec % 1000000ULL;
}

/*
  simple simulation of jamming
 */
void GPS::simulate_jamming(struct GPS_Data &d)
{
    auto &jam = jamming[instance];
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - jam.last_jam_ms > 1000) {
        jam.jam_start_ms = now_ms;
        jam.latitude = d.latitude;
        jam.longitude = d.longitude;
    }
    jam.last_jam_ms = now_ms;

    // how often each of the key state variables change during jamming
    const float vz_change_hz = 0.5;
    const float vel_change_hz = 0.8;
    const float pos_change_hz = 1.1;
    const float sats_change_hz = 3;
    const float acc_change_hz = 3;

    if (now_ms - jam.jam_start_ms < unsigned(1000U+(get_random16()%5000))) {
        // total loss of signal for a period at the start is common
        d.num_sats = 0;
        d.have_lock = false;
    } else {
        if ((now_ms - jam.last_sats_change_ms)*0.001 > 2*fabsf(rand_float())/sats_change_hz) {
            jam.last_sats_change_ms = now_ms;
            d.num_sats = 2 + (get_random16() % 15);
            if (d.num_sats >= 4) {
                if (get_random16() % 2 == 0) {
                    d.have_lock = false;
                } else {
                    d.have_lock = true;
                }
            } else {
                d.have_lock = false;
            }
        }
        if ((now_ms - jam.last_vz_change_ms)*0.001 > 2*fabsf(rand_float())/vz_change_hz) {
            jam.last_vz_change_ms = now_ms;
            d.speedD = rand_float() * 400;
        }
        if ((now_ms - jam.last_vel_change_ms)*0.001 > 2*fabsf(rand_float())/vel_change_hz) {
            jam.last_vel_change_ms = now_ms;
            d.speedN = rand_float() * 400;
            d.speedE = rand_float() * 400;
        }
        if ((now_ms - jam.last_pos_change_ms)*0.001 > 2*fabsf(rand_float())/pos_change_hz) {
            jam.last_pos_change_ms = now_ms;
            jam.latitude += rand_float()*200 * LATLON_TO_M_INV * 1e-7;
            jam.longitude += rand_float()*200 * LATLON_TO_M_INV * 1e-7;
        }
        if ((now_ms - jam.last_acc_change_ms)*0.001 > 2*fabsf(rand_float())/acc_change_hz) {
            jam.last_acc_change_ms = now_ms;
            d.vertical_acc = fabsf(rand_float())*300;
            d.horizontal_acc = fabsf(rand_float())*300;
            d.speed_acc = fabsf(rand_float())*50;
        }
    }

    d.latitude = constrain_float(jam.latitude, -90, 90);
    d.longitude = constrain_float(jam.longitude, -180, 180);
}

/*
  return GPS time of week
 */
GPS_Backend::GPS_TOW GPS_Backend::gps_time()
{
    GPS_TOW gps_tow;
    struct timeval tv;
    simulation_timeval(&tv);
    const uint32_t epoch = 86400*(10*365 + (1980-1969)/4 + 1 + 6 - 2) - (GPS_LEAPSECONDS_MILLIS / 1000ULL);
    uint32_t epoch_seconds = tv.tv_sec - epoch;
    gps_tow.week = epoch_seconds / AP_SEC_PER_WEEK;
    uint32_t t_ms = tv.tv_usec / 1000;
    // round time to nearest 200ms
    gps_tow.ms = (epoch_seconds % AP_SEC_PER_WEEK) * AP_MSEC_PER_SEC + ((t_ms/200) * 200);
    return gps_tow;
}

void GPS::check_backend_allocation()
{
    const Type configured_type = Type(_sitl->gps_type[instance].get());
    if (allocated_type == configured_type) {
        return;
    }

    // mismatch; delete any already-allocated backend:
    if (backend != nullptr) {
        delete backend;
        backend = nullptr;
    }

    // attempt to allocate backend
    switch (configured_type) {
    case Type::NONE:
        // no GPS attached
        break;

#if AP_SIM_GPS_UBLOX_ENABLED
    case Type::UBLOX:
        backend = NEW_NOTHROW GPS_UBlox(*this, instance);
        break;
#endif

#if AP_SIM_GPS_NMEA_ENABLED
    case Type::NMEA:
        backend = NEW_NOTHROW GPS_NMEA(*this, instance);
        break;
#endif

#if AP_SIM_GPS_SBP_ENABLED
    case Type::SBP:
        backend = NEW_NOTHROW GPS_SBP(*this, instance);
        break;
#endif

#if AP_SIM_GPS_SBP2_ENABLED
    case Type::SBP2:
        backend = NEW_NOTHROW GPS_SBP2(*this, instance);
        break;
#endif

#if AP_SIM_GPS_NOVA_ENABLED
    case Type::NOVA:
        backend = NEW_NOTHROW GPS_NOVA(*this, instance);
        break;
#endif

#if AP_SIM_GPS_MSP_ENABLED
    case Type::MSP:
        backend = NEW_NOTHROW GPS_MSP(*this, instance);
        break;
#endif

#if AP_SIM_GPS_SBF_ENABLED
    case Type::SBF:
        backend = NEW_NOTHROW GPS_SBF(*this, instance);
        break;
#endif

#if AP_SIM_GPS_TRIMBLE_ENABLED
    case Type::TRIMBLE:
        backend = NEW_NOTHROW GPS_Trimble(*this, instance);
        break;
#endif

#if AP_SIM_GPS_FILE_ENABLED
    case Type::FILE:
        backend = NEW_NOTHROW GPS_FILE(*this, instance);
        break;
#endif
    };

    if (configured_type != Type::NONE && backend == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SIM_GPS: No backend for %u", (unsigned)configured_type);
    }
    allocated_type = configured_type;
}

/*
  possibly send a new GPS packet
 */
void GPS::update()
{
    if (!init_sitl_pointer()) {
        return;
    }

    check_backend_allocation();
    if (backend == nullptr) {
        return;
    }

    double latitude =_sitl->state.latitude;
    double longitude = _sitl->state.longitude;
    float altitude = _sitl->state.altitude;
    const double speedN = _sitl->state.speedN;
    const double speedE = _sitl->state.speedE;
    const double speedD = _sitl->state.speedD;
    const uint32_t now_ms = AP_HAL::millis();

    if (now_ms < 20000) {
        // apply the init offsets for the first 20s. This allows for
        // having the origin a long way from the takeoff location,
        // which makes testing long flights easier
        latitude += _sitl->gps_init_lat_ofs;
        longitude += _sitl->gps_init_lon_ofs;
        altitude += _sitl->gps_init_alt_ofs;
    }

    //Capture current position as basestation location for
    if (!_gps_has_basestation_position &&
        now_ms >= _sitl->gps_lock_time[0]*1000UL) {
        _gps_basestation_data.latitude = latitude;
        _gps_basestation_data.longitude = longitude;
        _gps_basestation_data.altitude = altitude;
        _gps_basestation_data.speedN = speedN;
        _gps_basestation_data.speedE = speedE;
        _gps_basestation_data.speedD = speedD;
        _gps_has_basestation_position = true;
    }

    const uint8_t idx = instance;  // alias to avoid code churn

    struct GPS_Data d {};

    // simulate delayed lock times
    bool have_lock = (!_sitl->gps_disable[idx] && now_ms >= _sitl->gps_lock_time[idx]*1000UL);

    // Only let physics run and GPS write at configured GPS rate (default 5Hz).
    if ((now_ms - last_write_update_ms) < (uint32_t)(1000/_sitl->gps_hertz[instance])) {
        // Reading runs every iteration.
        // Beware- physics don't update every iteration with this approach.
        // Currently, none of the drivers rely on quickly updated physics.
        backend->update_read();
        return;
    }

    last_write_update_ms = now_ms;

    d.num_sats = _sitl->gps_numsats[idx];
    d.latitude = latitude;
    d.longitude = longitude;
    d.yaw_deg = _sitl->state.yawDeg;
    d.roll_deg = _sitl->state.rollDeg;
    d.pitch_deg = _sitl->state.pitchDeg;

    // add an altitude error controlled by a slow sine wave
    d.altitude = altitude + _sitl->gps_noise[idx] * sinf(now_ms * 0.0005f) + _sitl->gps_alt_offset[idx];

    // Add offset to c.g. velocity to get velocity at antenna and add simulated error
    Vector3f velErrorNED = _sitl->gps_vel_err[idx];
    d.speedN = speedN + (velErrorNED.x * rand_float());
    d.speedE = speedE + (velErrorNED.y * rand_float());
    d.speedD = speedD + (velErrorNED.z * rand_float());

    d.have_lock = have_lock;

    // fill in accuracies
    d.horizontal_acc = _sitl->gps_accuracy[idx];
    d.vertical_acc = _sitl->gps_accuracy[idx];
    d.speed_acc = _sitl->gps_vel_err[instance].get().xy().length();

    if (_sitl->gps_drift_alt[idx] > 0) {
        // add slow altitude drift controlled by a slow sine wave
        d.altitude += _sitl->gps_drift_alt[idx]*sinf(now_ms*0.001f*0.02f);
    }

    // correct the latitude, longitude, height and NED velocity for the offset between
    // the vehicle c.g. and GPS antenna
    Vector3f posRelOffsetBF = _sitl->gps_pos_offset[idx];
    if (!posRelOffsetBF.is_zero()) {
        // get a rotation matrix following DCM conventions (body to earth)
        Matrix3f rotmat;
        _sitl->state.quaternion.rotation_matrix(rotmat);

        // rotate the antenna offset into the earth frame
        Vector3f posRelOffsetEF = rotmat * posRelOffsetBF;

        // Add the offset to the latitude, longitude and height using a spherical earth approximation
        double const earth_rad_inv = 1.569612305760477e-7; // use Authalic/Volumetric radius
        double lng_scale_factor = earth_rad_inv / cos(radians(d.latitude));
        d.latitude += degrees(posRelOffsetEF.x * earth_rad_inv);
        d.longitude += degrees(posRelOffsetEF.y * lng_scale_factor);
        d.altitude -= posRelOffsetEF.z;

        // calculate a velocity offset due to the antenna position offset and body rotation rate
        // note: % operator is overloaded for cross product
        Vector3f gyro(radians(_sitl->state.rollRate),
                      radians(_sitl->state.pitchRate),
                      radians(_sitl->state.yawRate));
        Vector3f velRelOffsetBF = gyro % posRelOffsetBF;

        // rotate the velocity offset into earth frame and add to the c.g. velocity
        Vector3f velRelOffsetEF = rotmat * velRelOffsetBF;
        d.speedN += velRelOffsetEF.x;
        d.speedE += velRelOffsetEF.y;
        d.speedD += velRelOffsetEF.z;
    }

    if (_sitl->gps_rtk[idx]) {
        simulate_rtk(d, now_ms);
    }

    // get delayed data
    d.timestamp_ms = now_ms;
    d = interpolate_data(d, _sitl->gps_delay_ms[instance]);

    // Applying GPS glitch
    // Using first gps glitch
    Vector3f glitch_offsets = _sitl->gps_glitch[idx];
    d.latitude += glitch_offsets.x;
    d.longitude += glitch_offsets.y;
    d.altitude += glitch_offsets.z;

        if (_sitl->gps_jam[idx] == 1) {
            simulate_jamming(d);
        }

    backend->publish(&d);
}

void GPS_Backend::update_read()
{
    // swallow any config bytes, fast enough for injected RTCM corrections
    char buf[64];
    read_from_autopilot(buf, sizeof(buf));
}

void GPS::scan_rtcm(const char *buffer, ssize_t len)
{
    for (ssize_t i=0; i<len; i++) {
        if (!rtcm_parser.read(uint8_t(buffer[i]))) {
            continue;
        }
        const uint16_t id = rtcm_parser.get_id();
        const uint32_t now_ms = AP_HAL::millis();
        if (id == 1005 || id == 1006) {
            rtk.last_arp_ms = now_ms;
        } else if (id >= 1071 && id <= 1137) {
            if (rtk.last_msm_ms == 0 || now_ms - rtk.last_msm_ms > 30000) {
                rtk.stream_start_ms = now_ms;
            }
            rtk.last_msm_ms = now_ms;
        }
    }
}

/*
  RTK only while corrections arrive (SIM_GPS_RTK). Without them the
  position has a slowly wandering error like a standalone u-blox F9P
  (about 1 m); with them it goes RTK float, then RTK fixed (about 1 cm).
 */
void GPS::simulate_rtk(GPS_Data &d, uint32_t now_ms)
{
    // standalone error: first-order Gauss-Markov process per axis
    const float sigma_h = 1.0f, sigma_v = 1.5f, tau_s = 60.0f;
    const float dt = MIN((now_ms - rtk.last_update_ms) * 0.001f, 1.0f);
    rtk.last_update_ms = now_ms;
    const float a = expf(-dt / tau_s);
    // rand_float() is uniform in [-1,1], variance 1/3
    const float b = sqrtf(3 * (1 - a*a));
    Vector3f &e = rtk.standalone_error;
    e.x = a*e.x + sigma_h * b * rand_float();
    e.y = a*e.y + sigma_h * b * rand_float();
    e.z = a*e.z + sigma_v * b * rand_float();

    // a rover needs the observations (MSM) and the base position (1005/1006)
    const uint32_t age_ms = now_ms - rtk.last_msm_ms;
    const uint32_t stream_ms = now_ms - rtk.stream_start_ms;
    const bool corrections = rtk.last_msm_ms != 0 && rtk.last_arp_ms != 0 && age_ms < 30000;
    if (!corrections || stream_ms < 2000) {
        d.rtk = GPS_Data::RTK::NONE;
    } else if (age_ms > 5000 || stream_ms < 10000) {
        d.rtk = GPS_Data::RTK::FLOAT;
    } else {
        d.rtk = GPS_Data::RTK::FIXED;
    }

    Vector3f error;
    switch (d.rtk) {
    case GPS_Data::RTK::NONE:
        error = e;
        d.horizontal_acc = 1.5f;
        d.vertical_acc = 2.5f;
        break;
    case GPS_Data::RTK::FLOAT:
        error = e * 0.15f;
        d.horizontal_acc = 0.25f;
        d.vertical_acc = 0.4f;
        break;
    case GPS_Data::RTK::FIXED:
        error = Vector3f(rand_float(), rand_float(), 1.5f * rand_float()) * 0.01f;
        d.horizontal_acc = 0.014f;
        d.vertical_acc = 0.02f;
        break;
    }

    double const earth_rad_inv = 1.569612305760477e-7; // use Authalic/Volumetric radius
    d.latitude += degrees(error.x * earth_rad_inv);
    d.longitude += degrees(error.y * earth_rad_inv / cos(radians(d.latitude)));
    d.altitude -= error.z;
}

/*
  get delayed data by interpolation
*/
GPS_Data GPS::interpolate_data(const GPS_Data &d, uint32_t delay_ms)
{
    const uint8_t N = ARRAY_SIZE(_gps_history);
    const uint32_t now_ms = d.timestamp_ms;

    // add in into history array, shifting old elements
    memmove(&_gps_history[1], &_gps_history[0], sizeof(_gps_history[0])*(ARRAY_SIZE(_gps_history)-1));
    _gps_history[0] = d;

    for (uint8_t i=0; i<N-1; i++) {
        uint32_t dt1 = now_ms - _gps_history[i].timestamp_ms;
        uint32_t dt2 = now_ms - _gps_history[i+1].timestamp_ms;
        if (delay_ms >= dt1 && delay_ms <= dt2) {
            // we will interpolate this pair of samples. Start with
            // the older sample
            const GPS_Data &s1 = _gps_history[i+1];
            const GPS_Data &s2 = _gps_history[i];
            GPS_Data d2 = s1;
            const float p = (dt2 - delay_ms) / MAX(1,float(dt2 - dt1));
            d2.latitude += p * (s2.latitude - s1.latitude);
            d2.longitude += p * (s2.longitude - s1.longitude);
            d2.altitude += p * (s2.altitude - s1.altitude);
            d2.speedN += p * (s2.speedN - s1.speedN);
            d2.speedE += p * (s2.speedE - s1.speedE);
            d2.speedD += p * (s2.speedD - s1.speedD);
            d2.yaw_deg += p * wrap_180(s2.yaw_deg - s1.yaw_deg);
            return d2;
        }
    }
    // delay is too long, use last sample
    return _gps_history[N-1];
}

float GPS_Data::ground_track_rad() const
{
    return atan2f(speedE, speedN);
}

float GPS_Data::speed_2d() const
{
    const auto velocity = Vector2d{speedN, speedE};
    return velocity.length();
}

#endif  // HAL_SIM_GPS_ENABLED
