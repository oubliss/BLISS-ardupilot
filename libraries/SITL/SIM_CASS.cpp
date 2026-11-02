#include "SIM_CASS.h"

#if AP_SIM_CASS_ENABLED

#include <AP_Baro/AP_Baro.h>
#include <AP_Param/AP_Param.h>
#include "SIM_Aircraft.h"

extern const AP_HAL::HAL& hal;

using namespace SITL;

static constexpr float IMET_TAU = 1.5;           // s, bead response
static constexpr float HYT_TAU = 5.0;            // s, humidity response
static constexpr float HYT_TEMP_TAU = 10.0;      // s, sensor temperature response
static constexpr float HYT_SELF_HEATING = 1.2;   // K above the air at the surface density
static constexpr float SURFACE_DENSITY = 1.13;   // kg/m^3 (Norman, Oklahoma)
static constexpr float IMET_R_FIXED = 64900;     // Ohm
static constexpr float IMET_SOURCE = 3.3;        // V
static constexpr float IMET_BIAS[] { 0.08, -0.05, 0.03, -0.10 };  // K
static constexpr float HYT_BIAS[] { 0.6, -0.4, 0.2, 0.0 };        // % RH
static constexpr char COEFF_GROUP[] { 'A', 'B', 'C' };

// the coefficients AC_CASS_IMET uses for the fourth bead
static constexpr float IMET4_COEFF[] { 9.13124668e-04, 2.91081035e-04, -2.91641257e-06, 2.44944559e-07 };

void SITL::cass_ambient(const Aircraft &aircraft, float &temperature, float &humidity, float &pressure)
{
    const float alt_msl = aircraft.get_location().alt * 0.01;
    if (aircraft.get_ambient(temperature, humidity, pressure)) {
        if (!is_positive(pressure)) {
            pressure = AP_Baro::get_pressure_for_alt_amsl(alt_msl);
        }
        return;
    }
    // no atmosphere from the backend: the profile the CopterSonde SITL logged before
    const float alt = -aircraft.get_position_relhome().z;
    if (alt < 900) {
        temperature = 2.99e-11 * powf(alt, 4) - 3.70454e-8 * powf(alt, 3) - 3.86806e-6 * sq(alt) + 1.388511e-2 * alt + 287.66;
        humidity = -2.44407e-10 * powf(alt, 4) + 3.88881064e-7 * powf(alt, 3) - 1.41943e-4 * sq(alt) - 2.81895e-2 * alt + 51.63;
    } else {
        temperature = 289.64 - 6.5e-3 * (alt - 900);
        humidity = 34.44;
    }
    pressure = AP_Baro::get_pressure_for_alt_amsl(alt_msl);
}

// first-order lag of x towards target, starting at the target
static void follow(float &x, float target, float dt, float tau, bool first)
{
    if (first) {
        x = target;
        return;
    }
    x += (target - x) * (1 - expf(-dt / tau));
}

/*
  iMet bead on an ADS1115
 */
void CASS_iMet::load_coefficients()
{
    for (uint8_t i=0; i<4; i++) {
        coeff[i] = IMET4_COEFF[i];
    }
    if (instance >= ARRAY_SIZE(COEFF_GROUP)) {
        return;
    }
    float c[4];
    for (uint8_t i=0; i<4; i++) {
        char name[12];
        hal.util->snprintf(name, sizeof(name), "USR_SEN_%c%u", COEFF_GROUP[instance], unsigned(i+1));
        if (!AP_Param::get(name, c[i])) {
            return;
        }
        c[i] *= 1e-7;
    }
    if (is_zero(c[1])) {
        return;
    }
    for (uint8_t i=0; i<4; i++) {
        coeff[i] = c[i];
    }
}

// bead resistance at a temperature: invert 1/T = c0 + c1 L + c2 L^2 + c3 L^3, L = ln(R)
float CASS_iMet::resistance(float temperature) const
{
    float L = logf(10000);
    for (uint8_t i=0; i<30; i++) {
        const float f = coeff[0] + coeff[1] * L + coeff[2] * sq(L) + coeff[3] * powf(L, 3) - 1 / temperature;
        const float df = coeff[1] + 2 * coeff[2] * L + 3 * coeff[3] * sq(L);
        if (is_zero(df)) {
            break;
        }
        const float step = f / df;
        L -= step;
        if (fabsf(step) < 1e-7) {
            break;
        }
    }
    return expf(constrain_float(L, 0, 20));
}

int16_t CASS_iMet::convert(uint16_t cfg) const
{
    static constexpr float full_scale[8] { 6.144, 4.096, 2.048, 1.024, 0.512, 0.256, 0.256, 0.256 };
    const float vs = IMET_SOURCE + Aircraft::rand_normal(0, 0.0005);
    const float rt = resistance(bead_temperature);
    const float ain[4] { vs * IMET_R_FIXED / (rt + IMET_R_FIXED), vs, 0, 0 };
    float v;
    switch ((cfg >> 12) & 0x7) {
    case 0: v = ain[0] - ain[1]; break;
    case 1: v = ain[0] - ain[3]; break;
    case 2: v = ain[1] - ain[3]; break;
    case 3: v = ain[2] - ain[3]; break;
    default: v = ain[((cfg >> 12) & 0x7) - 4]; break;
    }
    const float counts = v / full_scale[(cfg >> 9) & 0x7] * 32768 + Aircraft::rand_normal(0, 0.5);
    return int16_t(constrain_float(roundf(counts), -32768, 32767));
}

void CASS_iMet::update(const Aircraft &aircraft)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (last_coeff_ms == 0 || now_ms - last_coeff_ms > 5000) {
        last_coeff_ms = now_ms;
        load_coefficients();
    }
    float temperature, humidity, pressure;
    cass_ambient(aircraft, temperature, humidity, pressure);
    const uint64_t now_us = AP_HAL::micros64();
    const float dt = (now_us - last_update_us) * 1e-6;
    follow(bead_temperature, temperature + IMET_BIAS[instance % ARRAY_SIZE(IMET_BIAS)], dt, IMET_TAU, last_update_us == 0);
    last_update_us = now_us;
}

int CASS_iMet::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    const auto &msg0 = data->msgs[0];
    if (data->nmsgs == 1 && msg0.flags == 0 && msg0.len >= 1) {
        // register pointer, optionally followed by a 16-bit value
        pointer = msg0.buf[0] & 0x03;
        if (pointer == 0x01 && msg0.len == 3) {
            config = (msg0.buf[1] << 8) | msg0.buf[2];
            if ((config & 0x8000) != 0) {
                conversion = convert(config);
            }
            config |= 0x8000;  // not busy
        }
        return 0;
    }
    I2C::i2c_msg *rd = nullptr;
    if (data->nmsgs == 2 && msg0.flags == 0 && msg0.len == 1 && data->msgs[1].flags == I2C_M_RD) {
        pointer = msg0.buf[0] & 0x03;
        rd = &data->msgs[1];
    } else if (data->nmsgs == 1 && msg0.flags == I2C_M_RD) {
        rd = &data->msgs[0];
    }
    if (rd == nullptr || rd->len != 2) {
        return -1;
    }
    uint16_t value;
    switch (pointer) {
    case 0x00: value = uint16_t(conversion); break;
    case 0x01: value = config; break;
    case 0x02: value = 0x8000; break;
    default: value = 0x7FFF; break;
    }
    rd->buf[0] = value >> 8;
    rd->buf[1] = value & 0xFF;
    return 0;
}

/*
  HYT271
 */
void CASS_HYT271::load_coefficients()
{
    for (uint8_t i=0; i<6; i++) {
        coeff[i] = 0;
    }
    if (instance >= ARRAY_SIZE(COEFF_GROUP)) {
        return;
    }
    for (uint8_t i=0; i<6; i++) {
        char name[12];
        hal.util->snprintf(name, sizeof(name), "USR_RH_%c%u", COEFF_GROUP[instance], unsigned(i+1));
        if (!AP_Param::get(name, coeff[i])) {
            coeff[i] = 0;
            return;
        }
        coeff[i] *= 1e-7;
    }
}

// the raw humidity that the firmware's correction turns into the ambient humidity
float CASS_HYT271::raw_humidity() const
{
    const float T = air_temperature;
    const float a = coeff[3];
    const float b = coeff[1] + coeff[4] * T;
    const float c = coeff[0] + coeff[2] * T + coeff[5] * sq(T) - humidity;
    float h = -1;
    if (fabsf(a) < 1e-9) {
        if (fabsf(b) > 1e-6) {
            h = -c / b;
        }
    } else {
        const float disc = sq(b) - 4 * a * c;
        if (disc >= 0) {
            // the root nearest the ambient humidity
            const float h1 = (-b + sqrtf(disc)) / (2 * a);
            const float h2 = (-b - sqrtf(disc)) / (2 * a);
            h = fabsf(h1 - humidity) < fabsf(h2 - humidity) ? h1 : h2;
        }
    }
    if (h < -5 || h > 105) {
        // no usable correction: the humidity at the sensor's own temperature
        const auto es = [](float t) { return expf(17.67 * (t - 273.15) / (t - 29.65)); };
        h = humidity * es(air_temperature) / es(sensor_temperature);
    }
    return h;
}

void CASS_HYT271::take_reading()
{
    const float h = constrain_float(raw_humidity() + HYT_BIAS[instance % ARRAY_SIZE(HYT_BIAS)]
                                    + Aircraft::rand_normal(0, 0.03), 0, 100);
    const uint16_t raw_h = uint16_t(roundf(h / 100 * 16383));
    const float t = constrain_float(sensor_temperature + Aircraft::rand_normal(0, 0.01), 233.15, 398.15);
    const uint16_t raw_t = uint16_t(roundf((t - 233.15) / 165 * 16383));
    reading[0] = (raw_h >> 8) & 0x3F;
    reading[1] = raw_h & 0xFF;
    reading[2] = raw_t >> 6;
    reading[3] = (raw_t << 2) & 0xFC;
}

void CASS_HYT271::update(const Aircraft &aircraft)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (last_coeff_ms == 0 || now_ms - last_coeff_ms > 5000) {
        last_coeff_ms = now_ms;
        load_coefficients();
    }
    float temperature, ambient_humidity, pressure;
    cass_ambient(aircraft, temperature, ambient_humidity, pressure);
    density_ratio = SURFACE_DENSITY / (pressure / (287.05 * temperature));
    const uint64_t now_us = AP_HAL::micros64();
    const float dt = (now_us - last_update_us) * 1e-6;
    const bool first = last_update_us == 0;
    follow(humidity, ambient_humidity, dt, HYT_TAU, first);
    follow(air_temperature, temperature, dt, IMET_TAU, first);
    follow(sensor_temperature, temperature + HYT_SELF_HEATING * sq(density_ratio), dt, HYT_TEMP_TAU, first);
    last_update_us = now_us;
}

int CASS_HYT271::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (data->nmsgs != 1) {
        return -1;
    }
    auto &msg = data->msgs[0];
    if (msg.flags == 0) {
        // measurement request
        request_ms = AP_HAL::millis();
        request_pending = true;
        return 0;
    }
    if (msg.flags != I2C_M_RD) {
        return -1;
    }
    bool stale = true;
    if (request_pending && AP_HAL::millis() - request_ms >= 60) {
        take_reading();
        request_pending = false;
        stale = false;
    }
    for (uint8_t i=0; i<msg.len && i<sizeof(reading); i++) {
        msg.buf[i] = reading[i];
    }
    if (stale && msg.len > 0) {
        msg.buf[0] |= 0x40;
    }
    return 0;
}

#endif  // AP_SIM_CASS_ENABLED
