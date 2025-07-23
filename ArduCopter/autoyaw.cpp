#include "Copter.h"

Mode::AutoYaw Mode::auto_yaw;

// roi_yaw - returns heading towards location held in roi
float Mode::AutoYaw::roi_yaw() const
{
    return get_bearing_cd(copter.inertial_nav.get_position_xy_cm(), roi.xy());
}

// returns a yaw in degrees, direction of vehicle travel:
float Mode::AutoYaw::look_ahead_yaw()
{
    const Vector3f& vel = copter.inertial_nav.get_velocity_neu_cms();
    const float speed_sq = vel.xy().length_squared();
    // Commanded Yaw to automatically look ahead.
    if (copter.position_ok() && (speed_sq > (YAW_LOOK_AHEAD_MIN_SPEED * YAW_LOOK_AHEAD_MIN_SPEED))) {
        _look_ahead_yaw = degrees(atan2f(vel.y,vel.x));
    }
    return _look_ahead_yaw;
}

// CASS wind function that sends latest wind estimation to the autopilot
// It will only track the wind if its horizontally stationary 
float Mode::AutoYaw::turn_into_wind()
{
    return copter.cass_wind_direction;
}

float Mode::AutoYaw::turn_into_wind_CT2()
{
    float _wind_dir = copter.cass_wind_direction;      // Estimated wind direction in centi-degrees
    float _wind_spd = copter.cass_wind_speed;           // Estimated wind speed in m/s
    float _xy_speed = copter.wp_nav->get_default_speed_xy();    // Copters target speed in cm/s
    float _wp_bearing = copter.wp_nav->get_yaw();       // Bearing for the next waypoint in centi-degrees
    float w_x, w_y, s_x, s_y;
    if(copter.position_ok()){
        if (copter.wp_nav->reached_wp_destination() || fabsf(copter.inertial_nav.get_velocity_z_up_cms()) > 80){
            _wind_CT2_yaw = _wind_dir;
        } else {
            s_x = _xy_speed*sinf(_wp_bearing*1.745e-4f);
            s_y = _xy_speed*cosf(_wp_bearing*1.745e-4f);
            w_x = 100*_wind_spd*sinf(_wind_dir*1.745e-4f);
            w_y = 100*_wind_spd*cosf(_wind_dir*1.745e-4f);
            _wind_CT2_yaw = wrap_360_cd(atan2f((s_x + w_x),(s_y + w_y))*5729.6f);
        }
    }
    return _wind_CT2_yaw;
}

void Mode::AutoYaw::set_mode_to_default(bool rtl)
{
    set_mode(default_mode(rtl));
}

// default_mode - returns auto_yaw.mode() based on WP_YAW_BEHAVIOR parameter
// set rtl parameter to true if this is during an RTL
Mode::AutoYaw::Mode Mode::AutoYaw::default_mode(bool rtl) const
{
    switch (copter.g.wp_yaw_behavior) {

    case WP_YAW_BEHAVIOR_NONE:
        return Mode::HOLD;

    case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL:
        if (rtl) {
            return Mode::HOLD;
        } else {
            return Mode::LOOK_AT_NEXT_WP;
        }

    case WP_YAW_BEHAVIOR_LOOK_AHEAD:
        return Mode::LOOK_AHEAD;
    
    //CASS implementation of wind tracker for vertical profiles
    case WP_YAW_BEHAVIOR_INTO_WIND:
        return Mode::AUTO_YAW_INTO_WIND;

    //CASS implementation of wind tracker for CT2 profiles
    case WP_YAW_BEHAVIOR_WIND_CT2:
        return Mode::AUTO_YAW_WIND_CT2;

    case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP:
    default:
        return Mode::LOOK_AT_NEXT_WP;
    }
}

// set_mode - sets the yaw mode for auto
void Mode::AutoYaw::set_mode(Mode yaw_mode)
{
    // return immediately if no change
    if (_mode == yaw_mode) {
        return;
    }
    _last_mode = _mode;
    _mode = yaw_mode;

    // perform initialisation
    switch (_mode) {

    case Mode::HOLD:
        break;

    case Mode::LOOK_AT_NEXT_WP:
        // wpnav will initialise heading when wpnav's set_destination method is called
        break;

    case Mode::ROI:
        // look ahead until we know otherwise
        break;

    case Mode::FIXED:
        // keep heading pointing in the direction held in fixed_yaw
        // caller should set the fixed_yaw
        break;

    case Mode::LOOK_AHEAD:
        // Commanded Yaw to automatically look ahead.
        _look_ahead_yaw = copter.ahrs.yaw_sensor * 0.01;  // cdeg -> deg
        break;

    case Mode::RESETTOARMEDYAW:
        // initial_armed_bearing will be set during arming so no init required
        break;

    case Mode::ANGLE_RATE:
        break;

    case Mode::RATE:
        // initialise target yaw rate to zero
        _yaw_rate_cds = 0.0;
        break;

    case Mode::CIRCLE:
    case Mode::PILOT_RATE:
    case Mode::WEATHERVANE:
        // no initialisation required
        break;
    
    case Mode::AUTO_YAW_INTO_WIND:
        // CASS: initialise target _wind_yaw on boot-up
        _wind_yaw = copter.ahrs.yaw_sensor;
        break;
    case Mode::AUTO_YAW_WIND_CT2:
        // CASS: initialise target _wind_CT2_yaw on boot-up
        _wind_CT2_yaw = copter.ahrs.yaw_sensor;
        break;

    }
}

// set_fixed_yaw - sets the yaw look at heading for auto mode
void Mode::AutoYaw::set_fixed_yaw(float angle_deg, float turn_rate_ds, int8_t direction, bool relative_angle)
{
    _last_update_ms = millis();

    // calculate final angle as relative to vehicle heading or absolute
    if (relative_angle) {
        if (_mode == Mode::HOLD) {
            _yaw_angle_cd = copter.ahrs.yaw_sensor;
        }
        _fixed_yaw_offset_cd = angle_deg * 100.0 * (direction >= 0 ? 1.0 : -1.0);
    } else {
        // absolute angle
        _fixed_yaw_offset_cd = wrap_180_cd(angle_deg * 100.0 - _yaw_angle_cd);
        if (direction < 0 && is_positive(_fixed_yaw_offset_cd)) {
            _fixed_yaw_offset_cd -= 36000.0;
        } else if (direction > 0 && is_negative(_fixed_yaw_offset_cd)) {
            _fixed_yaw_offset_cd += 36000.0;
        }
    }

    // get turn speed
    if (!is_positive(turn_rate_ds)) {
        // default to default slew rate
        _fixed_yaw_slewrate_cds = copter.attitude_control->get_slew_yaw_max_degs() * 100.0;
    } else {
        _fixed_yaw_slewrate_cds = MIN(copter.attitude_control->get_slew_yaw_max_degs(), turn_rate_ds) * 100.0;
    }

    // set yaw mode
    set_mode(Mode::FIXED);
}

// set_fixed_yaw - sets the yaw look at heading for auto mode
void Mode::AutoYaw::set_yaw_angle_rate(float yaw_angle_d, float yaw_rate_ds)
{
    _last_update_ms = millis();

    _yaw_angle_cd = yaw_angle_d * 100.0;
    _yaw_rate_cds = yaw_rate_ds * 100.0;

    // set yaw mode
    set_mode(Mode::ANGLE_RATE);
}

// set_yaw_angle_offset - sets the yaw look at heading for auto mode, as an offset from the current yaw angle
void Mode::AutoYaw::set_yaw_angle_offset(const float yaw_angle_offset_d)
{
    _last_update_ms = millis();

    _yaw_angle_cd = wrap_360_cd(_yaw_angle_cd + (yaw_angle_offset_d * 100.0));
    _yaw_rate_cds = 0.0f;

    // set yaw mode
    set_mode(Mode::ANGLE_RATE);
}

// set_roi - sets the yaw to look at roi for auto mode
void Mode::AutoYaw::set_roi(const Location &roi_location)
{
    // if location is zero lat, lon and altitude turn off ROI
    if (roi_location.alt == 0 && roi_location.lat == 0 && roi_location.lng == 0) {
        // set auto yaw mode back to default assuming the active command is a waypoint command.  A more sophisticated method is required to ensure we return to the proper yaw control for the active command
        auto_yaw.set_mode_to_default(false);
#if HAL_MOUNT_ENABLED
        // switch off the camera tracking if enabled
        copter.camera_mount.clear_roi_target();
#endif  // HAL_MOUNT_ENABLED
    } else {
#if HAL_MOUNT_ENABLED
        // check if mount type requires us to rotate the quad
        if (!copter.camera_mount.has_pan_control()) {
            if (roi_location.get_vector_from_origin_NEU(roi)) {
                auto_yaw.set_mode(Mode::ROI);
            }
        }
        // send the command to the camera mount
        copter.camera_mount.set_roi_target(roi_location);

        // TO-DO: expand handling of the do_nav_roi to support all modes of the MAVLink.  Currently we only handle mode 4 (see below)
        //      0: do nothing
        //      1: point at next waypoint
        //      2: point at a waypoint taken from WP# parameter (2nd parameter?)
        //      3: point at a location given by alt, lon, lat parameters
        //      4: point at a target given a target id (can't be implemented)
#else
        // if we have no camera mount aim the quad at the location
        if (roi_location.get_vector_from_origin_NEU(roi)) {
            auto_yaw.set_mode(Mode::ROI);
        }
#endif  // HAL_MOUNT_ENABLED
    }
}

// set auto yaw rate in centi-degrees per second
void Mode::AutoYaw::set_rate(float turn_rate_cds)
{
    set_mode(Mode::RATE);
    _yaw_rate_cds = turn_rate_cds;
}

// return true if fixed yaw target has been reached
bool Mode::AutoYaw::reached_fixed_yaw_target()
{
    if (mode() != Mode::FIXED) {
        // should not happen, not in the right mode
        return true;
    }

    if (!is_zero(_fixed_yaw_offset_cd)) {
        // still slewing yaw target
        return false;
    }

    // Within 2 deg of target
    return (fabsf(wrap_180_cd(copter.ahrs.yaw_sensor-_yaw_angle_cd)) <= 200);
}

// yaw_cd - returns target heading depending upon auto_yaw.mode()
float Mode::AutoYaw::yaw_cd()
{
    switch (_mode) {

    case Mode::ROI:
        // point towards a location held in roi
        _yaw_angle_cd = roi_yaw();
        break;

    case Mode::FIXED: {
        // keep heading pointing in the direction held in fixed_yaw
        // with no pilot input allowed
        const uint32_t now_ms = millis();
        float dt = (now_ms - _last_update_ms) * 0.001;
        _last_update_ms = now_ms;
        float yaw_angle_step = constrain_float(_fixed_yaw_offset_cd, - dt * _fixed_yaw_slewrate_cds, dt * _fixed_yaw_slewrate_cds);
        _fixed_yaw_offset_cd -= yaw_angle_step;
        _yaw_angle_cd += yaw_angle_step;
        break;
    }

    case Mode::LOOK_AHEAD:
        // Commanded Yaw to automatically look ahead.
        _yaw_angle_cd = look_ahead_yaw() * 100.0;
        break;

    case Mode::RESETTOARMEDYAW:
        // changes yaw to be same as when quad was armed
        _yaw_angle_cd = copter.initial_armed_bearing;
        break;

    case Mode::AUTO_YAW_INTO_WIND:
        // CASS implementation of wind tracker, send estimated wind dir = yaw command to autopilot
        return turn_into_wind();
    
    case Mode::AUTO_YAW_WIND_CT2:
        // CASS implementation of wind CT2 tracker, send optimal heading = yaw command to autopilot
        return turn_into_wind_CT2();

    case Mode::CIRCLE:
#if MODE_CIRCLE_ENABLED
        if (copter.circle_nav->is_active()) {
            _yaw_angle_cd = copter.circle_nav->get_yaw();
        }
#endif
        break;

    case Mode::ANGLE_RATE:{
        const uint32_t now_ms = millis();
        float dt = (now_ms - _last_update_ms) * 0.001;
        _last_update_ms = now_ms;
        _yaw_angle_cd += _yaw_rate_cds * dt;
        break;
    }

    case Mode::RATE:
    case Mode::WEATHERVANE:
    case Mode::PILOT_RATE:
        _yaw_angle_cd = copter.attitude_control->get_att_target_euler_cd().z;
        break;

    case Mode::LOOK_AT_NEXT_WP:
    default:
        // point towards next waypoint.
        // we don't use wp_bearing because we don't want the copter to turn too much during flight
        _yaw_angle_cd = copter.pos_control->get_yaw_cd();
    break;
    }
    
    return _yaw_angle_cd;
}

// returns yaw rate normally set by SET_POSITION_TARGET mavlink
// messages (positive is clockwise, negative is counter clockwise)
float Mode::AutoYaw::rate_cds()
{
    switch (_mode) {

    case Mode::HOLD:
    case Mode::ROI:
    case Mode::FIXED:
    case Mode::LOOK_AHEAD:
    case Mode::RESETTOARMEDYAW:
    case Mode::CIRCLE:
        _yaw_rate_cds = 0.0f;
        break;

    case Mode::LOOK_AT_NEXT_WP:
        _yaw_rate_cds = copter.pos_control->get_yaw_rate_cds();
        break;

    case Mode::PILOT_RATE:
        _yaw_rate_cds = _pilot_yaw_rate_cds;
        break;

    case Mode::ANGLE_RATE:
    case Mode::RATE:
    case Mode::WEATHERVANE:
        break;
    case Mode::AUTO_YAW_INTO_WIND:
    case Mode::AUTO_YAW_WIND_CT2:
        return _yaw_rate_cds;
    }

    // return zero turn rate (this should never happen)
    return _yaw_rate_cds;
}

AC_AttitudeControl::HeadingCommand Mode::AutoYaw::get_heading()
{
    // process pilot's yaw input
    _pilot_yaw_rate_cds = 0.0;
    if (!copter.failsafe.radio && copter.flightmode->use_pilot_yaw()) {
        // get pilot's desired yaw rate
        _pilot_yaw_rate_cds = copter.flightmode->get_pilot_desired_yaw_rate();
        if (!is_zero(_pilot_yaw_rate_cds)) {
            auto_yaw.set_mode(AutoYaw::Mode::PILOT_RATE);
        }
    } else if (auto_yaw.mode() == AutoYaw::Mode::PILOT_RATE) {
        // RC failsafe, or disabled make sure not in pilot control
        auto_yaw.set_mode(AutoYaw::Mode::HOLD);
    }

#if WEATHERVANE_ENABLED
    update_weathervane(_pilot_yaw_rate_cds);
#endif

    AC_AttitudeControl::HeadingCommand heading;
    heading.yaw_angle_cd = auto_yaw.yaw_cd();
    heading.yaw_rate_cds = auto_yaw.rate_cds();

    switch (auto_yaw.mode()) {
        case Mode::HOLD:
        case Mode::RATE:
        case Mode::PILOT_RATE:
        case Mode::WEATHERVANE:
            heading.heading_mode = AC_AttitudeControl::HeadingMode::Rate_Only;
            break;
        case Mode::LOOK_AT_NEXT_WP:
        case Mode::ROI:
        case Mode::FIXED:
        case Mode::LOOK_AHEAD:
        case Mode::RESETTOARMEDYAW:
        case Mode::ANGLE_RATE:
        case Mode::CIRCLE:
        case Mode::AUTO_YAW_INTO_WIND:
        case Mode::AUTO_YAW_WIND_CT2:
            heading.heading_mode = AC_AttitudeControl::HeadingMode::Angle_And_Rate;
            break;
    }

    return heading;
}

// handle the interface to the weathervane library
// pilot_yaw can be an angle or a rate or rcin from yaw channel. It just needs to represent a pilot's request to yaw the vehicle to enable pilot overrides.
#if WEATHERVANE_ENABLED
void Mode::AutoYaw::update_weathervane(const int16_t pilot_yaw_cds)
{
    if (!copter.flightmode->allows_weathervaning()) {
        return;
    }

    float yaw_rate_cds;
    if (copter.g2.weathervane.get_yaw_out(yaw_rate_cds, pilot_yaw_cds, copter.flightmode->get_alt_above_ground_cm()*0.01,
                                                                       copter.pos_control->get_roll_cd()-copter.attitude_control->get_roll_trim_cd(),
                                                                       copter.pos_control->get_pitch_cd(),
                                                                       copter.flightmode->is_taking_off(),
                                                                       copter.flightmode->is_landing())) {
        set_mode(Mode::WEATHERVANE);
        _yaw_rate_cds = yaw_rate_cds;
        return;
    }

    // if the weathervane controller has previously been activated we need to ensure we return control back to what was previously set
    if (mode() == Mode::WEATHERVANE) {
        _yaw_rate_cds = 0.0;
        if (_last_mode == Mode::HOLD) {
            set_mode_to_default(false);
        } else {
            set_mode(_last_mode);
        }
    }
}
#endif // WEATHERVANE_ENABLED
