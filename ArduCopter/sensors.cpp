#include "Copter.h"

void Copter::init_CASS_imet(){

    float coeff[4][4];

    //CS3D SENSORS (dummy values)
    //IMET temp number 57560:
    coeff[0][0] = g2.user_parameters.get_user_senA_c1()*1e-7f;
    coeff[0][1] = g2.user_parameters.get_user_senA_c2()*1e-7f;
    coeff[0][2] = g2.user_parameters.get_user_senA_c3()*1e-7f;
    coeff[0][3] = g2.user_parameters.get_user_senA_c4()*1e-7f;

    //IMET temp number 57551:
    coeff[1][0] = g2.user_parameters.get_user_senB_c1()*1e-7f;
    coeff[1][1] = g2.user_parameters.get_user_senB_c2()*1e-7f;
    coeff[1][2] = g2.user_parameters.get_user_senB_c3()*1e-7f;
    coeff[1][3] = g2.user_parameters.get_user_senB_c4()*1e-7f;

    //IMET temp number 57558:
    coeff[2][0] = g2.user_parameters.get_user_senC_c1()*1e-7f;
    coeff[2][1] = g2.user_parameters.get_user_senC_c2()*1e-7f;
    coeff[2][2] = g2.user_parameters.get_user_senC_c3()*1e-7f;
    coeff[2][3] = g2.user_parameters.get_user_senC_c4()*1e-7f;

    //IMET temp number none:
    coeff[3][0] = 9.19528749e-04f;
    coeff[3][1] = 2.89652159e-04f;
    coeff[3][2] = -2.78103495e-06f;
    coeff[3][3] = 2.40523976e-07f;
    
    // Initialize and set I2C addresses
    uint8_t deafult_i2cAddr = 0x48;
    uint8_t busId = 0;
    for(uint8_t i=0; i<4; i++){
        CASS_Imet[i].init(busId,deafult_i2cAddr + i);
    }
    // Set sensor coefficients
    CASS_Imet[0].set_sensor_coeff(coeff[0]);
    CASS_Imet[1].set_sensor_coeff(coeff[1]);
    CASS_Imet[2].set_sensor_coeff(coeff[2]);
    CASS_Imet[3].set_sensor_coeff(coeff[3]);
}

void Copter::init_CASS_hyt271(){
    // Initialize and set I2C addresses
    uint8_t deafult_i2cAddr = 0x10;
    uint8_t busId = 0;
    for(uint8_t i=0; i<4; i++){
        CASS_HYT271[i].init(busId,deafult_i2cAddr + i);
    }
}

// return barometric altitude in centimeters
void Copter::read_barometer(void)
{
    barometer.update();

    baro_alt = barometer.get_altitude() * 100.0f;
}

#if AP_RANGEFINDER_ENABLED
void Copter::init_rangefinder(void)
{
   rangefinder.set_log_rfnd_bit(MASK_LOG_CTUN);
   rangefinder.init(ROTATION_PITCH_270);
   rangefinder_state.alt_cm_filt.set_cutoff_frequency(g2.rangefinder_filt);
   rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);

   // upward facing range finder
   rangefinder_up_state.alt_cm_filt.set_cutoff_frequency(g2.rangefinder_filt);
   rangefinder_up_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_90);
}

// return rangefinder altitude in centimeters
void Copter::read_rangefinder(void)
{
    rangefinder.update();

    rangefinder_state.update();
    rangefinder_up_state.update();

#if HAL_PROXIMITY_ENABLED
    if (rangefinder_state.enabled_and_healthy() || rangefinder_state.data_stale()) {
        g2.proximity.set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());
    }
#endif
}
#endif  // AP_RANGEFINDER_ENABLED

bool print_flag = true;
// return true if rangefinder_alt can be used
bool Copter::rangefinder_alt_ok() const
{
    // BLISS modified Lidar health diagnosis including ambient humidity
    Location curr_loc;
    int32_t curr_alt_cm;

    // Get altitude wrt home
    if(!copter.ahrs.get_location(curr_loc)){
        return rangefinder_state.enabled_and_healthy();
    }

    if(!curr_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, curr_alt_cm)){
        return rangefinder_state.enabled_and_healthy();
    }

    // Get humidity
    float avg_hum = 0;
    float N = 0;
    for(int8_t i = 0; i < 4; i++){
        if(copter.CASS_HYT271[i].healthy()){
            avg_hum += copter.CASS_HYT271[i].relative_humidity();
            N++;
        }
    }
    if(N > 0){
        avg_hum /= N;
    }
    else{
        avg_hum = 0;
    }

    bool no_lidar = curr_alt_cm > g2.user_parameters.get_gpslidar_alt()*100.0f || avg_hum > g2.user_parameters.get_gpslidar_hum();

    if(g2.user_parameters.get_gpslidar_alt() < 1.0f && g2.user_parameters.get_gpslidar_hum() < 1.0f){
        return rangefinder_state.enabled_and_healthy();
    }
    else if(no_lidar){
        if (avg_hum > g2.user_parameters.get_gpslidar_hum() && !print_flag){
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "High humidity: Lidar disabled");
            print_flag = true;
        }
        return rangefinder_state.enabled_and_healthy() && !no_lidar;
    }
    else{
        if (rangefinder_state.enabled_and_healthy() && print_flag && !no_lidar){
            copter.gcs().send_text(MAV_SEVERITY_INFO, "Lidar enabled");
            print_flag = false;
        }
        return rangefinder_state.enabled_and_healthy() && !no_lidar;
    }
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_up_ok() const
{
    return rangefinder_up_state.enabled_and_healthy();
}

// update rangefinder based terrain offset
// terrain offset is the terrain's height above the EKF origin
void Copter::update_rangefinder_terrain_offset()
{
    float terrain_offset_cm = rangefinder_state.inertial_alt_cm - rangefinder_state.alt_cm_glitch_protected;
    rangefinder_state.terrain_offset_cm += (terrain_offset_cm - rangefinder_state.terrain_offset_cm) * (copter.G_Dt / MAX(copter.g2.surftrak_tc, copter.G_Dt));

    terrain_offset_cm = rangefinder_up_state.inertial_alt_cm + rangefinder_up_state.alt_cm_glitch_protected;
    rangefinder_up_state.terrain_offset_cm += (terrain_offset_cm - rangefinder_up_state.terrain_offset_cm) * (copter.G_Dt / MAX(copter.g2.surftrak_tc, copter.G_Dt));

    if (rangefinder_state.alt_healthy || rangefinder_state.data_stale()) {
        wp_nav->set_rangefinder_terrain_offset(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.terrain_offset_cm);
#if MODE_CIRCLE_ENABLED
        circle_nav->set_rangefinder_terrain_offset(rangefinder_state.enabled && wp_nav->rangefinder_used(), rangefinder_state.alt_healthy, rangefinder_state.terrain_offset_cm);
#endif
    }
}

// helper function to get inertially interpolated rangefinder height.
bool Copter::get_rangefinder_height_interpolated_cm(int32_t& ret) const
{
#if AP_RANGEFINDER_ENABLED
    return rangefinder_state.get_rangefinder_height_interpolated_cm(ret);
#else
    return false;
#endif
}
