#include "UserParameters.h"
#include "config.h"

#if USER_PARAMS_ENABLED
// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    // AP_GROUPINFO("_INT8", 0, UserParameters, _int8, 0),
    // AP_GROUPINFO("_INT16", 1, UserParameters, _int16, 0),
    // AP_GROUPINFO("_FLOAT", 2, UserParameters, _float, 0),

    //CASS custom parameters publish
    // Weather sensors
    AP_GROUPINFO("_SENSOR1", 0, UserParameters, _sensor1, 0),
    AP_GROUPINFO("_SENSOR2", 1, UserParameters, _sensor2, 0),
    AP_GROUPINFO("_SENSOR3", 2, UserParameters, _sensor3, 0),
    AP_GROUPINFO("_SENSOR4", 3, UserParameters, _sensor4, 0),
    AP_GROUPINFO("_SENSOR5", 4, UserParameters, _sensor5, 0),
    AP_GROUPINFO("_SENSOR6", 5, UserParameters, _sensor6, 0),
    AP_GROUPINFO("_SENSOR7", 6, UserParameters, _sensor7, 0),
    AP_GROUPINFO("_SENSOR8", 7, UserParameters, _sensor8, 0),
    // Wind Vane
    AP_GROUPINFO("_WV_CUTOFF", 8, UserParameters, wind_vane_cutoff, 0.06f),
    AP_GROUPINFO("_WV_WSA", 9, UserParameters, wind_vane_wsA, 30.0f),
    AP_GROUPINFO("_WV_WSB", 10, UserParameters, wind_vane_wsB, 9.0f),
    AP_GROUPINFO("_WV_SPDTOL", 11, UserParameters, wind_vane_spd_tol, 19.0f),
    AP_GROUPINFO("_WV_RTLEN", 12, UserParameters, wind_vane_enabled, 1.0f),
    AP_GROUPINFO("_WV_FS", 13, UserParameters, wind_vane_fs, 10.0f),
    AP_GROUPINFO("_WV_OFFSET", 14, UserParameters, wind_vane_offset, 0.0f),
    // VPBatt
    AP_GROUPINFO("_VPBATT_EN", 15, UserParameters, vpbatt_enabled, 1.0f),
    AP_GROUPINFO("_VPBATT_RES", 16, UserParameters, vpbatt_reserve, 30.0f),
    AP_GROUPINFO("_VPBATT_WH", 17, UserParameters, vpbatt_wh, 89.0f),
    AP_GROUPINFO("_MAX_CURR", 21, UserParameters, batt_max_curr, 45.0f),
    AP_GROUPINFO("_CURR_TOUT", 22, UserParameters, max_curr_timeout, 5.0f),
    // Mission Auto-generator
    AP_GROUPINFO("_AUTOVP_ALT", 18, UserParameters, autovp_max_altitude, 120.0f),
    // GPS-based Lidar activation
    AP_GROUPINFO("_LIDAR_ALT", 19, UserParameters, gpslidar_alt, 80.0f),
    AP_GROUPINFO("_LIDAR_HUM", 20, UserParameters, gpslidar_hum, 90.0f),
    // BLISS sensor coefficients
    AP_GROUPINFO("_SEN_A1", 23, UserParameters, _senA_c1, 10104.8989f),
    AP_GROUPINFO("_SEN_A2", 24, UserParameters, _senA_c2, 2620.50421f),
    AP_GROUPINFO("_SEN_A3", 25, UserParameters, _senA_c3, 0.0f),
    AP_GROUPINFO("_SEN_A4", 26, UserParameters, _senA_c4, 1.48891207f),
    AP_GROUPINFO("_SEN_B1", 27, UserParameters, _senB_c1, 10104.8989f),
    AP_GROUPINFO("_SEN_B2", 28, UserParameters, _senB_c2, 2620.50421f),
    AP_GROUPINFO("_SEN_B3", 29, UserParameters, _senB_c3, 0.0f),
    AP_GROUPINFO("_SEN_B4", 30, UserParameters, _senB_c4, 1.48891207f),
    AP_GROUPINFO("_SEN_C1", 31, UserParameters, _senC_c1, 10104.8989f),
    AP_GROUPINFO("_SEN_C2", 32, UserParameters, _senC_c2, 2620.50421f),
    AP_GROUPINFO("_SEN_C3", 33, UserParameters, _senC_c3, 0.0f),
    AP_GROUPINFO("_SEN_C4", 34, UserParameters, _senC_c4, 1.48891207f),
    
    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
#endif // USER_PARAMS_ENABLED
