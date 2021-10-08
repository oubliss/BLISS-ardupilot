#include "UserParameters.h"

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
    // Mission Auto-generator
    AP_GROUPINFO("_AUTOVP_ALT", 18, UserParameters, autovp_max_altitude, 120.0f),
    
    AP_GROUPEND
};
