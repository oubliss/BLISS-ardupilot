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
    // BLISS temp sensor coefficients
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
    // BLISS RH sensor coefficients are nested in sub-groups: the
    // parent table would otherwise approach AP_Param's 64-entry hard
    // limit. Each block holds the 6 poly22 coefficients in one entry.
    // User-visible names are USR_RH_A1..USR_RH_C6.
    AP_SUBGROUPINFO(_RHA, "_RH_A", 35, UserParameters, RHACoeffs),
    AP_SUBGROUPINFO(_RHB, "_RH_B", 36, UserParameters, RHBCoeffs),
    AP_SUBGROUPINFO(_RHC, "_RH_C", 37, UserParameters, RHCCoeffs),

    AP_GROUPEND
};

// Defaults are the identity transform (scaled by 1e7 to match the *1e-7f
// reader convention in Copter::init_CASS_hyt271). c2 stores p10 = 1, so
// an uncalibrated sensor with valid iT returns raw RH unchanged. Push
// calibration values from SWX_calibration_surface_iT.csv (_x1e7 rows).
const AP_Param::GroupInfo RHACoeffs::var_info[] = {
    AP_GROUPINFO("1", 0, RHACoeffs, c1, 0.0f),         // p00
    AP_GROUPINFO("2", 1, RHACoeffs, c2, 10000000.0f),  // p10 = 1 (identity)
    AP_GROUPINFO("3", 2, RHACoeffs, c3, 0.0f),         // p01
    AP_GROUPINFO("4", 3, RHACoeffs, c4, 0.0f),         // p20
    AP_GROUPINFO("5", 4, RHACoeffs, c5, 0.0f),         // p11
    AP_GROUPINFO("6", 5, RHACoeffs, c6, 0.0f),         // p02
    AP_GROUPEND
};

const AP_Param::GroupInfo RHBCoeffs::var_info[] = {
    AP_GROUPINFO("1", 0, RHBCoeffs, c1, 0.0f),         // p00
    AP_GROUPINFO("2", 1, RHBCoeffs, c2, 10000000.0f),  // p10 = 1 (identity)
    AP_GROUPINFO("3", 2, RHBCoeffs, c3, 0.0f),         // p01
    AP_GROUPINFO("4", 3, RHBCoeffs, c4, 0.0f),         // p20
    AP_GROUPINFO("5", 4, RHBCoeffs, c5, 0.0f),         // p11
    AP_GROUPINFO("6", 5, RHBCoeffs, c6, 0.0f),         // p02
    AP_GROUPEND
};

const AP_Param::GroupInfo RHCCoeffs::var_info[] = {
    AP_GROUPINFO("1", 0, RHCCoeffs, c1, 0.0f),         // p00
    AP_GROUPINFO("2", 1, RHCCoeffs, c2, 10000000.0f),  // p10 = 1 (identity)
    AP_GROUPINFO("3", 2, RHCCoeffs, c3, 0.0f),         // p01
    AP_GROUPINFO("4", 3, RHCCoeffs, c4, 0.0f),         // p20
    AP_GROUPINFO("5", 4, RHCCoeffs, c5, 0.0f),         // p11
    AP_GROUPINFO("6", 5, RHCCoeffs, c6, 0.0f),         // p02
    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
#endif // USER_PARAMS_ENABLED
