#pragma once

#include <AP_Param/AP_Param.h>

// 6-coefficient blocks used as sub-groups inside UserParameters, one per
// RH sensor. Each block holds the poly22 surface fit produced by the
// MATLAB calibration pipeline (SWX_calibration_surface_iT.csv) in the
// column order: c1=p00, c2=p10, c3=p01, c4=p20, c5=p11, c6=p02.
// Sub-grouping is still used because the parent UserParameters table is
// near AP_Param's 64-entry-per-group hard limit
// (AP_Param.cpp check_group_info: idx >= 64 -> FATAL).
class RHACoeffs {
public:
    static const struct AP_Param::GroupInfo var_info[];
    AP_Float c1;
    AP_Float c2;
    AP_Float c3;
    AP_Float c4;
    AP_Float c5;
    AP_Float c6;
};

class RHBCoeffs {
public:
    static const struct AP_Param::GroupInfo var_info[];
    AP_Float c1;
    AP_Float c2;
    AP_Float c3;
    AP_Float c4;
    AP_Float c5;
    AP_Float c6;
};

class RHCCoeffs {
public:
    static const struct AP_Param::GroupInfo var_info[];
    AP_Float c1;
    AP_Float c2;
    AP_Float c3;
    AP_Float c4;
    AP_Float c5;
    AP_Float c6;
};

class UserParameters {

public:
    UserParameters();
    static const struct AP_Param::GroupInfo var_info[];

    // Put accessors to your parameter variables here
    // UserCode usage example: g2.user_parameters.get_int8Param()
    // AP_Int8 get_int8Param() const { return _int8; }
    // AP_Int16 get_int16Param() const { return _int16; }
    // AP_Float get_floatParam() const { return _float; }

    //CASS custom parameters accessors
    // Weather sensors serial number
    AP_Int32 get_user_sensor1() const{return _sensor1; }
    AP_Int32 get_user_sensor2() const{return _sensor2; }
    AP_Int32 get_user_sensor3() const{return _sensor3; }
    AP_Int32 get_user_sensor4() const{return _sensor4; }
    AP_Int32 get_user_sensor5() const{return _sensor5; }
    AP_Int32 get_user_sensor6() const{return _sensor6; }
    AP_Int32 get_user_sensor7() const{return _sensor7; }
    AP_Int32 get_user_sensor8() const{return _sensor8; }
    // Weather temp sensors coefficients
    AP_Float get_user_senA_c1() const{return _senA_c1; }
    AP_Float get_user_senA_c2() const{return _senA_c2; }
    AP_Float get_user_senA_c3() const{return _senA_c3; }
    AP_Float get_user_senA_c4() const{return _senA_c4; }
    AP_Float get_user_senB_c1() const{return _senB_c1; }
    AP_Float get_user_senB_c2() const{return _senB_c2; }
    AP_Float get_user_senB_c3() const{return _senB_c3; }
    AP_Float get_user_senB_c4() const{return _senB_c4; }
    AP_Float get_user_senC_c1() const{return _senC_c1; }
    AP_Float get_user_senC_c2() const{return _senC_c2; }
    AP_Float get_user_senC_c3() const{return _senC_c3; }
    AP_Float get_user_senC_c4() const{return _senC_c4; }
    // Weather RH sensors coefficients (poly22 surface fit; mapping: c1=p00, c2=p10, c3=p01, c4=p20, c5=p11, c6=p02)
    AP_Float get_user_RHA_c1() const{return _RHA.c1; }
    AP_Float get_user_RHA_c2() const{return _RHA.c2; }
    AP_Float get_user_RHA_c3() const{return _RHA.c3; }
    AP_Float get_user_RHA_c4() const{return _RHA.c4; }
    AP_Float get_user_RHA_c5() const{return _RHA.c5; }
    AP_Float get_user_RHA_c6() const{return _RHA.c6; }
    AP_Float get_user_RHB_c1() const{return _RHB.c1; }
    AP_Float get_user_RHB_c2() const{return _RHB.c2; }
    AP_Float get_user_RHB_c3() const{return _RHB.c3; }
    AP_Float get_user_RHB_c4() const{return _RHB.c4; }
    AP_Float get_user_RHB_c5() const{return _RHB.c5; }
    AP_Float get_user_RHB_c6() const{return _RHB.c6; }
    AP_Float get_user_RHC_c1() const{return _RHC.c1; }
    AP_Float get_user_RHC_c2() const{return _RHC.c2; }
    AP_Float get_user_RHC_c3() const{return _RHC.c3; }
    AP_Float get_user_RHC_c4() const{return _RHC.c4; }
    AP_Float get_user_RHC_c5() const{return _RHC.c5; }
    AP_Float get_user_RHC_c6() const{return _RHC.c6; }
    // Wind Vane
    AP_Float get_wvane_cutoff() const{return wind_vane_cutoff; }
    AP_Float get_wvane_wsA() const{return wind_vane_wsA; }
    AP_Float get_wvane_wsB() const{return wind_vane_wsB; }
    AP_Float get_wvane_spd_tol() const{return wind_vane_spd_tol; }
    AP_Float get_wvane_enabled() const{return wind_vane_enabled; }
    AP_Float get_wvane_fs() const{return wind_vane_fs; }
    AP_Float get_wvane_offset() const{return wind_vane_offset; }
    // Battery monitor
    AP_Float get_vpbatt_enabled() const{return vpbatt_enabled; }
    AP_Float get_vpbatt_reserve() const{return vpbatt_reserve; }
    AP_Float get_vpbatt_wh() const{return vpbatt_wh; }
    AP_Float get_batt_max_curr() const{return batt_max_curr; }
    AP_Float get_batt_max_curr_timeout() const{return max_curr_timeout; }
    // Mission auto-generator
    AP_Float get_autovp_max_alt() const{return autovp_max_altitude; }
    // GPS-based Lidar activation
    AP_Float get_gpslidar_alt() const{return gpslidar_alt; }
    AP_Float get_gpslidar_hum() const{return gpslidar_hum; }
    
private:
    // Put your parameter variable definitions here
    // AP_Int8 _int8;
    // AP_Int16 _int16;
    // AP_Float _float;

    // BLISS sensor serial number
    AP_Int32 _sensor1;
    AP_Int32 _sensor2;
    AP_Int32 _sensor3;
    AP_Int32 _sensor4;
    AP_Int32 _sensor5;
    AP_Int32 _sensor6;
    AP_Int32 _sensor7;
    AP_Int32 _sensor8;

    //BLISS temp sensor coefficient
    AP_Float _senA_c1;
    AP_Float _senA_c2;
    AP_Float _senA_c3;
    AP_Float _senA_c4;
    AP_Float _senB_c1;
    AP_Float _senB_c2;
    AP_Float _senB_c3;
    AP_Float _senB_c4;
    AP_Float _senC_c1;
    AP_Float _senC_c2;
    AP_Float _senC_c3;
    AP_Float _senC_c4;

    //BLISS RH sensor coefficient
    RHACoeffs _RHA;
    RHBCoeffs _RHB;
    RHCCoeffs _RHC;


    // BLISS wind vane param ID
    AP_Float    wind_vane_cutoff;
    AP_Float    wind_vane_wsA;
    AP_Float    wind_vane_wsB;
    AP_Float    wind_vane_spd_tol;
    AP_Float    wind_vane_enabled; 
    AP_Float    wind_vane_fs;
    AP_Float    wind_vane_offset;

    // BLISS Vertical profiling smart Battery monitor params
    AP_Float    vpbatt_enabled;
    AP_Float    vpbatt_reserve;
    AP_Float    vpbatt_wh;
    AP_Float    batt_max_curr;
    AP_Float    max_curr_timeout;

    // BLISS AutoVP mission auto-generation
    AP_Float    autovp_max_altitude;

    // GPS-based Lidar activation
    AP_Float    gpslidar_alt;
    AP_Float    gpslidar_hum;

};
