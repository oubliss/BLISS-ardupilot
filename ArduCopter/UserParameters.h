#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters() {}
    static const struct AP_Param::GroupInfo var_info[];
    
    // Put accessors to your parameter variables here
    // UserCode usage example: g2.user_parameters.get_int8Param()
    // AP_Int8 get_int8Param() const { return _int8; }
    // AP_Int16 get_int16Param() const { return _int16; }
    // AP_Float get_floatParam() const { return _float; }

    //CASS custom parameters accessors
    // Weather sensors
    AP_Int32 get_user_sensor1() const{return _sensor1; }
    AP_Int32 get_user_sensor2() const{return _sensor2; }
    AP_Int32 get_user_sensor3() const{return _sensor3; }
    AP_Int32 get_user_sensor4() const{return _sensor4; }
    AP_Int32 get_user_sensor5() const{return _sensor5; }
    AP_Int32 get_user_sensor6() const{return _sensor6; }
    AP_Int32 get_user_sensor7() const{return _sensor7; }
    AP_Int32 get_user_sensor8() const{return _sensor8; }
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
    // Mission auto-generator
    AP_Float get_autovp_max_alt() const{return autovp_max_altitude; }
    
private:
    // Put your parameter variable definitions here
    // AP_Int8 _int8;
    // AP_Int16 _int16;
    // AP_Float _float;

    // CASS custom parameters
    AP_Int32 _sensor1;
    AP_Int32 _sensor2;
    AP_Int32 _sensor3;
    AP_Int32 _sensor4;
    AP_Int32 _sensor5;
    AP_Int32 _sensor6;
    AP_Int32 _sensor7;
    AP_Int32 _sensor8;

    //CASS wind vane param ID
    AP_Float    wind_vane_cutoff;
    AP_Float    wind_vane_wsA;
    AP_Float    wind_vane_wsB;
    AP_Float    wind_vane_spd_tol;
    AP_Float    wind_vane_enabled; 
    AP_Float    wind_vane_fs;
    AP_Float    wind_vane_offset;

    //CASS Vertical profiling smart Battery monitor params
    AP_Float    vpbatt_enabled;
    AP_Float    vpbatt_reserve;
    AP_Float    vpbatt_wh;

    //CASS AutoVP mission auto-generation
    AP_Float    autovp_max_altitude;
};
