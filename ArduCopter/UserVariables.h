// user defined variables

// example variables used in Wii camera testing - replace with your own
// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES

//#define LOG_IMET_MSG 0x29 // SD Card Memory Direction for Temp sensor
//#define LOG_RH_MSG 0x30 // SD Card Memory Direction for RH sensor
//#define LOG_WIND_MSG 0x31 //SD Card Memory Direction for our Wind estimation

struct PACKED log_RH {
    LOG_PACKET_HEADER;
    uint64_t time_stamp;
    uint8_t healthy1;
    uint8_t healthy2;
    uint8_t healthy3;
    uint8_t healthy4;
    float humidity1;
    float humidity2;
    float humidity3;
    float humidity4;
    float RHtemp1;
    float RHtemp2;
    float RHtemp3;
    float RHtemp4;
};

struct PACKED log_IMET {
    LOG_PACKET_HEADER;
    uint64_t time_stamp;
    uint8_t fan_status;
    uint8_t healthy1;
    uint8_t healthy2;
    uint8_t healthy3;
    uint8_t healthy4;
    float temperature1;
    float temperature2;
    float temperature3;
    float temperature4;
    float resist1;
    float resist2;
    float resist3;
    float resist4;
};

struct PACKED log_WIND {
    LOG_PACKET_HEADER;
    uint64_t time_stamp;
    float _wind_dir;
    float _wind_speed;
    float _R13;
    float _R23;
    float _R33;
};

float cass_wind_direction;
float cass_wind_speed;

#endif  // USERHOOK_VARIABLES