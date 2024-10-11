#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Vehicle/AP_Vehicle.h>

//#define HYT271_DEFAULT_ADDR 0x10

class AC_CASS_HYT271 {
public:
    AC_CASS_HYT271(void);
    ~AC_CASS_HYT271(void){}

    bool init(uint8_t busId, uint8_t i2cAddr);
    float relative_humidity(void) { return _humidity; } // temperature in kelvin
    float temperature(void) { return _temperature; }   // voltage read by the ADCS
    bool healthy(void) { return _healthy; } // do we have a valid temperature reading?
    void set_i2c_addr(uint8_t addr);

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    HAL_Semaphore _sem; // semaphore for access to shared frontend data
    float _temperature; // degrees K
    float _humidity; //voltage read by the ADC
    bool _healthy; // we have a valid temperature reading to report
    bool _measure(void);
    bool _collect(float &hum, float &temp);
    void _timer(void); // update the temperature, called at 20Hz
};