#include "AC_CASS_Imet.h"
#include <utility>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

static const uint16_t mux_table[ADS1115_CHANNELS_COUNT] = {
    ADS1115_REG_CONFIG_MUX_DIFF_0_1,
    ADS1115_REG_CONFIG_MUX_DIFF_0_3,
    ADS1115_REG_CONFIG_MUX_DIFF_1_3,
    ADS1115_REG_CONFIG_MUX_DIFF_2_3,
    ADS1115_REG_CONFIG_MUX_SINGLE_0,
    ADS1115_REG_CONFIG_MUX_SINGLE_1,
    ADS1115_REG_CONFIG_MUX_SINGLE_2,
    ADS1115_REG_CONFIG_MUX_SINGLE_3 
    };

AC_CASS_Imet::AC_CASS_Imet() :
    _dev(nullptr),
    _temperature(0),
    _resist(0),
    _healthy(false)
{
}

bool AC_CASS_Imet::init(uint8_t busId, uint8_t i2cAddr)
{
    flag = false;
    adc_thermistor = 0;
    adc_source = 0;
    runs = 0;

    for(uint8_t i=0; i<3; i++){
        memset(coeff,1.0,sizeof(coeff));
    }

    config = ADS1115_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
             ADS1115_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
             ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
             ADS1115_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
             ADS1115_REG_CONFIG_DR_16SPS     | // 16 samples (conversions) per second
             ADS1115_REG_CONFIG_MODE_SINGLE  | // Single-shot mode (default)
             ADS1115_REG_CONFIG_PGA_6_144V   | // Set PGA/voltage range
             ADS1115_REG_CONFIG_OS_SINGLE;     // Start single-conversion
             
    // Bus 0 is for Pixhawk 2.1 I2C and Bus 1 is for Pixhawk 1 and PixRacer I2C
    // Check if device exists
    _dev = std::move(hal.i2c_mgr->get_device(busId, i2cAddr));
    if (!_dev) {
        return false;
    }
    _dev->get_semaphore()->take_blocking();

    _dev->set_retries(10);

    if (!_start_conversion(ADS1115_READ_SOURCE)) {
        _dev->get_semaphore()->give();
        return false;
    }

    _dev->get_semaphore()->give();
    hal.scheduler->delay(300);
    _dev->get_semaphore()->take_blocking();

    // Read the first source measurement
    _read_adc(adc_source);

    // Start a thermistor measure
    _start_conversion(ADS1115_READ_THERMISTOR);

    // lower retries for loop
    _dev->set_retries(2);

    _dev->get_semaphore()->give();

    /* Request 25Hz update */
    // Max conversion time is 12 ms
    _dev->register_periodic_callback(100000,
                                     FUNCTOR_BIND_MEMBER(&AC_CASS_Imet::_timer, void));
    return true;
}

void AC_CASS_Imet::set_i2c_addr(uint8_t addr){
    if (_dev) {
        _dev->set_address(addr);
    }
}

void AC_CASS_Imet::set_sensor_coeff(float *k){
    for(uint8_t i=0; i<4; i++){
        coeff[i] = k[i];
    }
}

bool AC_CASS_Imet::_start_conversion(uint8_t channel)
{
    // Create byte packets to be sent to the ADC
    struct PACKED {
        uint8_t reg;
        be16_t val;
    } config_pack;

    // Load packet with the desired configuration
    config_pack.reg = ADS1115_REG_POINTER_CONFIG;           // Register address
    config_pack.val = htobe16(config | mux_table[channel]); // desired configuration

    // Write
    return _dev->transfer((uint8_t *)&config_pack, sizeof(config_pack), nullptr, 0);
}

bool AC_CASS_Imet::_read_adc(float &value)
{
    uint8_t status[2];
    uint8_t data[2];

    // Check if ADC is ready to deliver, timeout if it takes too long
    uint32_t now = AP_HAL::millis();
    uint8_t cmd = ADS1115_REG_POINTER_CONFIG;   // Config. reg. address
    do{
        if (!_dev->transfer(&cmd, sizeof(cmd), status, sizeof(status))) {
            return false;
        }
        // Timeout after 10ms of no answering
        if ((AP_HAL::millis() - now) > 10){
            return false;
        }
        // Checksum to determine if the sensor is busy or not
    } while((status[0] & ADS1115_REG_CONFIG_OS_MASK) == ADS1115_REG_CONFIG_OS_BUSY);

    // Request data
    cmd = ADS1115_REG_POINTER_CONVERT; // Convert reg. address

    // Retreive data from sensor
    if (!_dev->transfer(&cmd, sizeof(cmd), data, sizeof(data))) {
        return false;
    }

    // Convert bytes to a floating point number and send
    value = (float)((data[0] << 8) | data[1]);
    return true;
}

void AC_CASS_Imet::_timer(void)
{
    // Retreive data from sensor by I2C
    float temp = adc_source;
    bool temp_healthy;
    // Collect thermistor and source measurements from the sensor by I2C
    if(flag == false){
        temp_healthy = _read_adc(adc_thermistor);
        runs += 1;
    }
    else{
        temp_healthy = _read_adc(temp);
        adc_source = 0.95f*adc_source + 0.05f*temp;
    }   

    // After 100 samples from the thermistor, re-measure voltage source and update it.
    if(runs == 100) {
        _start_conversion(ADS1115_READ_SOURCE);
        flag = true;
        runs = 0;
    }
    else{
        _start_conversion(ADS1115_READ_THERMISTOR);
        flag = false; 
    }

    WITH_SEMAPHORE(_sem);                          // semaphore for access to shared frontend data
    _healthy = temp_healthy;                       // report sensor health
    if (temp_healthy) {                            // If health is false, keep last value
        _calculate(adc_source, adc_thermistor);    // If data was collected, then calculate temperature and resistance
    }

}

void AC_CASS_Imet::_calculate(float source, float thermistor)
{
    _resist = 64900.0f * (source / thermistor - 1.0f);
    //converts to temperature (kelvin)
    _temperature = 1.0f / (coeff[0] + coeff[1] * logf(_resist) + coeff[2] * powf(logf(_resist), 2) + coeff[3] * powf(logf(_resist), 3));
}