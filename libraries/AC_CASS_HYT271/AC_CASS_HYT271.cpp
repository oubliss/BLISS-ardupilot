#include "AC_CASS_HYT271.h"
#include <utility>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

AC_CASS_HYT271::AC_CASS_HYT271() :
    _dev(nullptr),
    _temperature(0),
    _humidity(0),
    _humidity_corrected(0),
    _iT(0),
    _healthy(false)
{
}

bool AC_CASS_HYT271::init(uint8_t busId, uint8_t i2cAddr)
{
    // Bus 0 is for Pixhawk 2.1 I2C and Bus 1 is for Pixhawk 1 and PixRacer I2C
    // Check if device exists
    _dev = std::move(hal.i2c_mgr->get_device(busId, i2cAddr));
    if (!_dev) {
        return false;
    }
    _dev->get_semaphore()->take_blocking();

    _dev->set_retries(10);

    // Start the first measurement
    if (!_measure()) {
        _dev->get_semaphore()->give();
        return false;
    }

    // lower retries for run
    _dev->set_retries(3);

    _dev->get_semaphore()->give();

    // Register 10Hz periodic callback (100ms interval)
    // HYT271 max conversion time is ~60ms, so 100ms provides adequate margin
    _dev->register_periodic_callback(100000,
                                     FUNCTOR_BIND_MEMBER(&AC_CASS_HYT271::_timer, void));
    return true;
}

void AC_CASS_HYT271::set_i2c_addr(uint8_t addr){
    if (_dev) {
        _dev->set_address(addr);
    }
}

void AC_CASS_HYT271::set_sensor_coeff(float *k){
    for(uint8_t i=0; i<6; i++){
        coeff[i] = k[i];
    }
}

void AC_CASS_HYT271::set_iT(float iT)
{
    WITH_SEMAPHORE(_sem);
    _iT = iT;
}

bool AC_CASS_HYT271::_measure()
{
    uint8_t cmd = 0x00;
        if (!_dev->transfer(&cmd, 1, nullptr, 0)) {
            return false;
        }
        return true;
}

bool AC_CASS_HYT271::_collect(float &hum, float &hum_corr, float &temp)
{
    uint8_t data[4];
    int16_t raw;
    // Read sensors
    if (!_dev->transfer(nullptr, 0, data, sizeof(data))) {
        return false;
    }

    // Verify data with the checksum
    if ((data[0] & 0x40) == 0x40){
        return false;
    }

    WITH_SEMAPHORE(_sem);                           // semaphore for access to shared frontend data

    // Extract 14-bit humidity value (bits 13:0 of data[0:1])
    raw = ((data[0] << 8) | data[1]) & 0x3FFF;
    // Convert to relative humidity: 0-100% (scale factor = 100.0 / 16383.0)
    hum = 0.00610388f * (float)raw;

    if (_iT >= 200.0f) {
        // poly22 surface fit: f(h,T) = p00 + p10*h + p01*T + p20*h^2 + p11*h*T + p02*T^2
        // Coefficient layout matches SWX_calibration_surface_iT.csv column order.
        const float h = hum;
        const float T = _iT;
        hum_corr = coeff[0]
                 + coeff[1]*h
                 + coeff[2]*T
                 + coeff[3]*h*h
                 + coeff[4]*h*T
                 + coeff[5]*T*T;
        hum_corr = constrain_float(hum_corr, 0.0f, 100.0f);
    } else {
        hum_corr = hum;   // no valid iT yet — fall back to raw humidity
    }

    // Extract 14-bit temperature value (bits 15:2 of data[2:3])
    raw = (data[2] << 6) | (data[3] >> 2);
    // Convert to Kelvin: 233.15K to 398.15K (-40C to +125C) (scale factor = 165.0 / 16383.0)
    temp = 0.01007141f * (float)raw + 233.15f;

    return true;  
}

void AC_CASS_HYT271::_timer(void)
{
    // Two-phase measurement cycle:
    // 1. Collect data from previous measurement request
    // 2. Start new measurement for next callback
    _healthy = _collect(_humidity, _humidity_corrected, _temperature);
    _measure();
}