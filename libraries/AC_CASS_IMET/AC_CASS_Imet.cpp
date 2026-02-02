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
    adc_thermistor = 0;
    adc_source = 0;
    buffer_idx = 0;

    // Initialize buffers and coefficients
    for (uint8_t i = 0; i < MEDIAN_WINDOW_SIZE; i++) {
        source_buffer[i] = 0;
    }
    for (uint8_t i = 0; i < 4; i++) {
        coeff[i] = 1.0f;
    }

    config = ADS1115_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
             ADS1115_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
             ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
             ADS1115_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
             ADS1115_REG_CONFIG_DR_32SPS     | // 32 SPS (~31.25ms/conversion, allows 2 per 100ms callback)
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
        // Timeout after 40ms (accommodates 32 SPS conversion time of ~31.25ms)
        if ((AP_HAL::millis() - now) > 40){
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

    // Convert bytes to a signed 16-bit value (required for differential mode)
    // Differential measurements can be negative (AIN0 < AIN1)
    int16_t raw = (int16_t)((data[0] << 8) | data[1]);
    value = (float)raw;
    return true;
}

float AC_CASS_Imet::_median_filter(float *buffer, uint8_t size)
{
    // Create a local copy to sort (avoid modifying the circular buffer)
    float sorted[MEDIAN_WINDOW_SIZE];
    for (uint8_t i = 0; i < size; i++) {
        sorted[i] = buffer[i];
    }

    // Simple insertion sort - efficient for small arrays
    for (uint8_t i = 1; i < size; i++) {
        float key = sorted[i];
        int8_t j = i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }

    // Return middle element (median)
    return sorted[size / 2];
}

void AC_CASS_Imet::_timer(void)
{
    float raw_reading;
    bool therm_healthy = false;
    bool source_healthy = false;

    // Sequential measurements at 32 SPS: both thermistor and source sampled at 10Hz
    // Each conversion takes ~31.25ms, total ~62.5ms fits within 100ms callback

    // Step 1: Read thermistor (conversion started at end of previous callback)
    therm_healthy = _read_adc(raw_reading);
    if (therm_healthy) {
        adc_thermistor = raw_reading;  // Direct assignment, no filtering needed
    }

    // Step 2: Start source conversion and wait for result
    _start_conversion(ADS1115_READ_SOURCE);
    source_healthy = _read_adc(raw_reading);
    if (source_healthy) {
        // Store in circular buffer for median filter
        source_buffer[buffer_idx % MEDIAN_WINDOW_SIZE] = raw_reading;
        buffer_idx++;

        // Apply median filter first (removes voltage spikes)
        float median_val = _median_filter(source_buffer, MEDIAN_WINDOW_SIZE);

        // Then apply EMA filter for smoothing (alpha=0.10)
        if (adc_source > 0) {
            adc_source = 0.90f * adc_source + 0.10f * median_val;
        } else {
            adc_source = median_val;
        }
    }

    // Step 3: Start thermistor conversion for next callback
    _start_conversion(ADS1115_READ_THERMISTOR);

    WITH_SEMAPHORE(_sem);                          // semaphore for access to shared frontend data
    _healthy = therm_healthy && source_healthy;    // report sensor health
    // adc_thermistor is negative (AIN0 < AIN1), adc_source is positive
    if (_healthy && adc_thermistor < 0 && adc_source > 0) {
        _calculate(adc_source, adc_thermistor);    // Calculate temperature and resistance
    }
}

void AC_CASS_Imet::_calculate(float source, float thermistor_diff)
{
    // Differential mode: thermistor_diff = AIN0 - AIN1 = -V_thermistor (negative)
    // V_thermistor_actual = -thermistor_diff
    // V_fixed = source + thermistor_diff (voltage across R_fixed = V_source - V_thermistor)
    // R_therm = R_fixed * V_thermistor / V_fixed = R_fixed * (-thermistor_diff) / (source + thermistor_diff)
    _resist = 64900.0f * (-thermistor_diff) / (source + thermistor_diff);

    // Convert to temperature (Kelvin) using Steinhart-Hart equation
    _temperature = 1.0f / (coeff[0] + coeff[1] * logf(_resist) + coeff[2] * powf(logf(_resist), 2) + coeff[3] * powf(logf(_resist), 3));
}