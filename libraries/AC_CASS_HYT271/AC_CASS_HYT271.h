/*
 * AC_CASS_HYT271 - I2C Humidity and Temperature Sensor Driver
 * ============================================================
 *
 * DESCRIPTION
 * -----------
 * This driver interfaces with the HYT271 digital humidity and temperature
 * sensor manufactured by IST AG. The sensor provides calibrated, linearized
 * humidity and temperature readings via I2C interface.
 *
 *
 * SENSOR SPECIFICATIONS
 * ---------------------
 *
 *     | Parameter          | Value                    |
 *     |--------------------|--------------------------|
 *     | Humidity Range     | 0 to 100% RH             |
 *     | Humidity Accuracy  | +/- 1.8% RH (typical)    |
 *     | Temperature Range  | -40C to +125C            |
 *     | Temperature Accuracy| +/- 0.2C (typical)      |
 *     | Resolution         | 14-bit (both channels)   |
 *     | Conversion Time    | ~60ms (typical)          |
 *     | Interface          | I2C                      |
 *     | Default Address    | 0x28 (configurable)      |
 *
 *
 * I2C PROTOCOL
 * ------------
 *
 *     Measurement Request (MR):
 *         Master sends: [I2C_ADDR + W] [0x00]
 *         Sensor begins conversion (~60ms)
 *
 *     Data Fetch (DF):
 *         Master sends: [I2C_ADDR + R]
 *         Sensor returns: [HUM_H] [HUM_L] [TEMP_H] [TEMP_L]
 *
 *     Status bits in HUM_H (data[0]):
 *         Bit 7: Command mode indicator (0 = normal)
 *         Bit 6: Stale data flag (1 = data not updated since last read)
 *         Bits 5:0: Upper 6 bits of humidity
 *
 *
 * DATA FORMAT
 * -----------
 *
 *     Humidity (14-bit):
 *         raw = ((data[0] << 8) | data[1]) & 0x3FFF
 *         humidity_pct = raw * (100.0 / 16383.0)
 *
 *     Temperature (14-bit):
 *         raw = (data[2] << 6) | (data[3] >> 2)
 *         temp_kelvin = raw * (165.0 / 16383.0) + 233.15
 *
 *     Conversion constants:
 *         Humidity scale:    100.0 / 16383.0 = 0.00610388
 *         Temperature scale: 165.0 / 16383.0 = 0.01007141
 *         Temperature offset: 233.15 K (-40C)
 *
 *
 * TIMING
 * ------
 *
 *     | Parameter            | Value     |
 *     |----------------------|-----------|
 *     | Callback interval    | 100 ms    |
 *     | Update rate          | 10 Hz     |
 *     | Conversion time      | ~60 ms    |
 *     | Data latency         | 100-160ms |
 *
 *     Measurement sequence:
 *         T=0ms:   _timer() -> _collect() reads previous data
 *                            -> _measure() starts new conversion
 *         T=60ms:  Conversion complete (data ready)
 *         T=100ms: _timer() -> _collect() reads T=0ms data
 *                            -> _measure() starts new conversion
 *
 *
 * I2C ADDRESSING
 * --------------
 *
 *     The HYT271 default address is 0x28, but can be configured.
 *     Multiple sensors can share one I2C bus with different addresses.
 *
 *     | Sensor | Address |
 *     |--------|---------|
 *     | 1      | 0x10    |
 *     | 2      | 0x11    |
 *     | 3      | 0x12    |
 *     | 4      | 0x13    |
 *
 *
 * USAGE EXAMPLE
 * -------------
 *
 *     AC_CASS_HYT271 sensor;
 *
 *     // Initialize on I2C bus 0 at address 0x10
 *     if (sensor.init(0, 0x10)) {
 *         // Sensor initialized successfully
 *     }
 *
 *     // In main loop (data updated automatically by timer callback)
 *     if (sensor.healthy()) {
 *         float humidity = sensor.relative_humidity();  // 0-100%
 *         float temp_k = sensor.temperature();          // Kelvin
 *         float temp_c = temp_k - 273.15f;              // Celsius
 *     }
 *
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Vehicle/AP_Vehicle.h>

class AC_CASS_HYT271 {
public:
    AC_CASS_HYT271(void);
    ~AC_CASS_HYT271(void){}

    bool init(uint8_t busId, uint8_t i2cAddr);  // Initialize sensor on I2C bus
    float relative_humidity(void) { return _humidity; }  // Returns humidity in % (0-100)
    float temperature(void) { return _temperature; }     // Returns temperature in Kelvin
    bool healthy(void) { return _healthy; }              // Returns true if last reading was valid
    void set_i2c_addr(uint8_t addr);                     // Change I2C address at runtime

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;  // I2C device handle
    HAL_Semaphore _sem;                       // Semaphore for thread-safe data access
    float _temperature;                       // Cached temperature in Kelvin
    float _humidity;                          // Cached relative humidity in %
    bool _healthy;                            // True if last measurement was valid
    bool _measure(void);                      // Send measurement request to sensor
    bool _collect(float &hum, float &temp);   // Read and convert sensor data
    void _timer(void);                        // Periodic callback at 10Hz
};