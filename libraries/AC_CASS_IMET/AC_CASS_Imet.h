/*
 * AC_CASS_Imet - I2C Temperature Sensor Driver using ADS1115 ADC
 * ==============================================================
 *
 * DESCRIPTION
 * -----------
 * This driver reads temperature from an IMET bead thermistor using a Texas
 * Instruments ADS1115 16-bit Analog-to-Digital Converter. The thermistor is
 * part of a voltage divider circuit, and temperature is calculated using the
 * Steinhart-Hart equation.
 *
 * The driver uses differential measurement mode for the thermistor voltage
 * to achieve common-mode noise rejection, providing clean readings without
 * software filtering delay. The source voltage is measured single-ended with
 * heavy filtering since it should remain constant.
 *
 *
 * CIRCUIT SCHEMATIC
 * -----------------
 *
 *     VCC (3.3V) ──────────────────────┬──── AIN1 (Source voltage, single-ended)
 *                                      │
 *                                 ┌────┴────┐
 *                                 │Thermistor│  R_therm (NTC bead, on TOP)
 *                                 │  (IMET)  │
 *                                 └────┬────┘
 *                                      │
 *                                      ├──── AIN0 (Midpoint voltage)
 *                                      │
 *                                 ┌────┴────┐
 *                                 │  64.9k  │  R_fixed (precision resistor)
 *                                 │         │
 *                                 └────┬────┘
 *                                      │
 *     GND ─────────────────────────────┘
 *
 *     Differential measurement: AIN0 - AIN1 = -V_thermistor
 *
 *
 * ADC CHANNEL CONFIGURATION
 * -------------------------
 *
 *     | Channel | MUX Setting      | Measurement Type | Description              |
 *     |---------|------------------|------------------|--------------------------|
 *     | 0       | DIFF_0_1         | Differential     | AIN0 - AIN1 (thermistor) |
 *     | 5       | SINGLE_1         | Single-ended     | AIN1 vs GND (source)     |
 *
 *     Note: Differential mode provides common-mode noise rejection, eliminating
 *     voltage spikes caused by power supply fluctuations or EMI.
 *
 *
 * MEASUREMENT STRATEGY
 * --------------------
 *
 *     | Signal      | Mode         | Filtering        | Rationale                    |
 *     |-------------|--------------|------------------|------------------------------|
 *     | Thermistor  | Differential | None             | Clean signal, fast response  |
 *     | Source (Vcc)| Single-ended | Median + EMA     | Stable, can tolerate lag     |
 *
 *     Timing (sequential measurements at 32 SPS):
 *     - Timer callback: 10 Hz (100 ms period)
 *     - Thermistor sampled: 10 Hz (every callback)
 *     - Source voltage sampled: 10 Hz (every callback)
 *     - ADC conversion time: ~31.25 ms at 32 SPS
 *     - Total conversion time per callback: ~62.5 ms (leaves ~37.5 ms margin)
 *
 *     Measurement sequence per callback:
 *     |  Step  |  Action                                    |  Time      |
 *     |--------|--------------------------------------------|------------|
 *     |  1     |  Read thermistor (from previous callback)  |  ~1 ms     |
 *     |  2     |  Start source conversion                   |  ~1 ms     |
 *     |  3     |  Wait and read source result               |  ~31.25 ms |
 *     |  4     |  Start thermistor conversion (for next)    |  ~1 ms     |
 *     |        |  Total per callback                        |  ~34 ms    |
 *
 *
 * RESISTANCE CALCULATION
 * ----------------------
 *
 *     Given the voltage divider with R_therm on top and R_fixed on bottom:
 *
 *         V_thermistor = V_source * R_therm / (R_therm + R_fixed)
 *         V_fixed      = V_source * R_fixed / (R_therm + R_fixed)
 *
 *     The differential measurement gives:
 *
 *         diff = AIN0 - AIN1 = V_midpoint - V_source = -V_thermistor
 *
 *     Therefore:
 *
 *         V_thermistor = -diff
 *         V_fixed      = V_source - V_thermistor = V_source + diff
 *
 *     Solving for R_therm:
 *
 *         R_therm = R_fixed * V_thermistor / V_fixed
 *                 = R_fixed * (-diff) / (V_source + diff)
 *
 *     In code (AC_CASS_Imet.cpp):
 *
 *         _resist = 64900.0f * (-thermistor_diff) / (source + thermistor_diff);
 *
 *
 * MATH VERIFICATION EXAMPLE
 * -------------------------
 *
 *     Given:
 *         V_source = 3.3V (supply voltage)
 *         R_therm  = 10,000 ohms (thermistor at ~25C)
 *         R_fixed  = 64,900 ohms
 *
 *     Step 1: Calculate actual voltages
 *         Total R   = 10,000 + 64,900 = 74,900 ohms
 *         Current   = 3.3V / 74,900 = 44.06 uA
 *         V_therm   = 44.06 uA * 10,000 = 0.4406V
 *         V_fixed   = 44.06 uA * 64,900 = 2.8594V
 *         V_midpoint= V_fixed = 2.8594V (voltage at AIN0)
 *
 *     Step 2: Simulate ADC readings (at PGA=6.144V, LSB=0.1875mV, 32 SPS)
 *         source (AIN1)     = 3.3V    -> 17,600 counts
 *         midpoint (AIN0)   = 2.8594V -> 15,250 counts
 *         diff (AIN0-AIN1)  = 2.8594 - 3.3 = -0.4406V -> -2,350 counts
 *
 *     Step 3: Apply resistance formula
 *         R_therm = 64,900 * (-(-2350)) / (17600 + (-2350))
 *                 = 64,900 * 2350 / 15250
 *                 = 64,900 * 0.1541
 *                 = 10,001 ohms  (matches expected value)
 *
 *
 * TEMPERATURE CALCULATION (STEINHART-HART)
 * ----------------------------------------
 *
 *     The Steinhart-Hart equation converts resistance to temperature:
 *
 *         1/T = c0 + c1*ln(R) + c2*ln(R)^2 + c3*ln(R)^3
 *
 *     Where:
 *         T  = Temperature in Kelvin
 *         R  = Thermistor resistance in ohms
 *         c0, c1, c2, c3 = Sensor-specific coefficients
 *
 *     Coefficients are set via set_sensor_coeff() for each IMET sensor.
 *
 *
 * FILTERING IMPLEMENTATION
 * ------------------------
 *
 *     Source voltage uses a cascaded filter for robust noise rejection:
 *
 *     1. Median Filter (window size = 5):
 *        - Eliminates voltage spikes (outliers sorted to edges)
 *        - No phase distortion
 *        - Adds 2-sample latency
 *
 *     2. Exponential Moving Average (alpha = 0.10):
 *        - Smooths remaining noise
 *        - Formula: output = 0.90 * prev_output + 0.10 * input
 *        - Time constant: ~9 samples for 63% step response
 *
 *     Thermistor uses NO filtering:
 *        - Differential mode provides inherently clean signal
 *        - Instant response to temperature changes
 *
 *
 * I2C ADDRESSING
 * --------------
 *
 *     | ADDR Pin | I2C Address (7-bit) | I2C Address (8-bit shifted) |
 *     |----------|---------------------|----------------------------|
 *     | GND      | 0x48                | 0x90                       |
 *     | VDD      | 0x49                | 0x92                       |
 *     | SDA      | 0x4A                | 0x94                       |
 *     | SCL      | 0x4B                | 0x96                       |
 *
 *     Up to 4 ADS1115 devices can share one I2C bus.
 *
 *
 * USAGE EXAMPLE
 * -------------
 *
 *     AC_CASS_Imet sensor;
 *
 *     // Initialize on I2C bus 0 at address 0x48
 *     if (sensor.init(0, 0x48)) {
 *         // Set Steinhart-Hart coefficients for this specific thermistor
 *         float coeffs[4] = {c0, c1, c2, c3};
 *         sensor.set_sensor_coeff(coeffs);
 *     }
 *
 *     // In main loop (data updated automatically by timer callback)
 *     if (sensor.healthy()) {
 *         float temp_kelvin = sensor.temperature();
 *         float temp_celsius = temp_kelvin - 273.15f;
 *         float resistance = sensor.resistance();
 *     }
 *
 */

#pragma once

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define ADS1115_ADDRESS                 (0x90)    // 1001 000 shifted left 1 bit = 0x90 (ADDR = GND)
    #define ADS1115_READBIT                 (0x01)
/*=========================================================================*/

/*=========================================================================
    POINTER REGISTER
    -----------------------------------------------------------------------*/
    #define ADS1115_REG_POINTER_MASK        (0x03)
    #define ADS1115_REG_POINTER_CONVERT     (0x00)
    #define ADS1115_REG_POINTER_CONFIG      (0x01)
    #define ADS1115_REG_POINTER_LOWTHRESH   (0x02)
    #define ADS1115_REG_POINTER_HITHRESH    (0x03)
/*=========================================================================*/

/*=========================================================================
    CONFIG REGISTER
    -----------------------------------------------------------------------*/
    #define ADS1115_REG_CONFIG_OS_MASK      (0x80)
    #define ADS1115_REG_CONFIG_OS_SINGLE    (0x8000)  // Write: Set to start a single-conversion
    #define ADS1115_REG_CONFIG_OS_BUSY      (0x00)  // Read: Bit = 0 when conversion is in progress
    #define ADS1115_REG_CONFIG_OS_NOTBUSY   (0x80)  // Read: Bit = 1 when device is not performing a conversion

    #define ADS1115_REG_CONFIG_MUX_MASK     (0x7000)
    #define ADS1115_REG_CONFIG_MUX_DIFF_0_1 (0x0000)  // Differential P = AIN0, N = AIN1 (default)
    #define ADS1115_REG_CONFIG_MUX_DIFF_0_3 (0x1000)  // Differential P = AIN0, N = AIN3
    #define ADS1115_REG_CONFIG_MUX_DIFF_1_3 (0x2000)  // Differential P = AIN1, N = AIN3
    #define ADS1115_REG_CONFIG_MUX_DIFF_2_3 (0x3000)  // Differential P = AIN2, N = AIN3
    #define ADS1115_REG_CONFIG_MUX_SINGLE_0 (0x4000)  // Single-ended AIN0
    #define ADS1115_REG_CONFIG_MUX_SINGLE_1 (0x5000)  // Single-ended AIN1
    #define ADS1115_REG_CONFIG_MUX_SINGLE_2 (0x6000)  // Single-ended AIN2
    #define ADS1115_REG_CONFIG_MUX_SINGLE_3 (0x7000)  // Single-ended AIN3

    #define ADS1115_REG_CONFIG_PGA_MASK     (0x0E00)
    #define ADS1115_REG_CONFIG_PGA_6_144V   (0x0000)  // +/-6.144V range
    #define ADS1115_REG_CONFIG_PGA_4_096V   (0x0200)  // +/-4.096V range
    #define ADS1115_REG_CONFIG_PGA_2_048V   (0x0400)  // +/-2.048V range (default)
    #define ADS1115_REG_CONFIG_PGA_1_024V   (0x0600)  // +/-1.024V range
    #define ADS1115_REG_CONFIG_PGA_0_512V   (0x0800)  // +/-0.512V range
    #define ADS1115_REG_CONFIG_PGA_0_256V   (0x0A00)  // +/-0.256V range

    // Do not exceed VDD + 0.3V, or the gain set max!
    //                        ADS1015         ADS1115
    //                        -------------   -----------------
    // 2/3x gain  +/- 6.144V  1 bit = 3mV     0.1875mV (default)
    // 1x gain    +/- 4.096V  1 bit = 2mV     0.125mV
    // 2x gain    +/- 2.048V  1 bit = 1mV     0.0625mV
    // 4x gain    +/- 1.024V  1 bit = 0.5mV   0.03125mV
    // 8x gain    +/- 0.512V  1 bit = 0.25mV  0.015625mV
    // 16x gain   +/- 0.256V  1 bit = 0.125mV 0.0078125mV

    #define ADS1115_REG_CONFIG_MODE_MASK    (0x0100)
    #define ADS1115_REG_CONFIG_MODE_CONTIN  (0x0000)  // Continuous conversion mode
    #define ADS1115_REG_CONFIG_MODE_SINGLE  (0x0100)  // Power-down single-shot mode (default)

    #define ADS1115_REG_CONFIG_DR_MASK      (0x00E0)
    #define ADS1115_REG_CONFIG_DR_8SPS      (0x0000)  // 8 samples per second
    #define ADS1115_REG_CONFIG_DR_16SPS     (0x0020)  // 16 samples per second
    #define ADS1115_REG_CONFIG_DR_32SPS     (0x0040)  // 32 samples per second
    #define ADS1115_REG_CONFIG_DR_64SPS     (0x0060)  // 64 samples per second
    #define ADS1115_REG_CONFIG_DR_128SPS    (0x0080)  // 128 samples per second (default)
    #define ADS1115_REG_CONFIG_DR_250SPS    (0x00A0)  // 250 samples per second
    #define ADS1115_REG_CONFIG_DR_475SPS    (0x00C0)  // 475 samples per second
    #define ADS1115_REG_CONFIG_DR_860SPS    (0x00E0)  // 860 samples per second

    #define ADS1115_REG_CONFIG_CMODE_MASK   (0x0010)
    #define ADS1115_REG_CONFIG_CMODE_TRAD   (0x0000)  // Traditional comparator with hysteresis (default)
    #define ADS1115_REG_CONFIG_CMODE_WINDOW (0x0010)  // Window comparator

    #define ADS1115_REG_CONFIG_CPOL_MASK    (0x0008)
    #define ADS1115_REG_CONFIG_CPOL_ACTVLOW (0x0000)  // ALERT/RDY pin is low when active (default)
    #define ADS1115_REG_CONFIG_CPOL_ACTVHI  (0x0008)  // ALERT/RDY pin is high when active

    #define ADS1115_REG_CONFIG_CLAT_MASK    (0x0004)  // Determines if ALERT/RDY pin latches once asserted
    #define ADS1115_REG_CONFIG_CLAT_NONLAT  (0x0000)  // Non-latching comparator (default)
    #define ADS1115_REG_CONFIG_CLAT_LATCH   (0x0004)  // Latching comparator

    #define ADS1115_REG_CONFIG_CQUE_MASK    (0x0003)
    #define ADS1115_REG_CONFIG_CQUE_1CONV   (0x0000)  // Assert ALERT/RDY after one conversions
    #define ADS1115_REG_CONFIG_CQUE_2CONV   (0x0001)  // Assert ALERT/RDY after two conversions
    #define ADS1115_REG_CONFIG_CQUE_4CONV   (0x0002)  // Assert ALERT/RDY after four conversions
    #define ADS1115_REG_CONFIG_CQUE_NONE    (0x0003)  // Disable the comparator and put ALERT/RDY in high state (default)

    #define ADS1115_CHANNELS_COUNT          8
    #define ADS1115_READ_THERMISTOR         0   // Differential AIN0-AIN1 = -V_thermistor
    #define ADS1115_READ_SOURCE             5   // Single-ended AIN1 = V_source

    // Median filter configuration for noise rejection
    #define MEDIAN_WINDOW_SIZE              5
/*=========================================================================*/

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Vehicle/AP_Vehicle.h>

//#define IMET_DEFAULT_ADDR 0x48

class AC_CASS_Imet {
public:
    AC_CASS_Imet(void);
    ~AC_CASS_Imet(void){}
    bool init(uint8_t busId, uint8_t i2cAddr); // initialize sensor object
    float temperature(void) { return _temperature; } // temperature in kelvin
    float resistance(void) { return _resist; }   // voltage read by the ADCS
    bool healthy(void) { return _healthy; } // do we have a valid temperature reading?
    void set_i2c_addr(uint8_t addr);
    void set_sensor_coeff(float *k);

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev; // I2C object for communication management
    HAL_Semaphore _sem; // semaphore for data logging management
    float coeff[4]; // sensor coefficients
    float adc_thermistor, adc_source;   // thermistor (differential) and source voltage from ADC
    float source_buffer[MEDIAN_WINDOW_SIZE]; // circular buffer for median filter on source
    uint8_t buffer_idx;                 // index for circular buffer
    float _temperature; // degrees K
    float _resist; // pseudo-resistance read by the ADC
    bool _healthy; // we have a valid temperature reading to report
    uint16_t config; // Configuration to be sent to the ADC registers
    bool _start_conversion(uint8_t channel); // Configure and start conversion/measurement
    bool _read_adc(float &value); // Request and retreive data from the sensor
    void _timer(void); // update the temperature, called at 10Hz
    void _calculate(float source, float thermistor); // calculate temperature using adc readings and coefficients
    float _median_filter(float *buffer, uint8_t size); // returns median value from buffer for spike rejection
};