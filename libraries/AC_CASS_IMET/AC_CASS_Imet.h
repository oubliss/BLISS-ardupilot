/*
 * I2C driver for Measurement Specialties IMET ADS1115 digital temperature sensor
 */

//#pragma once

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
    #define ADS1115_READ_THERMISTOR         4
    #define ADS1115_READ_SOURCE             5
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
    HAL_Semaphore _sem; // iterruption object for data logging management
    bool flag;  // toggles between voltage and current measurements
    float coeff[4]; // sensor coefficients
    float adc_thermistor, adc_source;   // voltage source and thermistor form ADC
    float _temperature; // degrees K
    float _resist; // pseudo-resistance read by the ADC
    bool _healthy; // we have a valid temperature reading to report
    uint16_t config; // Configuration to be sent to the ADC registers
    uint8_t runs; // Number of samples taken before getting an updated measurment of the source
    bool _start_conversion(uint8_t channel); // Configure and start conversion/measurement
    bool _read_adc(float &value); // Request and retreive data from the sensor
    void _timer(void); // update the temperature, called at 20Hz
    void _calculate(float source, float thermistor); // calculate temperature using adc readings and coefficients
};