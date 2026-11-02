#pragma once

#include "SIM_config.h"

#if AP_SIM_CASS_ENABLED

#include "SIM_I2CDevice.h"

/*
  Simulators for the CopterSonde atmospheric sensors (CASS), read by the AC_CASS_IMET and
  AC_CASS_HYT271 drivers on I2C bus 0.

  The ambient temperature and humidity come from the physics backend when it models the
  atmosphere (the CopterSonde Gazebo world sends them in the JSON "atmosphere" section).
  Otherwise an altitude profile is used.

  The sensors follow the ambient air with first-order lags measured on CopterSonde
  profiles (iMet bead 1.5 s, HYT271 5 s, HYT271 temperature 10 s). The HYT271 also runs warmer
  than the air, by 1.2 K near the ground and more as the air thins. Each unit has a small
  fixed bias.
*/

namespace SITL {

// ambient air at the vehicle: temperature (K), relative humidity (%) and pressure (Pa)
void cass_ambient(const class Aircraft &aircraft, float &temperature, float &humidity, float &pressure);

/*
  iMet bead thermistor on an ADS1115 ADC. The bead runs from the source (AIN1) to AIN0, and a
  64.9 kOhm resistor from AIN0 to ground: the driver reads AIN0-AIN1 = -Vs*Rt/(Rt+Rf) and
  AIN1 = Vs. Conversions finish at once, because the driver busy-waits on the OS bit and
  simulated time does not advance inside a device callback.
*/
class CASS_iMet : public I2CDevice
{
public:
    CASS_iMet(uint8_t _instance) : instance(_instance) {}

    void update(const class Aircraft &aircraft) override;

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

private:
    const uint8_t instance;    // selects the USR_SEN_A/B/C coefficients and the bead bias
    uint8_t pointer;
    uint16_t config = 0x8583;  // power-on default
    int16_t conversion;
    float bead_temperature;    // K
    float coeff[4];            // Steinhart-Hart coefficients
    uint32_t last_coeff_ms;
    uint64_t last_update_us;

    void load_coefficients();
    float resistance(float temperature) const;
    int16_t convert(uint16_t cfg) const;
};

/*
  HYT271 capacitive humidity and temperature sensor. A write starts a measurement; a 4 byte
  read returns humidity and temperature, with the stale bit set until 60 ms after the
  request.

  The raw humidity inverts the firmware's poly22 correction (USR_RH_A/B/C) at the ambient
  temperature, so that the corrected humidity matches the air. The calibration was fitted on
  the real sensors, so it describes how they respond, including their self-heating. Without
  usable coefficients the raw humidity is the humidity at the sensor's own temperature.
*/
class CASS_HYT271 : public I2CDevice
{
public:
    CASS_HYT271(uint8_t _instance) : instance(_instance) {}

    void update(const class Aircraft &aircraft) override;

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

private:
    const uint8_t instance;    // selects the USR_RH_A/B/C coefficients and the bias
    float humidity;            // %, lagged ambient humidity
    float air_temperature;     // K, ambient temperature lagged like the iMet beads
    float sensor_temperature;  // K
    float density_ratio = 1;   // surface density / ambient density
    float coeff[6];            // poly22 coefficients [p00, p10, p01, p20, p11, p02]
    uint32_t last_coeff_ms;
    uint64_t last_update_us;
    uint32_t request_ms;
    bool request_pending;
    uint8_t reading[4];

    void load_coefficients();
    float raw_humidity() const;
    void take_reading();
};

} // namespace SITL

#endif  // AP_SIM_CASS_ENABLED
