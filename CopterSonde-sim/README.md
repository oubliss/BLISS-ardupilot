# CopterSonde Gazebo simulator

The CopterSonde flies over the Norman, OK radar-site world through air with a realistic, repeating daily cycle of the planetary boundary layer (PBL): temperature, relative humidity, wind and turbulence from the ground to 3500 m (11 500 ft). The firmware reads that air through its own sensor drivers:

- **iMet bead thermistors (AC_CASS_IMET):** SITL simulates the ADS1115 and the bead circuit.
- **HYT271 hygrometers (AC_CASS_HYT271):** SITL simulates the sensor itself.
- **Wind:** the wind-vane estimator measures the real tilt that the wind forces on the vehicle.

The IMET, RHUM and WIND logs, CASS_SENSOR_RAW and LAAIRS all work as on the real CopterSonde.

| Part | Where |
|---|---|
| World, vehicle model, atmosphere table, SITL parameters, tools | this folder (my-ardupilot `ArduPilot-4.6-CS-gazebo`) |
| SITL sensor simulators, JSON `atmosphere` fields | `libraries/SITL/SIM_CASS.*`, `SIM_JSON.*` |
| `CopterSondePlugin` (air, wind force, battery), `ArduPilotPlugin` `<json_extra_topic>` | ardupilot_gazebo `main-CopterSonde` |

## Run

Build once:

```bash
cd ~/Github/ardupilot_gazebo && git checkout main-CopterSonde && mkdir -p build && cd build && GZ_VERSION=jetty cmake .. && make -j8
```

```bash
cd ~/Github/my-ardupilot-CS && ./waf configure --board sitl && ./waf copter
```

Then start the two sides:

```bash
# Terminal A: Gazebo (generates the Norman world the first time)
~/Github/my-ardupilot-CS/CopterSonde-sim/tools/coptersonde_gazebo.sh

# Terminal B: SITL
~/Github/my-ardupilot-CS/CopterSonde-sim/tools/coptersonde_sitl.sh
```

Connect LAAIRS with **WSL SITL** (TCP 127.0.0.1:5760); SITL waits for that connection before it starts. TCP 5762 is free for MAVProxy (`mavproxy.py --master tcp:127.0.0.1:5762 --console`) or the flight-test script.

Fly a profile as in the field: on the ground flip RC7 (AutoVP, `RC7_OPTION 47`), arm in Loiter and switch to Auto. The script does the same:

```bash
python3 ~/Github/my-ardupilot-CS/CopterSonde-sim/tools/vp_flight_test.py --alt 2438
```

As on the real CS3D, the VP battery monitor (`USR_VPBATT_WH 190`, 25 % reserve) ends the climb with an RTL at about 2.7 km. To profile up to 10 000 ft, let it only warn: `--alt 3048 --param USR_VPBATT_EN=0`.

Use the scripts rather than a bare `sim_vehicle.py`. The `sim_vehicle.py` on PATH may belong to another ArduPilot tree (RFSonde), and it runs the firmware of the tree it lives in.

### Options

`coptersonde_gazebo.sh` options:

| Option | Effect |
|---|---|
| `--start-hour H` | local time of day when the simulation starts (default 9, the CSWX flight window) |
| `--cycle-hours H` | simulation hours for one day of the atmosphere: 24 is real time, 2 runs the day 12 times faster |
| `--truth-log FILE` | writes the air, wind and battery at the vehicle at 10 Hz (CSV), for `profile_eval.py` |
| `--headless` | Gazebo server only |
| `--instance N` | talk to SITL instance N; start SITL with `coptersonde_sitl.sh -I N` |

`-I 0`, the default, makes `sim_vehicle.py` stop every other running arducopter. Use another instance to keep one running.

## Atmosphere

`atmosphere/norman_pbl.csv` holds one day: every 30 min, every 25 m from 0 to 3500 m. It covers temperature, relative humidity, pressure, wind (east, north) and turbulence standard deviations. The plugin interpolates it in time (wrapping at midnight) and height. `tools/pbl_model.py` generates it.

The model is a 1D column model of potential temperature, humidity and wind:
- Surface fluxes follow the sun; at night a bulk flux carries the air towards the cooling ground.
- Mixing is a non-local K-profile in the convective boundary layer and Richardson-number mixing at night.
- The geostrophic wind veers with height, and the Coriolis force acts on the wind.
- Radiative cooling and relaxation to a reference sounding close the daily budget.

It runs the same day until it repeats, and writes the last one.

![The day](atmosphere/norman_pbl_day.png)

The forcing was fitted to the 32 CSWX ascents flown on 2026-08-27, 09-03, 09-17 and 09-25 (09:00–13:00 CDT, up to 2438 m), plus the season's dawn minimum and afternoon maximum. The resulting day:

| Time | What happens |
|---|---|
| Night | a surface inversion forms (19 °C at the ground, 26 °C at 300 m before dawn) with dew near the ground, and a south-southwesterly low-level jet of about 9 m/s at 300 m (an inertial oscillation) |
| 09–13 | a mixed layer grows into the night's inversion and residual layer; humidity increases with height to a maximum near its top, as in the CSWX profiles |
| Afternoon | a dry-adiabatic mixed layer 2.4 km deep at 35–36 °C at the ground, with convective turbulence (σu ≈ 1.5 m/s) |
| Above | the free troposphere: 3.4 K/km potential-temperature lapse rate and drier air |

Against the CSWX composite at 09–13 CDT, temperature is mostly within 1–2 K, humidity within 5–10 % and wind within 1–2 m/s. Individual days differed more than this from one another.

![Model against CSWX](atmosphere/norman_pbl_vs_cswx.png)

`tools/plot_atmosphere.py --cswx "<folder of CopterSonde logs>"` redraws both figures. To change the climate, edit `Forcing` in `tools/pbl_model.py` and rerun it (about 30 s):

```bash
python3 CopterSonde-sim/tools/pbl_model.py
```

To use another table, set `CS_ATMOSPHERE=/path/table.csv` before starting Gazebo, or change `<profile>` in `models/coptersonde/model.sdf`. Any table with the same columns works, including one built from real soundings.

Gusts are two Gauss-Markov processes per axis (large and small eddies) with the table's standard deviations. Their time scales are the integral length scale over the mean wind.

## Vehicle

`models/coptersonde` is the ArduPilot Iris airframe (`iris_with_ardupilot`, 1.75 kg, `FRAME_TYPE 1`) with a sensor scoop drawn at the nose. Its `CopterSondePlugin` does four things.

- **Propellers.** It replaces the Iris's LiftDrag blades. Thrust is k·ω² along the spin axis, reduced when air flows down through the disk (a climb) and in thin air; a drag torque of 0.016 m × thrust acts against the spin. Hover is at 450 rad/s, and the thrust-to-weight ratio is 3.5. The LiftDrag blades see only the vehicle's own motion, not the rotor's induced flow. In a 5 m/s descent they windmill, the air drives the rotors and the yaw control reverses: the Iris spins up in yaw and tumbles about 30 s into a long descent.

- **Air at the vehicle.** It looks up the table at the vehicle's height and the time of day, adds gusts, and sends the result to SITL in the JSON `atmosphere` section (temperature, humidity, pressure), with the wind in `velocity_wind`.
- **Wind force.** Air moving past the vehicle pushes it with the force that tilts the real CopterSonde. The wind-vane calibration gives the airspeed v at a tilt ratio r = tan(tilt) as v = a·r + b·√r (`USR_WV_WSA` 39.3, `USR_WV_WSB` 2.1 on the CS3D, log 00000080). So the horizontal force is m·g·r(v), scaled by the air density over the calibration density (1.13 kg/m³). Like the real vehicle, the estimator therefore reads a little low in the thin air aloft. Vertical motion has a plain quadratic drag.
- **Battery.** A 6S 9 Ah LiPo, sent in the JSON `battery` section. The current follows the rotor power from momentum theory: the induced power T·(Vc/2 + √((Vc/2)² + vi²)) grows with the thrust, the climb rate through the air and thinner air, and a 30 % profile power follows the rotor speed up to its hover value. The hover current (20 A) and the induced velocity (8 m/s) are fitted to the CSWX flights: the simulator draws 21.5 A in hover, 28.1 A climbing and 15.7 A descending at 5 m/s (real: 20–21, 24–28 and 15–18 A). The VP battery monitor and the current failsafes see the same numbers as on the real vehicle.

`gz topic -e -t /model/coptersonde/coptersonde` shows what goes to SITL every 10 ms, including the local hour and the wind speed and direction at the vehicle.

## Sensors in SITL

SITL simulates the sensors on I2C bus 0, where the drivers look for them. As on the CS3D on the CSWX flights, there are three of each (`USR_SENSOR4` and `USR_SENSOR8` are 0):

| Sensor | Addresses | Behaviour |
|---|---|---|
| iMet bead + ADS1115 | 0x48–0x4A | 1.5 s lag; bead resistance from the inverse Steinhart–Hart of the loaded `USR_SEN_A/B/C` coefficients, through the 64.9 kΩ divider and 16-bit ADC with noise; fixed biases of +0.08, −0.05 and +0.03 K |
| HYT271 | 0x10–0x12 | 5 s humidity lag; 14-bit humidity and temperature with the stale bit; the sensor runs 1.2 K warmer than the air near the ground, more as the air thins (as in the logs); the raw humidity inverts the loaded `USR_RH_A/B/C` correction, so the corrected humidity matches the air; biases of +0.6, −0.4 and +0.2 % |

The lags and the self-heating were measured on the CSWX flights, from the ascent–descent difference and HYT271 minus iMet temperature. Without Gazebo (plain `sim_vehicle.py -f quad`), the simulators use the altitude profile the SITL code used to fake.

`config/coptersonde.parm` loads the CS3D's sensor IDs and calibrations, wind vane, VP battery, AutoVP, battery, fence and navigation settings. It does not load the attitude tuning, notch filter or motor outputs, which belong to a different airframe.

It does tune the simulated airframe's roll: the Gazebo Iris's roll inertia is half its pitch inertia. With SITL's default rate gains, roll oscillates at about 2 Hz (rate spread 10–17 °/s in hover and descent); the stock Iris in `iris_runway.sdf` does the same. `ATC_RAT_RLL_P/I/D` 0.07/0.07/0.0018 (half the defaults) bring it to 0.5 °/s.

## Verification

A 10 000 ft AutoVP profile at 09:00 (`--alt 3048 --param USR_VPBATT_EN=0`, 5 m/s up and down) compared with `profile_eval.py` in 250 m bins, as sensor minus truth, mean / RMS:

| | Ascent | Descent |
|---|---|---|
| iMet temperature | +0.07 / 0.08 K | −0.02 / 0.04 K |
| HYT271 humidity (corrected) | +0.3 / 0.8 % | +0.1 / 0.7 % |
| Wind-vane speed | −0.7 / 0.8 m/s | −0.4 / 0.5 m/s |
| Wind-vane direction | −0.4 / 0.9° | +0.4 / 0.9° |

The sensors lag as the real ones do: warm and dry going up through air that cools and moistens with height. The wind-vane speed reads low by up to 23 % at 3 km, where the air is about 25 % thinner, so the same wind tilts the vehicle less. The real estimator has no density correction either. Attitude stays steady through the whole profile, at 0.3–0.4 °/s rate spread on roll and pitch. With `USR_VPBATT_EN 1` the VP battery monitor sends the vehicle home at about 2.7 km.

## Compare a flight with the truth

```bash
coptersonde_gazebo.sh --truth-log /tmp/truth.csv     # Terminal A
coptersonde_sitl.sh                                  # Terminal B, then fly
python3 CopterSonde-sim/tools/profile_eval.py logs/00000001.BIN /tmp/truth.csv
```

It splits the flight into ascent and descent and compares the logged iMet temperature, corrected humidity and wind-vane speed and direction with the truth in 50 m bins.

## Tests

```bash
python3 -m pytest CopterSonde-sim/tests
```

These check the table (grid, physical ranges, the midnight wrap, the diurnal cycle, the nocturnal inversion and jet, the afternoon mixed layer) and that the model's cycle repeats.

## World

The Norman, OK radar-site world is the RFSonde one: 2 km around KCRI at Max Westheimer Airport, with OpenStreetMap buildings, roads and trees and the radars at their true positions.

- **Origin:** the takeoff point at 35.2380802 N, 97.4631246 W, 358.6 m MSL (location `CopterSonde_Norman` in `Tools/autotest/locations.txt`).
- **Heading:** the vehicle starts facing south, into the prevailing wind.
- **Generation:** `tools/make_norman_world.py` writes the world and its `cs_norman_*` models; they are git-ignored. The `cs_` prefix keeps the RFSonde-sim models on the resource path from replacing them.

The OpenStreetMap data in `osm/norman_osm.json` is © OpenStreetMap contributors, ODbL.
