"""Digital twin of the CopterSonde's RTK base station: a u-blox ZED-F9P that LAAIRS configures, surveys in
and reads RTCM3 corrections from, to inject them into the drone (GPS_RTCM_DATA).

LAAIRS reaches it as the serial port socket://127.0.0.1:3101 (RTK Injection window, "Digital twin
(simulator)"). SITL turns RTK fixed only while those corrections arrive (SIM_GPS_RTK 1).
"""
