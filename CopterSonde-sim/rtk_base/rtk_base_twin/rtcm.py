"""RTCM 3.3 messages of a u-blox F9P base: 1005 (base position), MSM4 observations and 1230.

The observations are plausible, not physical: the rover simulator only needs well-formed corrections
(LAAIRS parses them with pyrtcm, ArduPilot frames them with its RTCM3 parser and SITL counts them).
"""

# AP_FLAKE8_CLEAN

import math

PREAMBLE = 0xD3
SPEED_OF_LIGHT_KM_PER_MS = 299792.458e-3

# MSM4 message per constellation, its signals (RTCM signal IDs as an F9P sends them) and satellites in view
GNSS = {
    'GPS': {'msg': 1074, 'signals': (2, 15), 'sats': (2, 5, 12, 13, 15, 18, 20, 24, 25, 29)},
    'GLONASS': {'msg': 1084, 'signals': (2, 8), 'sats': (1, 2, 8, 9, 10, 17, 18, 24)},
    'Galileo': {'msg': 1094, 'signals': (2, 14), 'sats': (3, 5, 8, 13, 15, 21, 26, 27)},
    'BeiDou': {'msg': 1124, 'signals': (2, 14), 'sats': (6, 9, 16, 19, 20, 21, 29, 35)},
}


def crc24q(data):
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


class BitWriter:
    def __init__(self):
        self.value = 0
        self.bits = 0

    def put(self, value, nbits):
        """append value (two's complement when negative) as nbits bits, most significant first"""
        if value < 0:
            value += 1 << nbits
        if not 0 <= value < (1 << nbits):
            raise ValueError(f'{value} does not fit in {nbits} bits')
        self.value = (self.value << nbits) | value
        self.bits += nbits

    def payload(self):
        pad = -self.bits % 8
        return (self.value << pad).to_bytes((self.bits + pad) // 8, 'big')


def frame(payload):
    head = bytes((PREAMBLE, (len(payload) >> 8) & 0x03, len(payload) & 0xFF)) + payload
    return head + crc24q(head).to_bytes(3, 'big')


def message_type(msg):
    """the message number of a framed RTCM3 message"""
    return (msg[3] << 4) | (msg[4] >> 4)


def msg_1005(station_id, ecef_m):
    w = BitWriter()
    w.put(1005, 12)
    w.put(station_id, 12)
    w.put(0, 6)                   # ITRF realization year
    w.put(1, 1)                   # GPS
    w.put(1, 1)                   # GLONASS
    w.put(1, 1)                   # Galileo
    w.put(0, 1)                   # a real reference station
    w.put(round(ecef_m[0] * 1e4), 38)
    w.put(1, 1)                   # single receiver oscillator
    w.put(0, 1)                   # reserved
    w.put(round(ecef_m[1] * 1e4), 38)
    w.put(0, 2)                   # quarter cycle indicator
    w.put(round(ecef_m[2] * 1e4), 38)
    return frame(w.payload())


def msm_epoch(gnss, gps_tow_ms, utc_seconds):
    """the 30-bit epoch time field of an MSM header"""
    if gnss == 'GLONASS':
        # day of week and time of day in Moscow time (UTC+3)
        moscow = utc_seconds + 3 * 3600
        day = int((moscow // 86400 + 4) % 7)     # 1970-01-01 was a Thursday, 0 = Sunday
        tod_ms = int((moscow % 86400) * 1000)
        return (day << 27) | tod_ms
    if gnss == 'BeiDou':
        return (gps_tow_ms - 14000) % (604800 * 1000)
    return gps_tow_ms


def msg_msm4(gnss, station_id, gps_tow_ms, utc_seconds, more_follow, t_since_start):
    """one MSM4 epoch: every satellite tracks every signal"""
    info = GNSS[gnss]
    sats, signals = info['sats'], info['signals']
    w = BitWriter()
    w.put(info['msg'], 12)
    w.put(station_id, 12)
    w.put(msm_epoch(gnss, gps_tow_ms, utc_seconds), 30)
    w.put(1 if more_follow else 0, 1)   # more MSMs of this epoch follow
    w.put(0, 3)                          # IODS
    w.put(0, 7)                          # reserved
    w.put(0, 2)                          # clock steering
    w.put(0, 2)                          # external clock
    w.put(0, 1)                          # divergence-free smoothing
    w.put(0, 3)                          # smoothing interval
    w.put(sum(1 << (64 - s) for s in sats), 64)
    w.put(sum(1 << (32 - s) for s in signals), 32)
    w.put((1 << (len(sats) * len(signals))) - 1, len(sats) * len(signals))

    # ranges of 20 000 - 25 000 km that drift with the orbits
    t = utc_seconds
    ranges_ms = [(20000 + 5000 * ((s * 0.6180339) % 1) + 600 * math.sin(t * 2 * math.pi / 43082 + s)) /
                 SPEED_OF_LIGHT_KM_PER_MS for s in sats]
    # rough ranges to 1/1024 ms; the fine fields hold the rest (int15 x 2^-24 ms reaches only +-2^-10 ms)
    rough = [round(r * 1024) / 1024 for r in ranges_ms]
    for r in rough:
        w.put(int(r), 8)
    for r in rough:
        w.put(int(round((r - int(r)) * 1024)) % 1024, 10)

    cells = [(i, k) for i in range(len(sats)) for k in range(len(signals))]
    # the second signal is delayed a little by the ionosphere
    exact = [ranges_ms[i] + k * 1.5e-5 for i, k in cells]
    for (i, k), r in zip(cells, exact):
        w.put(round((r - rough[i]) / 2 ** -24), 15)
    for (i, k), r in zip(cells, exact):
        w.put(round((r - rough[i] + 3e-7 * math.sin(t + i)) / 2 ** -29), 22)
    lock = min(15, int(math.log2(t_since_start + 1)))
    for _ in cells:
        w.put(lock, 4)
    for _ in cells:
        w.put(0, 1)                      # no half-cycle ambiguity
    for i, k in cells:
        w.put(47 - (sats[i] % 7) - 3 * k, 6)
    return frame(w.payload())


def msg_1230(station_id):
    w = BitWriter()
    w.put(1230, 12)
    w.put(station_id, 12)
    w.put(1, 1)          # biases aligned
    w.put(0, 3)          # reserved
    w.put(0b1010, 4)     # L1 C/A and L2 C/A biases follow
    w.put(0, 16)
    w.put(0, 16)
    return frame(w.payload())
