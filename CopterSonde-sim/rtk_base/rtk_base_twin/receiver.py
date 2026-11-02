"""The base receiver: a u-blox ZED-F9P configured by LAAIRS as an RTK base station.

It keeps the configuration LAAIRS sends (CFG-VALSET: message rates, measurement rate, TMODE survey-in or
fixed position), runs the survey-in and outputs NAV-PVT, NAV-SVIN and RTCM 1005, MSM4 and 1230 every
measurement epoch, like the real receiver. As shipped it outputs nothing until it is configured
(the real one sends NMEA, which LAAIRS ignores).
"""

# AP_FLAKE8_CLEAN

import math
import random
import time

from . import rtcm, ubx

# WGS-84
A = 6378137.0
F = 1 / 298.257223563
E2 = F * (2 - F)
GPS_EPOCH_UNIX = 315964800
GPS_LEAP_SECONDS = 18


def llh_to_ecef(lat, lon, h):
    la, lo = math.radians(lat), math.radians(lon)
    n = A / math.sqrt(1 - E2 * math.sin(la) ** 2)
    return ((n + h) * math.cos(la) * math.cos(lo),
            (n + h) * math.cos(la) * math.sin(lo),
            (n * (1 - E2) + h) * math.sin(la))


def ecef_to_llh(x, y, z):
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1 - E2))
    for _ in range(6):
        n = A / math.sqrt(1 - E2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1 - E2 * n / (n + h)))
    return math.degrees(lat), math.degrees(lon), h


def enu_to_ecef_delta(lat, lon, e, n, u):
    la, lo = math.radians(lat), math.radians(lon)
    return (-math.sin(lo) * e - math.sin(la) * math.cos(lo) * n + math.cos(la) * math.cos(lo) * u,
            math.cos(lo) * e - math.sin(la) * math.sin(lo) * n + math.cos(la) * math.sin(lo) * u,
            math.cos(la) * n + math.sin(la) * u)


def gps_tow_ms(unix):
    return int(((unix - GPS_EPOCH_UNIX + GPS_LEAP_SECONDS) % 604800) * 1000)


class Receiver:
    STANDALONE_SIGMA_H = 1.0     # m per axis, like SIM_GPS_RTK in SITL
    STANDALONE_SIGMA_V = 1.5
    STANDALONE_TAU = 60.0        # s
    SVIN_SIGMA0 = 2.5            # survey-in mean accuracy: SVIN_SIGMA0 * sqrt(SVIN_TAU / (t + SVIN_TAU))
    SVIN_TAU = 30.0

    def __init__(self, lat, lon, height_msl, geoid_m=-27.0, station_id=0, log=print, seed=None):
        self.lat, self.lon, self.hmsl = lat, lon, height_msl
        self.geoid = geoid_m
        self.station_id = station_id
        self.log = log
        self.rng = random.Random(seed)
        self.truth_ecef = llh_to_ecef(lat, lon, height_msl + geoid_m)
        self.error_enu = [0.0, 0.0, 0.0]
        self.last_t = None
        self.rates = {name: 0 for name in ubx.MSGOUT_KEYS}
        self.meas_period_ms = 1000
        self.tmode = 0
        self.svin_min_dur = 0
        self.svin_acc_limit_m = 0.0
        self.svin_start = None
        self.svin_dir = (0.0, 0.0, 0.0)
        self.svin_valid = False
        self.svin_done = None          # (dur, accuracy) once valid
        self.arp_ecef = None           # the base position in 1005 (surveyed or fixed)
        self.arp_acc_m = 0.0
        self.start = None
        self.epochs = 0

    # ---- configuration from LAAIRS ----------------------------------------------------------------
    def handle(self, cls, msg_id, payload, now):
        """process one UBX message; returns the bytes to answer with"""
        if cls == ubx.MON and msg_id == ubx.MON_VER and not payload:
            return ubx.mon_ver()
        if cls == ubx.CFG and msg_id == ubx.CFG_VALSET:
            self.valset(ubx.parse_valset(payload), now)
            return ubx.ack(cls, msg_id)
        if cls == ubx.CFG and msg_id in (ubx.CFG_MSG, ubx.CFG_TMODE3):
            # M8 configuration: accepted, not modelled (LAAIRS treats the base as an F9P)
            return ubx.ack(cls, msg_id)
        return b''

    def valset(self, values, now):
        for name, keys in ubx.MSGOUT_KEYS.items():
            if keys[0] in values or keys[1] in values:
                self.rates[name] = max(values.get(keys[0], 0), values.get(keys[1], 0))
        if ubx.KEY_RATE_MEAS in values and values[ubx.KEY_RATE_MEAS] >= 25:
            self.meas_period_ms = values[ubx.KEY_RATE_MEAS]
        if ubx.KEY_TMODE_SVIN_MIN_DUR in values:
            self.svin_min_dur = values[ubx.KEY_TMODE_SVIN_MIN_DUR]
        if ubx.KEY_TMODE_SVIN_ACC_LIMIT in values:
            self.svin_acc_limit_m = values[ubx.KEY_TMODE_SVIN_ACC_LIMIT] * 1e-4
        if ubx.KEY_TMODE_MODE not in values:
            return
        mode = values[ubx.KEY_TMODE_MODE]
        if mode == 1 and self.tmode != 1:
            self.svin_start = now
            self.svin_valid = False
            self.svin_done = None
            self.arp_ecef = None
            d = [self.rng.gauss(0, 1) for _ in range(3)]
            norm = math.sqrt(sum(c * c for c in d)) or 1.0
            self.svin_dir = tuple(c / norm for c in d)
            self.log(f'survey-in started: at least {self.svin_min_dur} s, to {self.svin_acc_limit_m:.3f} m')
        elif mode == 2:
            self.arp_ecef = self.fixed_position(values)
            self.arp_acc_m = values.get(ubx.KEY_TMODE_FIXED_POS_ACC, 0) * 1e-4
            lat, lon, h = ecef_to_llh(*self.arp_ecef)
            self.log(f'fixed base position {lat:.7f} {lon:.7f} {h:.2f} m (ellipsoid)')
        elif mode == 0 and self.tmode != 0:
            self.arp_ecef = None
            self.svin_start = None
            self.log('base mode off')
        self.tmode = mode

    def fixed_position(self, v):
        if v.get(ubx.KEY_TMODE_POS_TYPE, 0) == 1:
            lat = (v.get(ubx.KEY_TMODE_LAT, 0) + v.get(ubx.KEY_TMODE_LAT_HP, 0) * 0.01) * 1e-7
            lon = (v.get(ubx.KEY_TMODE_LON, 0) + v.get(ubx.KEY_TMODE_LON_HP, 0) * 0.01) * 1e-7
            h = (v.get(ubx.KEY_TMODE_HEIGHT, 0) + v.get(ubx.KEY_TMODE_HEIGHT_HP, 0) * 0.01) * 0.01
            return llh_to_ecef(lat, lon, h)
        return tuple((v.get(k, 0) + v.get(hp, 0) * 0.01) * 0.01
                     for k, hp in zip(ubx.KEY_TMODE_ECEF, ubx.KEY_TMODE_ECEF_HP))

    # ---- the receiver's own position ----------------------------------------------------------------
    def standalone_ecef(self, now):
        if self.last_t is not None:
            dt = min(now - self.last_t, 1.0)
            a = math.exp(-dt / self.STANDALONE_TAU)
            b = math.sqrt(1 - a * a)
            sig = (self.STANDALONE_SIGMA_H, self.STANDALONE_SIGMA_H, self.STANDALONE_SIGMA_V)
            self.error_enu = [a * e + s * b * self.rng.gauss(0, 1) for e, s in zip(self.error_enu, sig)]
        self.last_t = now
        d = enu_to_ecef_delta(self.lat, self.lon, *self.error_enu)
        return tuple(t + c for t, c in zip(self.truth_ecef, d))

    def survey(self, now):
        """(duration s, mean position, mean accuracy m)"""
        dur = int(now - self.svin_start)
        acc = self.SVIN_SIGMA0 * math.sqrt(self.SVIN_TAU / (dur + self.SVIN_TAU))
        mean = tuple(t + 0.5 * acc * c for t, c in zip(self.truth_ecef, self.svin_dir))
        return dur, mean, acc

    # ---- one measurement epoch ----------------------------------------------------------------------
    def epoch(self, now):
        """the bytes the receiver sends at this epoch"""
        if self.start is None:
            self.start = now
        self.epochs += 1
        utc = time.gmtime(now)
        itow = gps_tow_ms(now)
        own = self.standalone_ecef(now)
        out = []

        svin = None
        if self.tmode == 1 and self.svin_start is not None:
            dur, mean, acc = self.survey(now)
            if not self.svin_valid and dur >= self.svin_min_dur and acc <= self.svin_acc_limit_m:
                self.svin_valid = True
                self.svin_done = (dur, acc)
                self.arp_ecef, self.arp_acc_m = mean, acc
                lat, lon, h = ecef_to_llh(*mean)
                self.log(f'survey-in complete after {dur} s, {acc:.3f} m: base at {lat:.7f} {lon:.7f} '
                         f'{h:.2f} m (ellipsoid); sending RTCM 1005')
            if self.svin_valid:
                dur, acc = self.svin_done
                mean = self.arp_ecef
            svin = (dur, mean, acc)

        if self.rates['NAV-PVT'] and self.epochs % self.rates['NAV-PVT'] == 0:
            if self.arp_ecef is not None:
                lat, lon, h = ecef_to_llh(*self.arp_ecef)
                out.append(ubx.nav_pvt(itow, utc, 5, lat, lon, h, h - self.geoid,
                                       self.arp_acc_m, self.arp_acc_m, 31))
            else:
                lat, lon, h = ecef_to_llh(*own)
                out.append(ubx.nav_pvt(itow, utc, 3, lat, lon, h, h - self.geoid, 1.5, 2.5, 31))
        if self.rates['NAV-SVIN'] and self.epochs % self.rates['NAV-SVIN'] == 0:
            if svin is not None:
                dur, mean, acc = svin
                out.append(ubx.nav_svin(itow, dur, mean, acc, dur, self.svin_valid, not self.svin_valid))
            else:
                out.append(ubx.nav_svin(itow, 0, (0.0, 0.0, 0.0), 0.0, 0, False, False))

        if self.arp_ecef is not None and self.due('RTCM1005'):
            out.append(rtcm.msg_1005(self.station_id, self.arp_ecef))
        msm = [g for g, info in rtcm.GNSS.items() if self.due(f'RTCM{info["msg"]}')]
        for i, g in enumerate(msm):
            out.append(rtcm.msg_msm4(g, self.station_id, itow, now, i < len(msm) - 1, now - self.start))
        if self.due('RTCM1230'):
            out.append(rtcm.msg_1230(self.station_id))
        return b''.join(out)

    def due(self, name):
        return self.rates[name] and self.epochs % self.rates[name] == 0
