"""UBX framing and the messages a u-blox F9P base exchanges with LAAIRS."""

# AP_FLAKE8_CLEAN

import struct

SYNC = b'\xb5\x62'

ACK = 0x05
ACK_NAK, ACK_ACK = 0x00, 0x01
CFG = 0x06
CFG_MSG, CFG_TMODE3, CFG_VALSET = 0x01, 0x71, 0x8A
MON = 0x0A
MON_VER = 0x04
NAV = 0x01
NAV_PVT, NAV_SVIN = 0x07, 0x3B

# configuration keys (u-blox F9 interface description) the base acts on
KEY_TMODE_MODE = 0x20030001
KEY_TMODE_POS_TYPE = 0x20030002
KEY_TMODE_ECEF = (0x40030003, 0x40030004, 0x40030005)
KEY_TMODE_ECEF_HP = (0x20030006, 0x20030007, 0x20030008)
KEY_TMODE_LAT, KEY_TMODE_LON, KEY_TMODE_HEIGHT = 0x40030009, 0x4003000a, 0x4003000b
KEY_TMODE_LAT_HP, KEY_TMODE_LON_HP, KEY_TMODE_HEIGHT_HP = 0x2003000c, 0x2003000d, 0x2003000e
KEY_TMODE_FIXED_POS_ACC = 0x4003000f
KEY_TMODE_SVIN_MIN_DUR = 0x40030010
KEY_TMODE_SVIN_ACC_LIMIT = 0x40030011
KEY_RATE_MEAS = 0x30210001
SIGNED_KEYS = set(KEY_TMODE_ECEF + KEY_TMODE_ECEF_HP) | {
    KEY_TMODE_LAT, KEY_TMODE_LON, KEY_TMODE_HEIGHT, KEY_TMODE_LAT_HP, KEY_TMODE_LON_HP, KEY_TMODE_HEIGHT_HP}

# message output rates, USB and UART1 keys
MSGOUT_KEYS = {
    'NAV-PVT': (0x20910009, 0x20910007),
    'NAV-SVIN': (0x2091008b, 0x20910089),
    'RTCM1005': (0x209102c0, 0x209102be),
    'RTCM1074': (0x20910361, 0x2091035f),
    'RTCM1084': (0x20910366, 0x20910364),
    'RTCM1094': (0x2091036b, 0x20910369),
    'RTCM1124': (0x20910370, 0x2091036e),
    'RTCM1230': (0x20910306, 0x20910304),
}

NAV_PVT_FORMAT = '<IHBBBBBBIiBBBBiiiiIIiiiiiIIHH4sihH'
NAV_SVIN_FORMAT = '<B3sIIiiibbbBIIBB2s'


def frame(cls, msg_id, payload=b''):
    body = bytes((cls, msg_id)) + struct.pack('<H', len(payload)) + payload
    ck_a = ck_b = 0
    for byte in body:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return SYNC + body + bytes((ck_a, ck_b))


class Parser:
    """incremental UBX parser; anything else on the wire is skipped"""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf.extend(data)
        messages = []
        while True:
            start = self.buf.find(SYNC)
            if start < 0:
                del self.buf[:-1]
                return messages
            del self.buf[:start]
            if len(self.buf) < 6:
                return messages
            length = struct.unpack_from('<H', self.buf, 4)[0]
            if len(self.buf) < 8 + length:
                return messages
            msg = bytes(self.buf[:8 + length])
            if frame(msg[2], msg[3], msg[6:-2]) == msg:
                messages.append((msg[2], msg[3], msg[6:-2]))
                del self.buf[:8 + length]
            else:
                del self.buf[:2]


def parse_valset(payload):
    """{key: value} of a CFG-VALSET"""
    values = {}
    pos = 4
    sizes = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8}
    while pos + 4 <= len(payload):
        key = struct.unpack_from('<I', payload, pos)[0]
        size = sizes.get((key >> 28) & 0x7)
        if size is None or pos + 4 + size > len(payload):
            break
        values[key] = int.from_bytes(payload[pos + 4:pos + 4 + size], 'little', signed=key in SIGNED_KEYS)
        pos += 4 + size
    return values


def ack(cls, msg_id, accepted=True):
    return frame(ACK, ACK_ACK if accepted else ACK_NAK, bytes((cls, msg_id)))


def mon_ver():
    def text(s, n):
        return s.encode().ljust(n, b'\0')
    extensions = ('ROM BASE 0x118B2060', 'FWVER=HPG 1.32', 'PROTVER=27.31', 'MOD=ZED-F9P',
                  'GPS;GLO;GAL;BDS', 'QZSS', 'SIM=CopterSonde-sim twin')
    return frame(MON, MON_VER, text('EXT CORE 1.00 (fb2e6b)', 30) + text('00190000', 10) +
                 b''.join(text(e, 30) for e in extensions))


def nav_pvt(itow_ms, utc, fix_type, lat, lon, height_m, hmsl_m, h_acc_m, v_acc_m, num_sv):
    """utc: time.struct_time"""
    payload = struct.pack(
        NAV_PVT_FORMAT, itow_ms, utc.tm_year, utc.tm_mon, utc.tm_mday, utc.tm_hour, utc.tm_min, utc.tm_sec,
        0x07,                                   # valid date and time, fully resolved
        30, 0, fix_type,
        0x01 if fix_type else 0,                # gnssFixOK
        0xE0, num_sv,
        round(lon * 1e7), round(lat * 1e7), round(height_m * 1e3), round(hmsl_m * 1e3),
        round(h_acc_m * 1e3), round(v_acc_m * 1e3),
        0, 0, 0, 0, 0, 100, 18000000, 110, 0, b'\0' * 4, 0, 0, 0)
    return frame(NAV, NAV_PVT, payload)


def nav_svin(itow_ms, dur_s, mean_ecef_m, mean_acc_m, obs, valid, active):
    cm = [int(c * 100) for c in mean_ecef_m]
    hp = [round((c * 100 - int(c * 100)) * 100) for c in mean_ecef_m]
    hp = [max(-99, min(99, h)) for h in hp]
    payload = struct.pack(NAV_SVIN_FORMAT, 0, b'\0' * 3, itow_ms, dur_s, cm[0], cm[1], cm[2],
                          hp[0], hp[1], hp[2], 0, round(mean_acc_m * 1e4), obs,
                          1 if valid else 0, 1 if active else 0, b'\0' * 2)
    return frame(NAV, NAV_SVIN, payload)
