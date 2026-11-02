"""Tests of the RTK base twin (CopterSonde-sim/rtk_base) against the u-blox and RTCM decoders LAAIRS uses.

They need what LAAIRS uses to read the base (pyubx2, pyrtcm, pyserial) and are skipped without them:
    pip install pyubx2 pyrtcm pyserial
    python -m pytest CopterSonde-sim/tests/test_rtk_base_twin.py
"""

# AP_FLAKE8_CLEAN

import io
import os
import sys
import threading
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'rtk_base'))

from rtk_base_twin import rtcm, ubx  # noqa: E402
from rtk_base_twin.receiver import Receiver, ecef_to_llh, llh_to_ecef  # noqa: E402
from rtk_base_twin.server import BaseServer  # noqa: E402

pyubx2 = pytest.importorskip('pyubx2')
pytest.importorskip('pyrtcm')

SITE = (35.2380802, -97.4631246, 358.6)

# what LAAIRS sends (rtk_base_manager.py _configure_gen9 and _do_start_survey)
LAAIRS_CONFIG = [
    ('CFG_RATE_MEAS', 1000), ('CFG_RATE_NAV', 1), ('CFG_NAVSPG_DYNMODEL', 2),
    ('CFG_USBOUTPROT_NMEA', 0), ('CFG_USBOUTPROT_UBX', 1), ('CFG_USBOUTPROT_RTCM3X', 1),
    ('CFG_MSGOUT_RTCM_3X_TYPE1005_USB', 1), ('CFG_MSGOUT_RTCM_3X_TYPE1074_USB', 1),
    ('CFG_MSGOUT_RTCM_3X_TYPE1084_USB', 1), ('CFG_MSGOUT_RTCM_3X_TYPE1094_USB', 1),
    ('CFG_MSGOUT_RTCM_3X_TYPE1124_USB', 1), ('CFG_MSGOUT_RTCM_3X_TYPE1230_USB', 5),
    ('CFG_MSGOUT_UBX_NAV_SVIN_USB', 1), ('CFG_MSGOUT_UBX_NAV_PVT_USB', 1),
]


def survey_in(min_dur, acc_m):
    return [('CFG_TMODE_MODE', 1), ('CFG_TMODE_SVIN_MIN_DUR', min_dur), ('CFG_TMODE_SVIN_ACC_LIMIT', int(acc_m * 1e4)),
            ('CFG_MSGOUT_UBX_NAV_SVIN_USB', 1), ('CFG_MSGOUT_UBX_NAV_PVT_USB', 1)]


def send(receiver, msg, now=0.0):
    (cls, msg_id, payload), = ubx.Parser().feed(msg.serialize())
    return receiver.handle(cls, msg_id, payload, now)


def decode(data):
    reader = pyubx2.UBXReader(io.BytesIO(data), protfilter=pyubx2.UBX_PROTOCOL | pyubx2.RTCM3_PROTOCOL)
    return [msg for _, msg in reader]


def configured_receiver():
    r = Receiver(*SITE, log=lambda _: None, seed=1)
    ack, = decode(send(r, pyubx2.UBXMessage.config_set(1, 0, LAAIRS_CONFIG)))
    assert ack.identity == 'ACK-ACK'
    return r


def test_mon_ver_answers_the_probe():
    r = Receiver(*SITE, log=lambda _: None)
    ver, = decode(send(r, pyubx2.UBXMessage('MON', 'MON-VER', pyubx2.POLL)))
    assert ver.identity == 'MON-VER' and b'MOD=ZED-F9P' in b''.join(
        getattr(ver, f'extension_{i:02d}') for i in range(1, 8))
    # unconfigured, like the real receiver it sends nothing LAAIRS reads
    assert r.epoch(1.0) == b''


def test_rtcm_frames_decode():
    ecef = llh_to_ecef(*SITE)
    msg, = decode(rtcm.msg_1005(7, ecef))
    assert msg.identity == '1005' and msg.DF003 == 7
    assert (msg.DF025, msg.DF026, msg.DF027) == pytest.approx(ecef, abs=1e-4)
    for i, (gnss, info) in enumerate(rtcm.GNSS.items()):
        msm, = decode(rtcm.msg_msm4(gnss, 7, 123456789, 1.8e9, i < 3, 100.0))
        assert msm.identity == str(info['msg'])
        assert (msm.NSat, msm.NSig, msm.NCell) == (len(info['sats']), 2, 2 * len(info['sats']))
        assert 60 < msm.DF397_01 < 90                     # 18 000 - 27 000 km
    msg, = decode(rtcm.msg_1230(7))
    assert msg.identity == '1230'


def test_valset_signed_values():
    r = Receiver(*SITE, log=lambda _: None)
    lat, lon, h = 35.2381234567, -97.4631234567, 331.25
    # encoded as LAAIRS _set_fixed_gen9 does
    lat_main, lon_main = int(lat * 1e7), int(lon * 1e7)
    send(r, pyubx2.UBXMessage.config_set(1, 0, [
        ('CFG_TMODE_MODE', 2), ('CFG_TMODE_POS_TYPE', 1),
        ('CFG_TMODE_LAT', lat_main), ('CFG_TMODE_LAT_HP', int((lat * 1e7 - lat_main) * 100)),
        ('CFG_TMODE_LON', lon_main), ('CFG_TMODE_LON_HP', int((lon * 1e7 - lon_main) * 100)),
        ('CFG_TMODE_HEIGHT', int(h * 100)), ('CFG_TMODE_HEIGHT_HP', 0), ('CFG_TMODE_FIXED_POS_ACC', 10)]))
    got = ecef_to_llh(*r.arp_ecef)
    assert got[0] == pytest.approx(lat, abs=1e-8) and got[1] == pytest.approx(lon, abs=1e-8)
    assert got[2] == pytest.approx(h, abs=1e-3)


def test_survey_in_then_corrections():
    r = configured_receiver()
    pvt = [m for m in decode(r.epoch(0.0)) if m.identity == 'NAV-PVT'][0]
    assert pvt.fixType == 3 and pvt.hAcc == 1500
    send(r, pyubx2.UBXMessage.config_set(1, 0, survey_in(10, 2.0)), now=1.0)
    valid_at = None
    for t in range(1, 60):
        msgs = {m.identity: m for m in decode(r.epoch(float(t)))}
        svin = msgs['NAV-SVIN']
        assert {'1074', '1084', '1094', '1124'} <= set(msgs)
        if svin.valid:
            valid_at = valid_at or t
            assert not svin.active and '1005' in msgs and msgs['NAV-PVT'].fixType == 5
        else:
            assert svin.active and svin.dur == t - 1 and '1005' not in msgs
    # 2.5 m * sqrt(30 / (t + 30)) reaches 2.0 m after 16.9 s
    assert valid_at == 18


def test_fixed_base_position_stands_still():
    r = configured_receiver()
    send(r, pyubx2.UBXMessage.config_set(1, 0, survey_in(1, 3.0)), now=0.0)
    first = [m for m in decode(r.epoch(5.0)) if m.identity == '1005'][0]
    later = [m for m in decode(r.epoch(200.0)) if m.identity == '1005'][0]
    assert (first.DF025, first.DF026, first.DF027) == (later.DF025, later.DF026, later.DF027)


def test_serial_url_like_laairs():
    serial = pytest.importorskip('serial')
    r = Receiver(*SITE, log=lambda _: None, seed=2)
    server = BaseServer(r, '127.0.0.1', 0, log=lambda _: None)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    url = f'socket://127.0.0.1:{server.address[1]}'
    try:
        # the baud probe: MON-VER poll, then any UBX answer
        probe = serial.serial_for_url(url, baudrate=115200, timeout=0.1)
        probe.write(pyubx2.UBXMessage('MON', 'MON-VER', pyubx2.POLL).serialize())
        buf, deadline = b'', time.time() + 2
        while b'\xb5\x62' not in buf and time.time() < deadline:
            buf += probe.read(256)
        probe.close()
        assert b'\xb5\x62' in buf

        # reopened, configured and surveyed in: corrections stream once the survey is valid
        port = serial.serial_for_url(url, baudrate=115200, timeout=0.1)
        port.write(pyubx2.UBXMessage.config_set(1, 0, LAAIRS_CONFIG).serialize())
        port.write(pyubx2.UBXMessage.config_set(1, 0, survey_in(1, 3.0)).serialize())
        seen, data, deadline = set(), b'', time.time() + 6
        while not {'ACK-ACK', '1005', '1074'} <= seen and time.time() < deadline:
            data += port.read(4096)
            seen = {m.identity for m in decode(data)}
        port.close()
        assert {'ACK-ACK', 'NAV-SVIN', 'NAV-PVT', '1005', '1074', '1084', '1094', '1124'} <= seen
    finally:
        server.shutdown()
