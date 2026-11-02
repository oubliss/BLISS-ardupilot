#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""
Fly a CopterSonde vertical profile (AutoVP) in the Gazebo simulator.

Flips the AutoVP switch (RC7, RC7_OPTION 47) on the ground, so the firmware builds its mission
(take-off, 10 m, USR_AUTOVP_ALT, RTL), arms in LOITER, switches to AUTO (AUTO_OPTIONS 6 takes off
without raising the throttle) and waits until the vehicle has landed and disarmed. It prints the
altitude, the iMet temperature and HYT271 humidity (CASS_SENSOR_RAW) and the battery every --every
seconds.

Start Gazebo (coptersonde_gazebo.sh, optionally with --truth-log) and SITL (coptersonde_sitl.sh)
first, and connect a ground station to 5760 if SITL is waiting for one. Then compare the flight
with the truth: profile_eval.py LOG.BIN TRUTH.csv

  python3 vp_flight_test.py --alt 2438                 # SITL instance 0, MAVLink on TCP 5762
  python3 vp_flight_test.py --master tcp:127.0.0.1:5782 --alt 500
  python3 vp_flight_test.py --alt 3048 --param USR_VPBATT_EN=0    # warn only, no battery RTL
"""

import argparse
import importlib.util
import os
import subprocess
import sys
import time

os.environ.setdefault('MAVLINK20', '1')
from pymavlink import mavutil  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def cs_dialect():
    """The CopterSonde dialect (CASS_SENSOR_RAW) generated from modules/mavlink, cached."""
    cache = os.path.join(os.path.expanduser('~'), '.cache', 'coptersonde')
    out = os.path.join(cache, 'cs_ardupilotmega.py')
    xml = os.path.join(REPO, 'modules', 'mavlink', 'message_definitions', 'v1.0', 'ardupilotmega.xml')
    if not os.path.exists(out) or os.path.getmtime(out) < os.path.getmtime(xml):
        os.makedirs(cache, exist_ok=True)
        subprocess.run([sys.executable, '-m', 'pymavlink.tools.mavgen', '--lang=Python3', '--wire-protocol=2.0',
                        '--output=' + out, xml], cwd=os.path.join(REPO, 'modules', 'mavlink'), check=True,
                       stdout=subprocess.DEVNULL)
    spec = importlib.util.spec_from_file_location('cs_ardupilotmega', out)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Vehicle:
    def __init__(self, master):
        mavutil.mavlink = cs_dialect()
        self.m = mavutil.mavlink_connection(master, source_system=255)
        self.m.wait_heartbeat(timeout=120)
        self.state = {}
        self.texts = []
        for sid in (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, mavutil.mavlink.MAV_DATA_STREAM_EXTRA3):
            self.m.mav.request_data_stream_send(self.m.target_system, self.m.target_component, sid, 4, 1)
        self.rc = [1500, 1500, 1000, 1500, 0, 0, 1100, 0]
        self.last_rc = 0

    def pump(self, seconds):
        end = time.time() + seconds
        while True:
            if time.time() - self.last_rc > 0.2:
                self.m.mav.rc_channels_override_send(self.m.target_system, self.m.target_component, *self.rc)
                self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                          0, 0, 0)
                self.last_rc = time.time()
            left = end - time.time()
            if left <= 0:
                return
            msg = self.m.recv_match(blocking=True, timeout=min(left, 0.2))
            if msg is None:
                continue
            ty = msg.get_type()
            if ty == 'STATUSTEXT':
                self.texts.append(msg.text)
                print('  [fc] ' + msg.text, flush=True)
            elif ty == 'CASS_SENSOR_RAW':
                self.state['cass%d' % msg.app_datatype] = list(msg.values)
            else:
                self.state[ty] = msg

    def armed(self):
        hb = self.state.get('HEARTBEAT')
        return hb is not None and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

    def alt(self):
        p = self.state.get('GLOBAL_POSITION_INT')
        return p.relative_alt * 1e-3 if p else float('nan')

    def set_param(self, name, value):
        self.m.mav.param_set_send(self.m.target_system, self.m.target_component, name.encode(), value,
                                  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        self.pump(1)

    def wait_text(self, text, timeout):
        end = time.time() + timeout
        while time.time() < end:
            if any(text in t for t in self.texts):
                return True
            self.pump(0.5)
        return False


def mean(values):
    v = [x for x in values[:4] if x > 0]
    return sum(v) / len(v) if v else float('nan')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--master', default='tcp:127.0.0.1:5762', help='SITL MAVLink (default: instance 0 SERIAL1)')
    ap.add_argument('--alt', type=float, help='set USR_AUTOVP_ALT (m) before the flight')
    ap.add_argument('--param', action='append', default=[], metavar='NAME=VALUE',
                    help='set a parameter before the flight (repeatable), e.g. USR_VPBATT_EN=0')
    ap.add_argument('--every', type=float, default=20, help='status print interval (s)')
    ap.add_argument('--timeout', type=float, default=3600, help='give up after this many seconds')
    args = ap.parse_args()

    v = Vehicle(args.master)
    print('connected to system %d' % v.m.target_system, flush=True)
    if args.alt is not None:
        v.set_param('USR_AUTOVP_ALT', args.alt)
    for p in args.param:
        name, value = p.split('=')
        v.set_param(name, float(value))
    # build the AutoVP mission as soon as the EKF has a position
    built = False
    t0 = time.time()
    while not built and time.time() - t0 < 300:
        v.rc[6] = 1900
        v.pump(1.5)
        built = any('AutoVP mission received' in t for t in v.texts)
        v.rc[6] = 1100
        v.pump(5.5 if not built else 1)
    if not built:
        sys.exit('AutoVP did not build a mission (EKF not ready?)')
    # arm in LOITER with the throttle down, then AUTO takes off by itself (AUTO_OPTIONS bit 1)
    v.m.set_mode('LOITER')
    v.pump(1)
    for _ in range(30):
        v.m.arducopter_arm()
        v.pump(2)
        if v.armed():
            break
    if not v.armed():
        sys.exit('could not arm')
    v.m.set_mode('AUTO')
    v.pump(1)
    print('armed, AUTO, profiling', flush=True)
    t0 = time.time()
    last = 0
    top = 0
    while time.time() - t0 < args.timeout:
        v.pump(1)
        top = max(top, v.alt())
        if time.time() - last >= args.every:
            last = time.time()
            sys_status = v.state.get('SYS_STATUS')
            print('t %5.0f s  alt %7.1f m  T %6.2f C  RH %5.1f %%  batt %5.2f V %5.1f A' % (
                time.time() - t0, v.alt(), mean(v.state.get('cass0', [0])) - 273.15, mean(v.state.get('cass1', [0])),
                sys_status.voltage_battery * 1e-3 if sys_status else float('nan'),
                sys_status.current_battery * 1e-2 if sys_status else float('nan')), flush=True)
        if top > 5 and not v.armed():
            print('landed and disarmed after %.0f s, top %.0f m' % (time.time() - t0, top), flush=True)
            return
    sys.exit('timed out')


if __name__ == '__main__':
    main()
