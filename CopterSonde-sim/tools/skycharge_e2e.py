#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""
End-to-end test of the CopterSonde charging on the Skycharge pad: SITL + Gazebo (the CopterSonde on the
BOLOGNINI S1), the Skycharge digital twin, the RTK base twin, and LAAIRS's own Skycharge and RTK base managers
driving them as in the field.

Start the simulator first, e.g. on instance 7 with a small pack so the final charge is short:
  coptersonde_sim.sh -I 7 --headless --battery-ah 1 --battery-soc 0.9
Then run this with LAAIRS's Python (it imports laairs.core from --laairs), e.g. on Windows:
  python skycharge_e2e.py --laairs C:/Users/me/Github/BLISS-GCS/src --master tcp:127.0.0.1:5830
      --pad 127.0.0.1:2292 --rtk-base socket://127.0.0.1:3171

The phases, each checked:
  0. RTK: without corrections the drone has a standalone 3D fix; LAAIRS surveys the base in and injects its RTCM,
     and the drone turns RTK FIXED.
  1. Start-up on the pad, disarmed: LAAIRS enables scanning, the pad detects the drone and charges; the FC sees the
     pack voltage rise and LAAIRS blocks arming. With RTK streaming, LAAIRS now requires RTK FIXED to fly.
  2. The operator skips charging: the pad ends with CHARGING_FORCIBLY_STOPPED, which LAAIRS must not call full.
  3. Take-off (GUIDED) in RTK FIXED: LAAIRS stops the pad as the drone arms; hover; RTL lands back on the pad,
     RTK FIXED all the way.
  4. Landing: the pad stays parked until the post-flight checklist is acknowledged, then charges until the current
     falls below the cutoff (REACHED_CURRENT_CUTOFF), which LAAIRS calls full.
"""

import argparse
import os
import sys
import tempfile
import time

os.environ.setdefault('MAVLINK20', '1')
from pymavlink import mavutil  # noqa: E402


class Fail(Exception):
    pass


class Vehicle:
    """SITL over MAVLink, as a ground station (RC override and GCS heartbeat, like vp_flight_test.py)."""

    def __init__(self, master):
        self.m = mavutil.mavlink_connection(master, source_system=255)
        self.m.wait_heartbeat(timeout=120)
        for sid in (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1):
            self.m.mav.request_data_stream_send(self.m.target_system, self.m.target_component, sid, 4, 1)
        self.state = {}
        self.texts = []
        self.rc = [1500, 1500, 1000, 1500, 0, 0, 1100, 0]
        self.last_rc = 0
        self.on_update = None
        self.flight_fixes = []   # GPS fix types while armed

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
                print('      [fc] ' + msg.text, flush=True)
            elif ty == 'HEARTBEAT' and msg.get_srcComponent() != 1:
                continue
            else:
                self.state[ty] = msg
            if ty == 'GPS_RAW_INT' and self.armed():
                self.flight_fixes.append(msg.fix_type)
            if self.on_update and ty in ('HEARTBEAT', 'SYS_STATUS', 'BATTERY_STATUS'):
                self.on_update(self)

    def armed(self):
        hb = self.state.get('HEARTBEAT')
        return hb is not None and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

    def mode(self):
        hb = self.state.get('HEARTBEAT')
        return mavutil.mode_mapping_acm.get(hb.custom_mode, '?') if hb else '?'

    def alt(self):
        p = self.state.get('GLOBAL_POSITION_INT')
        return p.relative_alt * 1e-3 if p else float('nan')

    def battery(self):
        s = self.state.get('SYS_STATUS')
        b = self.state.get('BATTERY_STATUS')
        if s is None:
            return None, None, None, None
        level = s.battery_remaining if s.battery_remaining >= 0 else None
        consumed = b.current_consumed if b is not None and b.current_consumed >= 0 else None
        return level, s.voltage_battery * 1e-3, s.current_battery * 1e-2, consumed

    def fix(self):
        g = self.state.get('GPS_RAW_INT')
        return g.fix_type if g else None

    def set_param(self, name, value):
        self.m.mav.param_set_send(self.m.target_system, self.m.target_component, name.encode(), value,
                                  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        self.pump(1)


class Laairs:
    """The LAAIRS pieces of the charging workflow (event bus + SkychargeManager), without the GUI."""

    def __init__(self, src, host, port, capacity_mah, config_file):
        sys.path.insert(0, src)
        from laairs.core.event_bus import EventBus, EventType
        from laairs.core.rtk_base_manager import RTKBaseManager, RTKState
        from laairs.core.skycharge_manager import SkychargeBatteryConfig, SkychargeManager
        self.RTKBaseManager, self.RTKState = RTKBaseManager, RTKState
        self.rtk = None
        self.events = EventType
        self.bus = EventBus()
        self.mgr = SkychargeManager(self.bus, config_file=config_file)
        self.mgr.set_connection_target('custom', host, port)
        self.mgr.batt_cfg = SkychargeBatteryConfig(capacity_mah=capacity_mah)
        self.mgr.policy.auto_manage = True
        self.mgr.policy.block_arming = True
        self.mgr.conn_cfg.poll_interval = 1.0
        self._log_seq = 0

    def drone(self, vehicle):
        """What LAAIRS's DataManager and Skycharge window pass on from MAVLink."""
        self.bus.emit(self.events.DATA_UPDATED, {'is_armed': vehicle.armed()})
        level, voltage, _current, consumed = vehicle.battery()
        self.mgr.update_drone_battery(level, voltage, consumed)

    def rtk_streaming(self):
        return self.rtk is not None and self.rtk.get_status().state == self.RTKState.STREAMING

    def precision_landing_required(self):
        """LAAIRS's ConnectionWindow._precision_landing_required: Skycharge on and the RTK base streaming."""
        return self.mgr.enabled and self.rtk_streaming()

    def print_log(self):
        for seq, ts, level, text in self.mgr.get_log_entries(self._log_seq):
            self._log_seq = seq
            print(f'      [laairs {ts}] {"" if level == "INFO" else level + " "}{text}', flush=True)


def wait_for(v, laairs, what, cond, timeout, every=10.0):
    t0 = time.time()
    last = 0
    while time.time() - t0 < timeout:
        v.pump(0.5)
        laairs.print_log()
        if cond():
            print(f'  ok: {what} ({time.time() - t0:.0f} s)', flush=True)
            return
        if time.time() - last > every:
            last = time.time()
            s = laairs.mgr.get_status()
            level, volt, curr, _ = v.battery()
            print(f'    .. {what}: pad {s.raw_state or s.state} {s.current:.2f} A {s.pad_voltage:.2f} V, '
                  f'FC {volt or 0:.2f} V {curr or 0:.1f} A, alt {v.alt():.1f} m, {v.mode()}'
                  f'{" ARMED" if v.armed() else ""}, SoC est {s.soc_estimate}', flush=True)
    raise Fail(f'timed out waiting for: {what}')


def check(ok, what):
    if not ok:
        raise Fail(what)
    print(f'  ok: {what}', flush=True)


def run(args):
    print('connecting to SITL', args.master, flush=True)
    v = Vehicle(args.master)
    laairs = Laairs(args.laairs, args.pad_host, args.pad_port, args.capacity,
                    os.path.join(tempfile.mkdtemp(), 'skycharge_e2e.json'))
    v.on_update = laairs.drone
    v.set_param('BATT_CAPACITY', args.capacity)
    v.pump(3)
    mgr = laairs.mgr
    st = mgr.get_status
    fix_names = {3: '3D', 4: 'DGPS', 5: 'RTK FLOAT', 6: 'RTK FIXED'}

    if args.rtk_base:
        print('\n0. RTK base: survey-in and corrections', flush=True)
        wait_for(v, laairs, 'standalone GPS fix', lambda: (v.fix() or 0) >= 3, 120)
        check(v.fix() < 5, f'no RTK without corrections ({fix_names.get(v.fix(), v.fix())})')
        laairs.rtk = laairs.RTKBaseManager(mavlink_connection=v.m, measurement_rate_hz=1)
        check(laairs.rtk.connect(args.rtk_base), f'LAAIRS connects to the RTK base at {args.rtk_base}')
        wait_for(v, laairs, 'base configured', lambda: laairs.rtk.state == laairs.RTKState.CONFIGURING
                 and laairs.rtk.get_message_stats()['msg_counts'].get('NAV-PVT'), 60)
        check(laairs.rtk.start_survey(min_time=60, accuracy_limit=2.0), 'survey-in started (60 s, 2.0 m)')
        wait_for(v, laairs, 'survey-in complete, RTCM streaming', laairs.rtk_streaming, 180, every=15)
        wait_for(v, laairs, 'drone RTK FIXED', lambda: v.fix() == 6, 60)
        r = laairs.rtk.get_status()
        print(f'    base: survey {r.survey_status.observation_time} s to {r.survey_status.mean_accuracy:.2f} m, '
              f'{r.rtcm_messages_sent} RTCM messages injected', flush=True)

    print('\n1. on the pad at start-up, disarmed', flush=True)
    mgr.set_enabled(True)            # the site-arrival checklist's "Use Skycharge charging pad"
    wait_for(v, laairs, 'pad link', lambda: st().connected, 60)
    wait_for(v, laairs, 'pad detects the drone', lambda: st().is_on_pad, 60)
    wait_for(v, laairs, 'charging current flows', lambda: st().is_charging and st().current > 1.0, 90)
    v0 = v.battery()[1]
    wait_for(v, laairs, 'arming lock engaged', lambda: st().arming_locked, 30)
    v.pump(15)
    check(v.battery()[1] > v0 + 0.05, f'FC pack voltage rises while charging ({v0:.2f} -> {v.battery()[1]:.2f} V)')
    if args.rtk_base:
        check(laairs.precision_landing_required(), 'LAAIRS precision landing mode: RTK FIXED required to fly')

    print('\n2. operator skips the charge', flush=True)
    mgr.override_and_skip()
    wait_for(v, laairs, 'pad ends the charge', lambda: st().state == 'charged', 30)
    s = st()
    check(s.stop_reason == 'CHARGING_FORCIBLY_STOPPED' and not s.is_charged and s.charge_end == 'stopped',
          f'early end reported as stopped, not full ({s.stop_reason})')
    check(not s.arming_locked, 'arming lock released')

    print('\n3. take-off, hover, RTL back onto the pad', flush=True)
    wait_for(v, laairs, 'position for GUIDED', lambda: v.state.get('GPS_RAW_INT') is not None
             and v.state['GPS_RAW_INT'].fix_type >= 3, 120)
    if args.rtk_base:
        # LAAIRS's Arm & Takeoff gate: with precision landing required it asks before arming without RTK FIXED
        check(v.fix() == 6, f'RTK FIXED at take-off ({fix_names.get(v.fix(), v.fix())})')
    v.m.set_mode('GUIDED')
    v.pump(2)
    for _ in range(30):
        v.m.arducopter_arm()
        v.pump(2)
        if v.armed():
            break
    check(v.armed(), 'armed in GUIDED')
    v.m.mav.command_long_send(v.m.target_system, v.m.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                              0, 0, 0, 0, 0, 0, 0, args.alt)
    wait_for(v, laairs, 'LAAIRS suspends the pad for flight', lambda: st().state == 'stopped', 30)
    wait_for(v, laairs, f'climb to {args.alt:g} m', lambda: v.alt() > args.alt - 1.5, 90)
    v.pump(args.hover)
    v.m.set_mode('RTL')
    wait_for(v, laairs, 'RTL lands and disarms', lambda: not v.armed(), 300)
    if args.rtk_base:
        fixes = v.flight_fixes
        check(fixes and all(f == 6 for f in fixes),
              f'RTK FIXED for all {len(fixes)} GPS reports in flight'
              f'{"" if fixes and all(f == 6 for f in fixes) else " (got " + str(sorted(set(fixes))) + ")"}')
        check(laairs.rtk_streaming(), f'corrections still streaming ({laairs.rtk.get_status().rtcm_messages_sent} '
              'RTCM messages injected)')

    print('\n4. landed: charge after the post-flight checklist', flush=True)
    v.pump(8)
    check(st().state == 'stopped', 'pad stays parked until the post-flight checklist is acknowledged')
    laairs.bus.emit(laairs.events.POSTFLIGHT_CHECKLIST_ACKNOWLEDGED, {})
    wait_for(v, laairs, 'pad detects the drone back on the tiles', lambda: st().is_on_pad, 60)
    wait_for(v, laairs, 'charging', lambda: st().is_charging and st().current > 1.0, 90)
    wait_for(v, laairs, 'charge ends', lambda: st().state == 'charged', args.charge_timeout, every=20)
    s = st()
    check(s.stop_reason == 'REACHED_CURRENT_CUTOFF' and s.is_charged,
          f'pad reports full by current cutoff ({s.stop_reason}, {s.energy_wh:.1f} Wh in '
          f'{s.charge_time_secs / 60:.1f} min)')
    check(s.cycle_state == 'complete_reported', f'LAAIRS charge cycle completes ({s.cycle_state})')
    check(not s.arming_locked, 'arming allowed again')
    mgr.set_enabled(False)
    if laairs.rtk is not None:
        laairs.rtk.stop()
    print('\nPASS: SITL + Gazebo + Skycharge twin' + (' + RTK base twin' if args.rtk_base else '') +
          ' + LAAIRS charged the CopterSonde through a flight cycle', flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--laairs', required=True, help="BLISS-GCS src folder (LAAIRS's laairs package)")
    ap.add_argument('--master', default='tcp:127.0.0.1:5762', help='SITL MAVLink (default: instance 0 SERIAL1)')
    ap.add_argument('--pad', default='127.0.0.1:2222', help='the twin (or a pad) as host:port')
    ap.add_argument('--rtk-base', default='socket://127.0.0.1:3101',
                    help="the RTK base's serial port or URL (default: the twin, %(default)s); '' flies without RTK")
    ap.add_argument('--capacity', type=int, default=1000, help='pack mAh; match CS_BATTERY_AH (default 1000)')
    ap.add_argument('--alt', type=float, default=20.0, help='take-off altitude, m')
    ap.add_argument('--hover', type=float, default=40.0, help='hover time, s')
    ap.add_argument('--charge-timeout', type=float, default=1200.0, help='s to wait for the final charge')
    args = ap.parse_args()
    args.pad_host, _, port = args.pad.rpartition(':')
    args.pad_port = int(port)
    try:
        run(args)
    except Fail as e:
        print(f'\nFAIL: {e}', flush=True)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
