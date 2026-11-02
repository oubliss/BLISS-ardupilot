# AP_FLAKE8_CLEAN
"""
The charge source: the MUX HW2 board's state machine and meters, plus skycharged around it.

States, reasons and command semantics are the ones documented on support.skycharge.de
(system-states, tool, source/sink-board-configuration) and in libskycharge:

  SCANNING ---contact held DETECT_SECS---> LINK_ESTABLISHED --> [PRECHARGE_DELAYED] -->
  PRECHARGING (current ramps from precharge-current to charging-max-current over precharge-secs)
  --> CHARGING --> CHARGING_FINISHED (reason REACHED_CURRENT_CUTOFF, CHARGING_TIME_ELAPSED,
  CHARGING_FORCIBLY_STOPPED, CRITICAL_TEMP_ON_MUX/SINK, INVAL_CHARGING_SETTINGS, LOW_BATT_VOLTAGE)

- Losing contact in any on-pad state gives ERR_BAD_LINK, the normal end of a session; after
  error-indication-timeout-secs (default 5 s) the source scans again.
- CHARGING_FINISHED keeps watching the sink and restarts charging after repeat-charge-after-mins.
- stop-scan suspends everything (STOPPED) until resume-scan.
- sink-stop-charge acts only in PRECHARGE_DELAYED, PRECHARGING and CHARGING; sink-start-charge only
  in CHARGING_FINISHED. Both do nothing in other states, and still succeed.

Where the documentation is silent the twin makes a choice, marked "twin:" below.
"""

import enum
import random
import threading
import time
from collections import deque

from . import conf as conf_mod
from . import params
from .plant import PsuCommand


class State(enum.IntEnum):
    STOPPED = 0x00
    SCANNING = 0x01
    LINK_ESTABLISHED = 0x02
    PRECHARGE_DELAYED = 0x03
    PRECHARGING = 0x04
    CHARGING = 0x05
    CHARGING_FINISHED = 0x06
    ERR_BAD_LINK = 0x80
    ERR_LOW_MUX_VOLTAGE = 0x81
    ERR_VOLTAGE_ON_OUTPUT = 0x82
    ERR_SHORT_CIRCUIT = 0x83


class Reason(enum.IntEnum):
    UNKNOWN = 0x00
    REACHED_CURRENT_CUTOFF = 0x01
    CHARGING_TIME_ELAPSED = 0x02
    CHARGING_FORCIBLY_STOPPED = 0x03
    CRITICAL_TEMP_ON_MUX = 0x04
    CRITICAL_TEMP_ON_SINK = 0x05
    INVAL_CHARGING_SETTINGS = 0x06
    LOW_BATT_VOLTAGE = 0x07


ON_PAD = (State.LINK_ESTABLISHED, State.PRECHARGE_DELAYED, State.PRECHARGING, State.CHARGING,
          State.CHARGING_FINISHED)
CHARGE = (State.PRECHARGING, State.CHARGING)
STOPPABLE = (State.PRECHARGE_DELAYED, State.PRECHARGING, State.CHARGING)

# Faults the operator of the twin can inject (skytwin fault ...)
FAULTS = {
    'short-circuit': 'a conductive object shorts the tiles: ERR_SHORT_CIRCUIT while scanning',
    'low-mux-voltage': 'mains/PSU input sags: ERR_LOW_MUX_VOLTAGE until cleared',
    'voltage-on-output': 'external voltage on the tiles (only if ignore-voltage-on-output is false)',
    'mux-overtemp': 'the source overheats: charging ends with CRITICAL_TEMP_ON_MUX',
    'contact-bounce': 'the contact unit chatters: contact drops for 0.3 s every 4 s',
}


class ChargeSource:
    """MUX HW2 + skycharged. Thread-safe: every public method takes the lock."""

    FW_VERSION = (1, 2, 2)
    HW_VERSION = (1, 1, 0)
    PLC_PROTO_VERSION = (1, 0, 0)

    DETECT_SECS = 3.0            # twin: "a few seconds" of steady contact before the sink counts
    LINK_SECS = 0.5              # twin: LINK_ESTABLISHED dwell (intermediate in capacity mode)
    LOW_BATT_CHECK_SECS = 1.0    # twin: battery-voltage check this long after current starts
    CUTOFF_GRACE_SECS = 2.0      # twin: no cutoff decision before the current has settled
    SCAN_VOLTAGE = 12.0          # factory scanning voltage (system-operation page)
    MUX_CRITICAL_C = 80.0        # twin: source temperature that ends a charge
    SINK_CRITICAL_C = 70.0       # twin: sink temperature that ends a charge (PLC mode only)
    BOOT_SECS = 45.0             # twin: power-on to skycharged answering

    def __init__(self, conf=None, plant=None, clock=time.monotonic, seed=None):
        self.conf = conf or conf_mod.Conf()
        self.plant = plant
        self.clock = clock
        self.rng = random.Random(seed)
        self.lock = threading.RLock()
        self.faults = set()
        self.events = deque(maxlen=500)
        self._event_seq = 0
        self.booting_until = None
        self.boot_count = 0
        self.daemon_running = True
        self._load_conf()
        self._reset_runtime(self.clock())
        self.mux_temp = self._ambient().ambient_temperature + 6.0
        self.sink_temp = self._ambient().ambient_temperature

    # ------------------------------------------------------------------ setup

    def _load_conf(self):
        """skycharged start: the conf file overwrites the board parameters."""
        self.dev = dict(self.conf.dev)
        self.sink = dict(self.conf.sink)

    def _reset_runtime(self, now):
        self.state = State.SCANNING
        self.reason = Reason.UNKNOWN
        self.since = now
        self.last_step = now
        self.contact_since = None
        self.below_cutoff_since = None
        self.low_batt_checked = False
        self.charge_started = None
        self.energy_mwh = 0.0
        self.charge_mah = 0.0
        self.charging_secs = 0.0
        self.soc_perc = 0
        self.until_full_secs = 0
        self.voltage_mv = int(self.SCAN_VOLTAGE * 1000)
        self.current_ma = 0
        self.measurement = None
        self.command = PsuCommand('scan', self.SCAN_VOLTAGE, 0.0)

    def _ambient(self):
        if self.plant is None:
            from .plant import Measurement
            return Measurement()
        return self.plant.measure()

    def log(self, text):
        self._event_seq += 1
        self.events.append((self._event_seq, time.time(), text))

    def events_since(self, seq):
        with self.lock:
            return [e for e in self.events if e[0] > seq]

    # ------------------------------------------------------------------ helpers

    def _enter(self, state, now, reason=None):
        if state == self.state and reason is None:
            return
        prev = self.state
        self.state = state
        self.since = now
        if reason is not None:
            self.reason = reason
        elif state != State.CHARGING_FINISHED:
            self.reason = Reason.UNKNOWN
        text = f'{prev.name} -> {state.name}'
        if state == State.CHARGING_FINISHED:
            text += f' ({self.reason.name})'
        self.log(text)
        if state in (State.SCANNING, State.LINK_ESTABLISHED):
            self._new_session()
        if state in CHARGE and prev not in CHARGE:
            self.charge_started = now
            self.below_cutoff_since = None
            self.low_batt_checked = False
            if not self.dev['keep-silence']:
                self.log('beep')

    def _new_session(self):
        self.energy_mwh = 0.0
        self.charge_mah = 0.0
        self.charging_secs = 0.0
        self.soc_perc = 0
        self.until_full_secs = 0

    def _finish(self, reason, now):
        self._enter(State.CHARGING_FINISHED, now, reason)

    def _start_current(self, now):
        if self.sink['precharge-secs'] > 0:
            self._enter(State.PRECHARGING, now)
        else:
            self._enter(State.CHARGING, now)

    def _begin_charge(self, now):
        if self.sink['precharge-delay-secs'] > 0:
            self._enter(State.PRECHARGE_DELAYED, now)
        else:
            self._start_current(now)

    def psu_limits(self):
        vmin, vmax, imax = params.PSU_LIMITS.get(self.dev['psu-type'], (11.0, 57.0, 15.0))
        return vmin, vmax, imax

    def setting_problems(self):
        """Sink settings the source refuses (INVAL_CHARGING_SETTINGS). twin: plausibility rules."""
        s = self.sink
        vmin, vmax, imax = self.psu_limits()
        problems = []
        if not vmin * 1000 <= s['batt-max-voltage-mv'] <= vmax * 1000:
            problems.append(f"batt-max-voltage-mv {s['batt-max-voltage-mv']} outside the PSU's "
                            f'{vmin:g}-{vmax:g} V')
        if s['batt-min-voltage-mv'] >= s['batt-max-voltage-mv']:
            problems.append('batt-min-voltage-mv not below batt-max-voltage-mv')
        if s['charging-max-current-ma'] < 100:
            problems.append('charging-max-current-ma below 100 mA')
        if s['cutoff-min-current-ma'] >= s['charging-max-current-ma']:
            problems.append('cutoff-min-current-ma not below charging-max-current-ma')
        if s['batt-capacity-mah'] == 0:
            problems.append('batt-capacity-mah is 0')
        return problems

    def _current_limit(self, now):
        """CC limit for the present state: precharge ramp, then the sink maximum, capped by the PSU."""
        s = self.sink
        if self.dev['psu-use-fixed-v-i']:
            return self.dev['psu-fixed-current-ma'] / 1000.0
        i_max = s['charging-max-current-ma'] / 1000.0
        if self.state == State.PRECHARGING and s['precharge-secs'] > 0:
            f = min(max((now - self.since) / s['precharge-secs'], 0.0), 1.0)
            i_pre = s['precharge-current-ma'] / 1000.0
            i_max = i_pre + (i_max - i_pre) * f
        return min(i_max, self.psu_limits()[2])

    def _voltage_setpoint(self):
        if self.dev['psu-use-fixed-v-i']:
            v = self.dev['psu-fixed-voltage-mv'] / 1000.0
        else:
            v = self.sink['batt-max-voltage-mv'] / 1000.0
        vmin, vmax, _ = self.psu_limits()
        return min(max(v, vmin), vmax)

    def _psu_command(self, now):
        if self.booting_until is not None or not self.daemon_running:
            return PsuCommand()
        if self.state in CHARGE:
            return PsuCommand('charge', self._voltage_setpoint(), self._current_limit(now))
        if self.state in (State.SCANNING, State.LINK_ESTABLISHED, State.PRECHARGE_DELAYED,
                          State.CHARGING_FINISHED):
            return PsuCommand('scan', self.SCAN_VOLTAGE, 0.0)
        return PsuCommand()

    def _contact(self, m, now):
        if not (m.valid and m.contact):
            return False
        if 'contact-bounce' in self.faults and (now % 4.0) < 0.3:
            return False
        return True

    # ------------------------------------------------------------------ the loop

    def step(self):
        """Advance the board by the time since the last step (call at 10-50 Hz)."""
        with self.lock:
            now = self.clock()
            dt = min(max(now - self.last_step, 0.0), 1.0)
            self.last_step = now
            if self.booting_until is not None:
                if now < self.booting_until:
                    self._publish(PsuCommand())
                    return
                self._finish_boot(now)
            m = self.plant.measure() if self.plant else None
            if m is None:
                from .plant import Measurement
                m = Measurement()
            self.measurement = m
            self._update_meters(m, now, dt)
            self._update_thermal(m, dt)
            self._run(m, now)
            self._publish(self._psu_command(now))

    def _publish(self, command):
        self.command = command
        if self.plant:
            self.plant.command(command)

    def _update_meters(self, m, now, dt):
        noise_v = self.rng.gauss(0, 0.003)
        noise_i = self.rng.gauss(0, 0.005)
        charging = self.state in CHARGE and self.command.mode == 'charge'
        if charging and m.contact:
            i = m.current + noise_i
            if i * 1000 < self.dev['min-sense-current-ma']:
                i = 0.0
            v = m.voltage + noise_v
            self.energy_mwh += v * i * dt / 3.6
            self.charge_mah += i * dt / 3.6
            self.charging_secs += dt
        elif self.state in CHARGE:
            v, i = 0.0, 0.0
            self.charging_secs += dt
        elif self.command.mode == 'scan':
            v, i = self.SCAN_VOLTAGE + noise_v, 0.0
        else:
            v, i = 0.0, 0.0
        self.voltage_mv = max(int(round(v * 1000)), 0)
        self.current_ma = max(int(round(i * 1000)), 0)
        # twin: the capacity-mode SoC is the output voltage between the sink's min and max voltage,
        # and the time to full the remaining capacity at the present current
        s = self.sink
        if self.state in CHARGE and self.current_ma > 0:
            span = max(s['batt-max-voltage-mv'] - s['batt-min-voltage-mv'], 1)
            self.soc_perc = int(min(max((self.voltage_mv - s['batt-min-voltage-mv']) * 100 / span, 0), 100))
            left_mah = (100 - self.soc_perc) / 100.0 * s['batt-capacity-mah']
            self.until_full_secs = int(left_mah / self.current_ma * 3600)
        elif self.state == State.CHARGING_FINISHED and self.reason == Reason.REACHED_CURRENT_CUTOFF:
            self.soc_perc, self.until_full_secs = 100, 0
        elif self.state not in ON_PAD:
            self.soc_perc, self.until_full_secs = 0, 0

    def _update_thermal(self, m, dt):
        # twin: first-order thermal models. The source loses ~10 % of its output power and has a fan;
        # the sink heats with I^2 in its protection MOSFETs and the contact unit.
        power = self.voltage_mv * self.current_ma / 1e6
        target = m.ambient_temperature + 6.0 + 0.012 * power
        if 'mux-overtemp' in self.faults:
            target = self.MUX_CRITICAL_C + 10.0
            dt *= 20.0
        self.mux_temp += (target - self.mux_temp) * min(dt / 240.0, 1.0)
        i = self.current_ma / 1000.0
        sink_target = m.ambient_temperature + 0.06 * i * i
        self.sink_temp += (sink_target - self.sink_temp) * min(dt / 90.0, 1.0)

    def _run(self, m, now):
        s = self.state
        t = now - self.since
        contact = self._contact(m, now)
        if 'low-mux-voltage' in self.faults and s != State.ERR_LOW_MUX_VOLTAGE:
            self._enter(State.ERR_LOW_MUX_VOLTAGE, now)
            return
        if ('voltage-on-output' in self.faults and not self.dev['ignore-voltage-on-output']
                and s == State.SCANNING):
            self._enter(State.ERR_VOLTAGE_ON_OUTPUT, now)
            return
        if s == State.STOPPED:
            return
        if s == State.ERR_LOW_MUX_VOLTAGE:
            if 'low-mux-voltage' not in self.faults:
                self._enter(State.SCANNING, now)
            return
        if s == State.ERR_VOLTAGE_ON_OUTPUT:
            if 'voltage-on-output' not in self.faults or self.dev['ignore-voltage-on-output']:
                self._enter(State.SCANNING, now)
            return
        if s in (State.ERR_BAD_LINK, State.ERR_SHORT_CIRCUIT):
            if t >= self.dev['error-indication-timeout-secs']:
                self._enter(State.SCANNING, now)
            return
        if s == State.SCANNING:
            if 'short-circuit' in self.faults:
                self._enter(State.ERR_SHORT_CIRCUIT, now)
                return
            if not contact:
                self.contact_since = None
            elif self.contact_since is None:
                self.contact_since = now
            elif now - self.contact_since >= self.DETECT_SECS:
                self.contact_since = None
                self._enter(State.LINK_ESTABLISHED, now)
            return

        # a sink is (or was) on the pad
        if not contact:
            self._enter(State.ERR_BAD_LINK, now)
            return
        if s == State.LINK_ESTABLISHED:
            if t >= self.LINK_SECS:
                if self.setting_problems() and not self.dev['ignore-inval-charging-settings']:
                    for p in self.setting_problems():
                        self.log(f'invalid charging settings: {p}')
                    self._finish(Reason.INVAL_CHARGING_SETTINGS, now)
                else:
                    self._begin_charge(now)
        elif s == State.PRECHARGE_DELAYED:
            if t >= self.sink['precharge-delay-secs']:
                self._start_current(now)
        elif s in CHARGE:
            self._run_charge(m, now, t)
        elif s == State.CHARGING_FINISHED:
            repeat = self.dev['repeat-charge-after-mins']
            if repeat and t >= repeat * 60:
                self.log(f'repeat-charge-after-mins ({repeat}) elapsed: charging again')
                self._new_session()
                self._start_current(now)

    def _run_charge(self, m, now, t):
        s = self.sink
        run_t = now - (self.charge_started or now)
        if not self.low_batt_checked and run_t >= self.LOW_BATT_CHECK_SECS:
            self.low_batt_checked = True
            pack_mv = self.voltage_mv
            if pack_mv < s['batt-min-voltage-mv'] and not self.dev['ignore-low-batt-voltage']:
                self.log(f"battery at {pack_mv} mV is below batt-min-voltage-mv {s['batt-min-voltage-mv']}")
                self._finish(Reason.LOW_BATT_VOLTAGE, now)
                return
        if self.state == State.PRECHARGING and t >= s['precharge-secs']:
            self._enter(State.CHARGING, now)
        if self.mux_temp >= self.MUX_CRITICAL_C:
            self._finish(Reason.CRITICAL_TEMP_ON_MUX, now)
            return
        if self.dev['detect-mode'] == params.DETECT_PLC and self.sink_temp >= self.SINK_CRITICAL_C:
            self._finish(Reason.CRITICAL_TEMP_ON_SINK, now)
            return
        if self.charging_secs >= s['total-charge-secs']:
            self._finish(Reason.CHARGING_TIME_ELAPSED, now)
            return
        if run_t < self.CUTOFF_GRACE_SECS:
            return
        if self.current_ma < s['cutoff-min-current-ma']:
            if self.below_cutoff_since is None:
                self.below_cutoff_since = now
            elif (now - self.below_cutoff_since) * 1000 >= s['cutoff-timeout-ms']:
                self._finish(Reason.REACHED_CURRENT_CUTOFF, now)
        else:
            self.below_cutoff_since = None

    # ------------------------------------------------------------------ power

    def power_cycle(self, boot_secs=None):
        """Mains off and on: the BeagleBone reboots and skycharged reloads skycharge.conf."""
        with self.lock:
            now = self.clock()
            self.booting_until = now + (self.BOOT_SECS if boot_secs is None else boot_secs)
            self.boot_count += 1
            self.log('power cycle: charge source rebooting')
            self._publish(PsuCommand())

    def restart_daemon(self):
        """systemctl restart skycharged: board parameters come back from skycharge.conf."""
        with self.lock:
            self._load_conf()
            self._reset_runtime(self.clock())
            self.log('skycharged restarted: parameters reloaded from skycharge.conf')

    def _finish_boot(self, now):
        self.booting_until = None
        self._load_conf()
        self._reset_runtime(now)
        self.log('skycharged up: parameters loaded from skycharge.conf')

    @property
    def booting(self):
        with self.lock:
            return self.booting_until is not None

    # ------------------------------------------------------------------ commands (skycharge-cli, HTTP)

    def scan_stop(self):
        with self.lock:
            self._enter(State.STOPPED, self.clock())

    def scan_resume(self):
        with self.lock:
            if self.state == State.STOPPED:
                self._enter(State.SCANNING, self.clock())

    def charge_stop(self):
        with self.lock:
            if self.state in STOPPABLE:
                self._finish(Reason.CHARGING_FORCIBLY_STOPPED, self.clock())

    def charge_start(self):
        with self.lock:
            if self.state == State.CHARGING_FINISHED:
                self._new_session()
                self._start_current(self.clock())

    def reset(self):
        with self.lock:
            self.log('reset')
            self._reset_runtime(self.clock())

    def sink_linked(self):
        """In PLC mode the sink answers only with a data link, i.e. while it sits on the pad."""
        with self.lock:
            return self.state in ON_PAD

    def get_dev_params(self):
        with self.lock:
            return dict(self.dev)

    def get_sink_params(self):
        with self.lock:
            return dict(self.sink)

    def set_dev_params(self, values):
        with self.lock:
            self.dev.update(values)
            self.log('set-dev-params ' + ', '.join(f'{k}={v}' for k, v in values.items()))

    def set_sink_params(self, values):
        with self.lock:
            self.sink.update(values)
            self.log('set-sink-params ' + ', '.join(f'{k}={v}' for k, v in values.items()))

    def charging_state(self):
        """struct sky_charging_state, as a dict."""
        with self.lock:
            return {
                'state': self.state,
                'reason': self.reason,
                'voltage_mV': self.voltage_mv,
                'current_mA': self.current_ma,
                'energy_mWh': int(self.energy_mwh),
                'charge_mAh': int(self.charge_mah),
                'charging_secs': int(self.charging_secs),
                'until_full_secs': int(self.until_full_secs),
                'state_of_charge': int(self.soc_perc),
                'mux_temperature_C': int(round(self.mux_temp)),
                'mux_humidity_perc': int(round(self.measurement.ambient_humidity)) if self.measurement else 0,
                'sink_temperature_C': int(round(self.sink_temp)) if self.state in ON_PAD else 0,
                'link_quality_factor': 0,
            }

    def uid(self):
        h = (self.conf.device_uuid.replace('-', '') + '0' * 24)[:24]
        return h

    def version(self, which):
        return {'fw': self.FW_VERSION, 'hw': self.HW_VERSION, 'plc': self.PLC_PROTO_VERSION}[which]
