# AP_FLAKE8_CLEAN
"""Tests for the Skycharge digital twin (CopterSonde-sim/skycharge)."""

import json
import os
import socket
import sys
import threading
import time
import urllib.error
import urllib.request

import pytest

SKY = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'skycharge')
if not os.path.isdir(SKY):
    SKY = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, SKY)

from skycharge_twin import cli, conf, params  # noqa: E402
from skycharge_twin.device import ChargeSource, Reason, State  # noqa: E402
from skycharge_twin.plant import StandalonePlant, charge_current, PsuCommand  # noqa: E402
from skycharge_twin.twin import Twin  # noqa: E402

CONF = os.path.join(SKY, 'skycharge.conf')
VENDOR_KEY = os.environ.get('SKYCHARGE_VENDOR_KEY', '')  # optional: test with the real vendor key


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def make(soc=0.5, capacity=9.0, on_pad=False, load=0.3, **sink):
    clock = Clock()
    plant = StandalonePlant(soc=soc, capacity_ah=capacity, contact=on_pad, load=load)
    c = conf.load(CONF)
    c.sink.update(sink)
    dev = ChargeSource(c, plant, clock=clock, seed=1)
    return dev, plant, clock


def run(dev, plant, clock, secs, dt=0.05, until=None):
    end = clock.t + secs
    while clock.t < end:
        clock.t += dt
        plant.step(dt)
        dev.step()
        if until is not None and until():
            return True
    return False


def cli_run(dev, line):
    out, err = [], []
    rc = cli.run_command_line(dev, line, out.append, err.append)
    return rc, ''.join(out), ''.join(err)


# ---------------------------------------------------------------- parameters

def test_dev_params_print_like_the_documentation():
    dev, _, _ = make()
    rc, out, _ = cli_run(dev, 'skycharge-cli show-dev-params')
    assert rc == 0
    lines = out.splitlines()
    assert lines[0] == 'Source has the following parameters:'
    assert lines[1] == '\tpsu-type                           RSP-750-48 (0x01)'
    assert lines[2] == '\tdetect-mode                        CAPACITY (0x02)'
    assert '\tsense-current-calib-point1-ma       1000:1000' in lines
    assert '\tsense-voltage-calib-point1-mv      15000:15000' in lines


def test_sink_params_json_is_strings():
    dev, _, _ = make()
    rc, out, _ = cli_run(dev, 'skycharge-cli show-sink-params --json')
    assert rc == 0
    data = json.loads(out)
    assert list(data)[:3] == ['capabilities', 'batt-type', 'batt-capacity-mah']
    assert data['batt-type'] == 'Li-Po'
    assert data['batt-max-voltage-mv'] == '25200'
    assert data['user-data1'] == '0x00000000'
    assert data['user-uart-config'] == '0-8-N-1'


def test_param_parsing_follows_libskycharge():
    p = params.find_param(params.SINK_PARAMS, 'BATT_TYPE')
    assert p.name == 'batt-type'
    assert params.parse_value(p, 'lion') == params.BATT_LION
    with pytest.raises(params.ParamError):
        params.parse_value(p, 'Li-HV')
    q = params.find_param(params.SINK_PARAMS, 'batt-capacity-mah')
    assert params.parse_value(q, '0x10') == 16
    assert params.parse_value(q, '9000mAh') == 9000
    u = params.find_param(params.SINK_PARAMS, 'user-uart-config')
    assert params.format_value(u, params.parse_value(u, '115200-8-E-2'), False) == '115200-8-E-2'


def test_set_sink_params_batch_and_errors():
    dev, _, _ = make()
    rc, _, err = cli_run(dev, 'skycharge-cli set-sink-params batt-type:Li-Ion,batt-capacity-mah:9000,'
                         'batt-max-voltage-mv:25000')
    assert rc == 0, err
    s = dev.get_sink_params()
    assert (s['batt-type'], s['batt-capacity-mah'], s['batt-max-voltage-mv']) == (params.BATT_LION, 9000, 25000)
    # a bad value rejects the whole batch, before anything is written
    rc, _, err = cli_run(dev, 'skycharge-cli set-sink-params batt-capacity-mah:1000,batt-type:Li-HV')
    assert rc == 255 and "Can't parse value 'Li-HV' for param batt-type" in err
    assert dev.get_sink_params()['batt-capacity-mah'] == 9000
    rc, _, err = cli_run(dev, 'skycharge-cli set-sink-param batt-chemistry Li-Po')
    assert rc == 255 and err.startswith('Incorrect parameter: batt-chemistry\nPlease, use one of the following:')
    rc, _, err = cli_run(dev, 'skycharge-cli set-sink-params batt-type')
    assert rc == 255 and 'is missing' in err


def test_usage_and_device_id():
    dev, _, _ = make()
    rc, _, err = cli_run(dev, 'skycharge-cli show-charging-state --pretty')
    assert rc == 255 and err.startswith('Skycharge charging pad command line tool.')
    rc, out, _ = cli_run(dev, 'skycharge-cli list-devs')
    assert rc == 0 and dev.conf.dev_id in out and 'bolognini' in out
    assert cli_run(dev, f'skycharge-cli dev-info --id {dev.conf.dev_id}')[0] == 0
    rc, _, err = cli_run(dev, 'skycharge-cli dev-info --id DEADBEEF')
    assert rc == 255 and 'was no found' in err
    assert cli_run(dev, 'skycharge-cli open-droneport')[0] == 255


# ---------------------------------------------------------------- the charging cycle

def test_full_cycle_land_charge_cutoff_takeoff():
    dev, plant, clock = make(soc=0.95, capacity=1.0, **{'batt-max-voltage-mv': 25000,
                                                        'charging-max-current-ma': 6000,
                                                        'precharge-secs': 10})
    run(dev, plant, clock, 2)
    assert dev.state == State.SCANNING
    rc, out, _ = cli_run(dev, 'skycharge-cli show-charging-state')
    assert 'Dev state:       SCANNING' in out and 'Voltage:         12.0' in out

    plant.place(True)
    run(dev, plant, clock, 2.9)
    assert dev.state == State.SCANNING            # not before the detection delay
    assert run(dev, plant, clock, 1, until=lambda: dev.state == State.LINK_ESTABLISHED)
    assert run(dev, plant, clock, 1, until=lambda: dev.state == State.PRECHARGE_DELAYED)
    assert run(dev, plant, clock, 2, until=lambda: dev.state == State.PRECHARGING)
    run(dev, plant, clock, 5)
    st = dev.charging_state()
    assert 1000 < st['current_mA'] < 6000          # ramping
    assert run(dev, plant, clock, 10, until=lambda: dev.state == State.CHARGING)
    assert run(dev, plant, clock, 3600, until=lambda: dev.state == State.CHARGING_FINISHED)
    assert dev.reason == Reason.REACHED_CURRENT_CUTOFF
    st = dev.charging_state()
    assert st['energy_mWh'] > 0 and st['charge_mAh'] > 0 and st['state_of_charge'] == 100
    rc, out, _ = cli_run(dev, 'skycharge-cli show-charging-state --json')
    data = json.loads(out)
    assert data['state'] == 'CHARGING_FINISHED' and data['reason'] == 'REACHED_CURRENT_CUTOFF'
    assert data['soc'] == '100% ~ 00h:00m'
    rc, out, _ = cli_run(dev, 'skycharge-cli show-charging-state --no-pretty')
    assert 'CHARGING_FINISHED (REACHED_CURRENT_CUTOFF)' in out

    plant.place(False)                             # take-off
    run(dev, plant, clock, 0.2)
    assert dev.state == State.ERR_BAD_LINK
    assert run(dev, plant, clock, 6, until=lambda: dev.state == State.SCANNING)
    assert dev.charging_state()['energy_mWh'] == 0


def test_drone_load_above_cutoff_never_reaches_cutoff():
    """The source meters the pack's charge plus the drone's own draw: above the cutoff it floats."""
    dev, plant, clock = make(soc=0.95, capacity=1.0, load=0.8,
                             **{'batt-max-voltage-mv': 25000, 'total-charge-secs': 1800})
    plant.place(True)
    assert run(dev, plant, clock, 3600, until=lambda: dev.state == State.CHARGING_FINISHED)
    assert dev.reason == Reason.CHARGING_TIME_ELAPSED


def test_operator_commands_follow_the_documented_states():
    dev, plant, clock = make(soc=0.3, on_pad=True)
    assert run(dev, plant, clock, 10, until=lambda: dev.state == State.PRECHARGING)
    dev.charge_start()                             # only in CHARGING_FINISHED: ignored
    assert dev.state == State.PRECHARGING
    assert cli_run(dev, 'skycharge-cli sink-stop-charge')[0] == 0
    assert (dev.state, dev.reason) == (State.CHARGING_FINISHED, Reason.CHARGING_FORCIBLY_STOPPED)
    run(dev, plant, clock, 1)
    assert dev.charging_state()['current_mA'] == 0
    assert cli_run(dev, 'skycharge-cli sink-start-charge')[0] == 0
    assert dev.state == State.PRECHARGING
    cli_run(dev, 'skycharge-cli stop-scan')
    assert dev.state == State.STOPPED
    run(dev, plant, clock, 5)
    assert dev.state == State.STOPPED and dev.charging_state()['voltage_mV'] == 0
    cli_run(dev, 'skycharge-cli resume-scan')
    assert dev.state == State.SCANNING
    cli_run(dev, 'skycharge-cli sink-stop-charge')   # not charging: ignored
    assert dev.state == State.SCANNING


def test_repeat_charge_after_minutes():
    dev, plant, clock = make(soc=0.3, on_pad=True)
    assert run(dev, plant, clock, 10, until=lambda: dev.state == State.PRECHARGING)
    dev.charge_stop()
    run(dev, plant, clock, 9 * 60)
    assert dev.state == State.CHARGING_FINISHED
    assert run(dev, plant, clock, 70, until=lambda: dev.state in (State.PRECHARGING, State.CHARGING))


def test_time_limit_low_battery_and_bad_settings():
    dev, plant, clock = make(soc=0.2, on_pad=True, **{'total-charge-secs': 60})
    assert run(dev, plant, clock, 120, until=lambda: dev.state == State.CHARGING_FINISHED)
    assert dev.reason == Reason.CHARGING_TIME_ELAPSED

    dev, plant, clock = make(soc=0.0, on_pad=True, **{'batt-min-voltage-mv': 22000})
    dev.set_dev_params({'ignore-low-batt-voltage': 0})
    assert run(dev, plant, clock, 20, until=lambda: dev.state == State.CHARGING_FINISHED)
    assert dev.reason == Reason.LOW_BATT_VOLTAGE

    dev, plant, clock = make(soc=0.5, on_pad=True, **{'batt-max-voltage-mv': 70000})
    assert run(dev, plant, clock, 10, until=lambda: dev.state == State.CHARGING_FINISHED)
    assert dev.reason == Reason.INVAL_CHARGING_SETTINGS


def test_power_cycle_reloads_the_conf_file():
    dev, plant, clock = make()
    cli_run(dev, 'skycharge-cli set-sink-param batt-capacity-mah 9000')
    dev.power_cycle(30)
    rc, _, err = cli_run(dev, 'skycharge-cli show-sink-params')
    assert rc == 255 and "Unable to connect to 'localhost:5555'" in err
    run(dev, plant, clock, 31)
    assert not dev.booting
    assert dev.get_sink_params()['batt-capacity-mah'] == 5000


def test_faults():
    dev, plant, clock = make()
    dev.faults.add('short-circuit')
    run(dev, plant, clock, 0.2)
    assert dev.state == State.ERR_SHORT_CIRCUIT
    dev.faults.clear()
    assert run(dev, plant, clock, 6, until=lambda: dev.state == State.SCANNING)
    plant.soc = 0.3
    plant.place(True)
    assert run(dev, plant, clock, 45, until=lambda: dev.state == State.CHARGING)
    dev.faults.add('mux-overtemp')
    assert run(dev, plant, clock, 120, until=lambda: dev.state == State.CHARGING_FINISHED)
    assert dev.reason == Reason.CRITICAL_TEMP_ON_MUX


def test_charge_current_model():
    cmd = PsuCommand('charge', 25.0, 10.0)
    assert charge_current(cmd, 22.0, 0.03, 0.08, 0.8) == 10.0          # CC
    assert 0 < charge_current(cmd, 24.5, 0.03, 0.08, 0.8) < 10.0       # CV taper
    assert charge_current(cmd, 25.3, 0.03, 0.08, 0.0) == 0.0           # no reverse current
    assert charge_current(PsuCommand('scan', 12.0, 0.0), 22.0, 0.03, 0.08, 0.8) == 0.0


# ---------------------------------------------------------------- network fronts

def free_port():
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture
def served_twin(tmp_path):
    paramiko = pytest.importorskip('paramiko')
    from skycharge_twin.http_api import HttpApi
    from skycharge_twin.ssh_server import SshServer
    if os.path.exists(VENDOR_KEY):
        key_path = VENDOR_KEY
    else:
        key = paramiko.RSAKey.generate(2048)
        key_path = str(tmp_path / 'id_rsa')
        key.write_private_key_file(key_path)
    plant = StandalonePlant(soc=0.4, contact=False)
    twin = Twin(CONF, plant, log=lambda *_: None)
    ssh_port, http_port = free_port(), free_port()
    twin.ssh = SshServer(twin, '127.0.0.1', ssh_port, [key_path], str(tmp_path / 'host_key'), log=lambda *_: None)
    twin.http = HttpApi(twin, '127.0.0.1', http_port, log=lambda *_: None)
    twin.start()
    yield twin, ssh_port, http_port, key_path
    twin.stop()


def ssh_exec(port, key_path, command, **kw):
    import paramiko
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect('127.0.0.1', port=port, username='root', key_filename=key_path, allow_agent=False,
              look_for_keys=False, timeout=5, **kw)
    try:
        _, stdout, stderr = c.exec_command(command, timeout=10)
        rc = stdout.channel.recv_exit_status()
        return rc, stdout.read().decode(), stderr.read().decode()
    finally:
        c.close()


def test_ssh_like_laairs(served_twin):
    twin, port, _, key = served_twin
    rc, out, _ = ssh_exec(port, key, 'skycharge-cli show-charging-state --json')
    assert rc == 0 and json.loads(out)['state'] == 'SCANNING'
    rc, out, _ = ssh_exec(port, key, 'skycharge-cli dev-info')
    assert rc == 0 and out.startswith('Source info:\n   FW:        v1.2.2')
    rc, _, err = ssh_exec(port, key, 'skycharge-cli set-sink-params batt-type:LiFePO4')
    assert rc == 255 and "Can't parse value" in err
    rc, _, _ = ssh_exec(port, key, 'skytwin place && skytwin battery 0.3')
    # the twin loop keeps drawing the drone's load, so the state of charge may already be a tick lower
    assert rc == 0 and twin.plant.contact and abs(twin.plant.soc - 0.3) < 1e-3
    deadline = time.time() + 15
    while time.time() < deadline and twin.device.state != State.PRECHARGING:
        time.sleep(0.1)
    assert twin.device.state == State.PRECHARGING
    rc, out, _ = ssh_exec(port, key, 'nosuchcommand')
    assert rc == 127


def test_ssh_refuses_passwords_and_sha2_only_clients(served_twin):
    import paramiko
    _, port, _, key = served_twin
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    with pytest.raises(paramiko.AuthenticationException):
        c.connect('127.0.0.1', port=port, username='root', password='root', allow_agent=False, look_for_keys=False,
                  timeout=5)
    # a client that no longer signs with SHA-1 (paramiko 5) cannot log in, as on the real pad
    with pytest.raises(paramiko.SSHException):
        ssh_exec(port, key, 'true', disabled_algorithms={'pubkeys': ['ssh-rsa']})


def test_monitor_streams_until_closed(served_twin):
    import paramiko
    _, port, _, key = served_twin
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect('127.0.0.1', port=port, username='root', key_filename=key, allow_agent=False, look_for_keys=False)
    stdin, stdout, _ = c.exec_command('skycharge-cli monitor --json')
    stdin.close()                     # EOF on stdin must not stop the stream, as with sshd
    first = json.loads(''.join(stdout.readline() for _ in range(15)))
    second = json.loads(''.join(stdout.readline() for _ in range(15)))
    assert first['state'] == second['state'] == 'SCANNING'
    assert second['timestamp'] > first['timestamp']
    c.close()


def test_http_api(served_twin):
    twin, _, port, _ = served_twin
    c = twin.device.conf
    base = f'http://127.0.0.1:{port}/v1'
    q = f'user-uuid={c.user_uuid}&device-uuid={c.device_uuid}'
    data = json.load(urllib.request.urlopen(f'{base}/charging-state?{q}'))
    assert data['errno'] == 'SUCCESS' and data['charging-state']['state'] == 'SCANNING'
    assert data['charging-state']['voltage-mV'] > 11000
    req = urllib.request.Request(f'{base}/sink-charging-params?{q}&batt-capacity-mah=9000', method='PUT')
    assert json.load(urllib.request.urlopen(req))['errno'] == 'SUCCESS'
    assert twin.device.get_sink_params()['batt-capacity-mah'] == 9000
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(f'{base}/charging-state?user-uuid=11111111-0000-0000-0000-000000000000')
    assert e.value.code == 403
    assert json.load(urllib.request.urlopen(f'{base}/droneport-open?{q}'))['errno'] == 'EOPNOTSUPP'


def test_power_cycle_drops_ssh(served_twin):
    import paramiko
    twin, port, _, key = served_twin
    rc, _, _ = ssh_exec(port, key, 'skytwin power-cycle 2')
    time.sleep(0.6)
    with pytest.raises((paramiko.SSHException, OSError, EOFError)):
        ssh_exec(port, key, 'true')
    deadline = time.time() + 6
    while time.time() < deadline:
        try:
            if ssh_exec(port, key, 'true')[0] == 0:
                break
        except (paramiko.SSHException, OSError, EOFError):
            time.sleep(0.3)
    assert ssh_exec(port, key, 'true')[0] == 0


if __name__ == '__main__':
    threading.excepthook = lambda a: None
    sys.exit(pytest.main([__file__, '-v']))
