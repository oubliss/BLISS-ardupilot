# AP_FLAKE8_CLEAN
"""
skycharge-cli, as skyclient.c in libskycharge v2.2.2 implements it: the same commands, options,
output formats (pretty, --no-pretty, --json) and exit status (0, or 255 for exit(-1)).

Remote invocation (<addr> [<port>]) is accepted, but every command runs on this twin: it is the
device, and the broker is not emulated.
"""

import datetime
import errno
import os
import shlex
import time

from . import params
from .device import Reason, State

USAGE = "\n".join((
    'Skycharge charging pad command line tool.',
    '',
    'Usage:',
    '    skycharge-cli discover-broker               [<timeout-secs>]',
    '    skycharge-cli peer-info                      --conffile <conff> <addr> [<port>]',
    '    skycharge-cli list-devs                     [(--conffile <conff>)] [(<addr> [<port>])]',
    '    skycharge-cli monitor [(--pretty | --json)] [--link-stat] [(--id <dev-id>)] [(--conffile'
    ' <conff>)] [(<addr> [<port>])]',
    '    skycharge-cli show-dev-params [--json]      [(--id <dev-id>)] [(--conffile <conff>)] [(<addr> [<port>])]',
    '    skycharge-cli set-dev-param <key> <value>   [(--id <dev-id>)] [(--conffile <conff>)] [(<addr> [<port>])]',
    '    skycharge-cli set-dev-params <key-value-pairs-string> [(--id <dev-id>)] [(--conffile <conff>)]'
    ' [(<addr> [<port>])]',
    '    skycharge-cli resume-scan                   [(--id <dev-id>)] [(--conffile <conff>)] [(<addr> [<port>])]',
    '    skycharge-cli stop-scan                     [(--id <dev-id>)] [(--conffile <conff>)] [(<addr> [<port>])]',
    '    skycharge-cli open-droneport                [(--id <dev-id>)] [(--conffile <conff>)] [(<addr> [<port>])]',
    '    skycharge-cli close-droneport               [(--id <dev-id>)] [(--conffile <conff>)] [(<addr> [<port>])]',
    '    skycharge-cli show-droneport-state          [(--id <dev-id>)] [(--conffile <conff>)] [(<addr> [<port>])]',
    '    skycharge-cli drone-detect                  [(--id <dev-id>)] [(--conffile <conff>)] [(<addr> [<port>])]',
    '    skycharge-cli show-charging-state [(--no-pretty | --json)] [--link-stat] [(--id <dev-id>)]'
    ' [(--conffile <conff>)] [(<addr> [<port>])]',
    '    skycharge-cli reset                         [(--id <dev-id>)] [(--conffile <conff>)] [(<addr> [<port>])]',
    '    skycharge-cli dev-info [--json]             [(--id <dev-id>)] [(--conffile <conff>)] [(<addr> [<port>])]',
    '    skycharge-cli gps-info                      [(--id <dev-id>)] [(--conffile <conff>)] [(<addr> [<port>])]',
    '    skycharge-cli sink-info [--json]            [(--id <dev-id>)] [(--conffile <conff>)] [(<addr> [<port>])]',
    '    skycharge-cli show-sink-params [--json]     [(--id <dev-id>)] [(--conffile <conff>)] [(<addr> [<port>])]',
    '    skycharge-cli set-sink-param <key> <value>  [(--id <dev-id>)] [(--conffile <conff>)] [(<addr> [<port>])]',
    '    skycharge-cli set-sink-params <key-value-pairs-string> [(--id <dev-id>)] [(--conffile <conff>)]'
    ' [(<addr> [<port>])]',
    '    skycharge-cli sink-start-charge             [(--id <dev-id>)] [(--conffile <conff>)] [(<addr> [<port>])]',
    '    skycharge-cli sink-stop-charge              [(--id <dev-id>)] [(--conffile <conff>)] [(<addr> [<port>])]',
    '    skycharge-cli sink-flash-info               [(--id <dev-id>)] [(--conffile <conff>)]',
    '    skycharge-cli sink-firmware-update [<elf-file>] [--bootloader] [(--id <dev-id>)] [(--conffile <conff>)]',
    '    skycharge-cli sink-passthru-send            [(--id <dev-id>)] [(--conffile <conff>)] [(<addr> [<port>])]',
    '    skycharge-cli sink-passthru-recv            [(--id <dev-id>)] [(--conffile <conff>)] [(<addr> [<port>])]',
    '',
    'Options:',
    '    --id <dev-id> - short ID of the device, is used when \'list-devs\' command outputs many devices',
    '    --conffile    - path to the configuration file, default path is /etc/skycharge.conf',
    '    --pretty      - outputs in pretty human readable format.',
    '    --no-pretty   - the opposite of \'pretty\', when \'pretty\' is default.',
    '    --json        - outputs in json format.',
    '    --link-stat   - includes data link statistics in the output.',
    '    --bootloader  - update bootloader firmware, operation is considered as dangerous and can brick'
    ' your sink device if update procedure is being interrupted',
    '',
    'Description:',
    '    Skycharge console client communicates with local and remote devices.',
    '    Almost each command has non obligatory <addr> and <port> to execute',
    '    command remotely on a server.  If <addr> and <port> are not specified',
    '    local device will be used.',
    '',
))

# command -> (positional arguments, allowed flags)
COMMANDS = {
    'discover-broker': (['timeout?'], set()),
    'peer-info': ([], {'--conffile'}),
    'list-devs': ([], {'--conffile'}),
    'monitor': ([], {'--pretty', '--json', '--link-stat', '--id', '--conffile'}),
    'show-dev-params': ([], {'--json', '--id', '--conffile'}),
    'set-dev-param': (['key', 'value'], {'--id', '--conffile'}),
    'set-dev-params': (['pairs'], {'--id', '--conffile'}),
    'resume-scan': ([], {'--id', '--conffile'}),
    'stop-scan': ([], {'--id', '--conffile'}),
    'open-droneport': ([], {'--id', '--conffile'}),
    'close-droneport': ([], {'--id', '--conffile'}),
    'show-droneport-state': ([], {'--id', '--conffile'}),
    'drone-detect': ([], {'--id', '--conffile'}),
    'show-charging-state': ([], {'--no-pretty', '--json', '--link-stat', '--id', '--conffile'}),
    'reset': ([], {'--id', '--conffile'}),
    'dev-info': ([], {'--json', '--id', '--conffile'}),
    'gps-info': ([], {'--id', '--conffile'}),
    'sink-info': ([], {'--json', '--id', '--conffile'}),
    'show-sink-params': ([], {'--json', '--id', '--conffile'}),
    'set-sink-param': (['key', 'value'], {'--id', '--conffile'}),
    'set-sink-params': (['pairs'], {'--id', '--conffile'}),
    'sink-start-charge': ([], {'--id', '--conffile'}),
    'sink-stop-charge': ([], {'--id', '--conffile'}),
    'sink-flash-info': ([], {'--id', '--conffile'}),
    'sink-firmware-update': (['elf?'], {'--bootloader', '--id', '--conffile'}),
    'sink-passthru-send': ([], {'--id', '--conffile'}),
    'sink-passthru-recv': ([], {'--id', '--conffile'}),
}
NO_REMOTE = {'discover-broker', 'sink-flash-info', 'sink-firmware-update'}
TAKES_VALUE = {'--id', '--conffile'}


class Exit(Exception):
    def __init__(self, code):
        super().__init__(code)
        self.code = code


class Cli:
    """One skycharge-cli invocation. out/err are callables taking text."""

    def __init__(self, device, out, err, stop=None, sleep=time.sleep):
        self.dev = device
        self.out = out
        self.err = err
        self.stop = stop
        self.sleep = sleep

    def fail(self, where, text):
        # sky_err() prefixes __FILE__:func():__LINE__
        self.err(f'skyclient.c:main():{where}: {text}\n')
        raise Exit(255)

    def errno_text(self, code):
        return os.strerror(code)

    # ------------------------------------------------------------------ argv

    def parse(self, argv):
        if not argv or argv[0] not in COMMANDS:
            return None
        cmd = argv[0]
        positional_spec, allowed = COMMANDS[cmd]
        opts = {}
        positional = []
        i = 1
        while i < len(argv):
            a = argv[i]
            if a.startswith('--'):
                if a not in allowed:
                    return None
                if a in TAKES_VALUE:
                    if i + 1 >= len(argv):
                        return None
                    opts[a] = argv[i + 1]
                    i += 2
                    continue
                opts[a] = True
            else:
                positional.append(a)
            i += 1
        if '--pretty' in opts and '--json' in opts or '--no-pretty' in opts and '--json' in opts:
            return None
        required = [p for p in positional_spec if not p.endswith('?')]
        optional = [p for p in positional_spec if p.endswith('?')]
        if len(positional) < len(required):
            return None
        args = dict(zip(required, positional))
        rest = positional[len(required):]
        for name in optional:
            if rest:
                args[name[:-1]] = rest.pop(0)
        if cmd == 'peer-info':
            if '--conffile' not in opts or not rest:
                return None
        if rest and cmd in NO_REMOTE:
            return None
        if len(rest) > 2:
            return None
        args['addr'] = rest[0] if rest else None
        args['port'] = rest[1] if len(rest) > 1 else None
        return cmd, opts, args

    def run(self, argv):
        """Execute argv (without the program name). Returns the exit status."""
        try:
            return self._run(argv)
        except Exit as e:
            return e.code

    def _run(self, argv):
        parsed = self.parse(argv)
        if parsed is None:
            self.err(USAGE + '\n')
            return 255
        cmd, opts, args = parsed
        if args['port'] is not None and not args['port'].isdigit():
            self.err(f"Error: You've called a remote command invocation and specified the remote address "
                     f"'{args['addr']}' and port '{args['port']}', but port is an invalid number.\n"
                     "       If you attempted to call the 'monitor' command with the '1000 --raw' parameters, "
                     "be aware that these parameters don't exist any more.\n"
                     "       See help for details.\n")
            return 255
        if cmd == 'discover-broker':
            self.err('skyclient.c:main():889: sky_discoverbroker(): Connection timed out\n')
            return 255
        if cmd == 'peer-info':
            self.err(f"Unable to connect to '{args['addr']}:{args['port'] or 5555}'\n")
            return 255
        if cmd in ('set-dev-params', 'set-sink-params') and not self._pairs_syntax_ok(args['pairs']):
            return 255
        if not self.dev.daemon_running or self.dev.booting:
            addr = args['addr'] or 'localhost'
            self.err(f"Unable to connect to '{addr}:{args['port'] or 5555}'\n")
            return 255
        dev_id = self.dev.conf.dev_id
        if '--id' in opts and opts['--id'] != dev_id:
            self.err(f"skyclient.c:sky_prepare_dev():267: Error: device by id '{opts['--id']}' was no found\n")
            return 255
        handler = getattr(self, 'cmd_' + cmd.replace('-', '_'))
        handler(opts, args)
        return 0

    def _pairs_syntax_ok(self, text):
        for kv in [p for p in text.replace(';', ',').replace(' ', ',').split(',') if p]:
            sep = min([i for i in (kv.find('='), kv.find(':')) if i >= 0], default=-1)
            if sep < 0 or not kv[sep + 1:]:
                name = kv[:sep] if sep >= 0 else kv
                self.err(f"skyclient.c:parse_dev_or_sink_params():562: Value for a parameter '{name}' is missing\n")
                return False
        return True

    # ------------------------------------------------------------------ output helpers

    @staticmethod
    def _timestamp():
        now = datetime.datetime.now()
        return now.strftime('%Y-%m-%d %H:%M:%S') + f'.{now.microsecond // 1000:03d}'

    @staticmethod
    def _hms(secs):
        return secs // 3600, secs % 3600 // 60, secs % 60

    def _state_text(self, st):
        state = st['state']
        if state != State.CHARGING_FINISHED or st['reason'] == Reason.UNKNOWN:
            return state.name
        return f"CHARGING_FINISHED ({st['reason'].name})"

    def print_charging_state(self, st, json_out, pretty, link_stat):
        v = st['voltage_mV'] / 1000.0
        i = st['current_mA'] / 1000.0
        p = st['voltage_mV'] * st['current_mA'] / 1e6
        h, m, s = self._hms(st['charging_secs'])
        fh, fm, _ = self._hms(st['until_full_secs'])
        if json_out:
            lines = [
                '{',
                f'\t"timestamp": "{self._timestamp()}",',
                f"\t\"state\": \"{st['state'].name}\",",
                f"\t\"reason\": \"{st['reason'].name}\",",
                f'\t"voltage": "{v:.3f}",',
                f'\t"current": "{i:.3f}",',
                f'\t"power": "{p:.3f}",',
                f"\t\"energy\": \"{st['energy_mWh'] / 1000.0:.3f}\",",
                f"\t\"charge\": \"{st['charge_mAh'] / 1000.0:.3f}\",",
                f"\t\"source-humidity\": \"{st['mux_humidity_perc']}\",",
                f"\t\"source-temperature\": \"{st['mux_temperature_C']}\",",
                f"\t\"sink-temperature\": \"{st['sink_temperature_C']}\",",
                f'\t"charging": "{h:02d}h:{m:02d}m:{s:02d}s",',
                f"\t\"soc\":  \"{st['state_of_charge']}% ~ {fh:02d}h:{fm:02d}m\"{',' if link_stat else ''}",
            ]
            if link_stat:
                lines += ['\t"link-stat" : {',
                          f"\t\t\"link-quality\": \"{st['link_quality_factor']}\",",
                          '\t\t"tx-bytes": "0",', '\t\t"tx-packets": "0",', '\t\t"tx-err-bytes": "0",',
                          '\t\t"tx-err-packets": "0",', '\t\t"rx-bytes": "0",', '\t\t"rx-packets": "0",',
                          '\t\t"rx-err-bytes": "0",', '\t\t"rx-err-packets": "0"', '\t}']
            lines.append('}')
            self.out('\n'.join(lines) + '\n')
        elif not pretty:
            line = (f"{self._timestamp()}\t{self._state_text(st):<30}\t{v:.3f}V\t{i:.3f}A\t{p:.3f}W\t"
                    f"{st['energy_mWh'] / 1000.0:.3f}Wh\t{st['charge_mAh'] / 1000.0:.3f}Ah\t"
                    f"{st['charging_secs']}s\t{st['until_full_secs']}s\t")
            if link_stat:
                line += f"\t0\t0\t0\t0\t0\t0\t0\t0\t{st['link_quality_factor']}%"
            self.out(line + '\n')
        else:
            text = (f'Timestamp:       {self._timestamp()}\n'
                    f'Dev state:       {self._state_text(st)}\n'
                    f'Voltage:         {v:.3f}V\n'
                    f'Current:         {i:.3f}A\n'
                    f'Power:           {p:.3f}W\n'
                    f"Energy:          {st['energy_mWh'] / 1000.0:.3f}Wh\n"
                    f"Charge:          {st['charge_mAh'] / 1000.0:.3f}Ah\n"
                    f"Source humidity: {st['mux_humidity_perc']}%\n"
                    f"Source temp:     {st['mux_temperature_C']}C\n"
                    f"Sink temp:       {st['sink_temperature_C']}C\n"
                    f'Charging:        {h:02d}h:{m:02d}m:{s:02d}s\n')
            if link_stat:
                text += (f"Link quality:    {st['link_quality_factor']}%\n"
                         'TX stat:\n    bytes:       0\n    packets:     0\n    err bytes:   0\n    err packets: 0\n'
                         'RX stat:\n    bytes:       0\n    packets:     0\n    err bytes:   0\n    err packets: 0\n')
            self.out(text + '\n')

    def print_params(self, table, values, json_out, title):
        if json_out:
            lines = ['{']
            for n, p in enumerate(table):
                comma = ',' if n < len(table) - 1 else ''
                lines.append(f'\t"{p.name}" : "{params.format_value(p, values[p.name], False)}"{comma}')
            lines.append('}')
        else:
            lines = [f'{title} has the following parameters:']
            for p in table:
                lines.append(f'\t{p.name:<34} {params.format_value(p, values[p.name], True)}')
        self.out('\n'.join(lines) + '\n')

    def print_info(self, title, json_out, linked=True):
        fw = '.'.join(map(str, self.dev.version('fw')))
        hw = '.'.join(map(str, self.dev.version('hw')))
        plc = '.'.join(map(str, self.dev.version('plc')))
        uid = self.dev.uid() if title == 'Source' else self.dev.uid()[::-1]
        if json_out:
            self.out('{\n'
                     f'\t"fw-version": "{fw}",\n'
                     f'\t"hw-version": "{hw}",\n'
                     f'\t"plc-proto-version": "{plc}",\n'
                     f'\t"uid": "{uid}"\n'
                     '}\n')
        else:
            self.out(f'{title} info:\n'
                     f'   FW:        v{fw}\n'
                     f'   HW:        v{hw}\n'
                     f'   PLC proto: v{plc}\n'
                     f'   UID:       {uid}\n')

    # ------------------------------------------------------------------ commands

    def cmd_list_devs(self, opts, args):
        name = self.dev.conf.device_name
        width = max(len(name), len('DEV-NAME'))
        fw = 'v' + '.'.join(map(str, self.dev.version('fw')))
        hw = 'v' + '.'.join(map(str, self.dev.version('hw')))
        self.out('Found sky devices:\n'
                 f"\t  DEV-ID     TYPE  {'DEV-NAME':>{width}}      FW      HW\n"
                 f'\t{self.dev.conf.dev_id}  MUX-HW2  {name:>{width}}  {fw}  {hw}\n')

    def cmd_monitor(self, opts, args):
        while self.stop is None or not self.stop.is_set():
            st = self.dev.charging_state()
            # unlike show-charging-state, monitor is tab-separated unless --pretty is given
            self.print_charging_state(st, '--json' in opts, '--pretty' in opts, '--link-stat' in opts)
            self.sleep(1.0)

    def cmd_show_charging_state(self, opts, args):
        st = self.dev.charging_state()
        self.print_charging_state(st, '--json' in opts, '--no-pretty' not in opts, '--link-stat' in opts)

    def cmd_show_dev_params(self, opts, args):
        self.print_params(params.DEV_PARAMS, self.dev.get_dev_params(), '--json' in opts, 'Source')

    def cmd_show_sink_params(self, opts, args):
        if self.dev.get_dev_params()['detect-mode'] == params.DETECT_PLC and not self.dev.sink_linked():
            self.err(f'skyclient.c:main():1023: sky_sink_paramsget(): {os.strerror(errno.ENOTCONN)}\n')
            raise Exit(255)
        self.print_params(params.SINK_PARAMS, self.dev.get_sink_params(), '--json' in opts, 'Sink')

    def _parse_pairs(self, opts, args, table):
        if 'pairs' in args:
            text = args['pairs']
            items = []
            for kv in [p for p in text.replace(';', ',').replace(' ', ',').split(',') if p]:
                sep = min(i for i in (kv.find('='), kv.find(':')) if i >= 0)
                items.append((kv[:sep], kv[sep + 1:]))
        else:
            items = [(args['key'], args['value'])]
        values = {}
        for key, value in items:
            p = params.find_param(table, key)
            if p is None:
                self.err(f'Incorrect parameter: {key}\nPlease, use one of the following:\n'
                         + ''.join(f'\t{q.name}\n' for q in table))
                raise Exit(255)
            try:
                values[p.name] = params.parse_value(p, value)
            except params.ParamError:
                self.err(f"skyclient.c:parse_key_value_params():529: Can't parse value '{value}' for param {key}\n")
                raise Exit(255)
        return values

    def cmd_set_dev_param(self, opts, args):
        self.dev.set_dev_params(self._parse_pairs(opts, args, params.DEV_PARAMS))

    cmd_set_dev_params = cmd_set_dev_param

    def cmd_set_sink_param(self, opts, args):
        values = self._parse_pairs(opts, args, params.SINK_PARAMS)
        if self.dev.get_dev_params()['detect-mode'] == params.DETECT_PLC and not self.dev.sink_linked():
            self.err(f'skyclient.c:main():1076: sky_sink_paramsset(): {os.strerror(errno.ENOTCONN)}\n')
            raise Exit(255)
        self.dev.set_sink_params(values)

    cmd_set_sink_params = cmd_set_sink_param

    def cmd_resume_scan(self, opts, args):
        self.dev.scan_resume()

    def cmd_stop_scan(self, opts, args):
        self.dev.scan_stop()

    def _no_droneport(self, func):
        self.err(f'skyclient.c:main():1100: {func}(): {os.strerror(errno.EOPNOTSUPP)}\n')
        raise Exit(255)

    def cmd_open_droneport(self, opts, args):
        self._no_droneport('sky_droneport_open')

    def cmd_close_droneport(self, opts, args):
        self._no_droneport('sky_droneport_close')

    def cmd_show_droneport_state(self, opts, args):
        self._no_droneport('sky_droneport_state')

    def cmd_drone_detect(self, opts, args):
        if self.dev.sink_linked():
            self.out('Drone is detected on the charging pad.\n')
        else:
            self.out('Drone is not detected on the charging pad.\n')

    def cmd_reset(self, opts, args):
        self.dev.reset()

    def cmd_dev_info(self, opts, args):
        self.print_info('Source', '--json' in opts)

    def cmd_gps_info(self, opts, args):
        self.err(f'skyclient.c:main():1162: sky_gpsdata(): {os.strerror(errno.EOPNOTSUPP)}\n')
        raise Exit(255)

    def cmd_sink_info(self, opts, args):
        if self.dev.get_dev_params()['detect-mode'] != params.DETECT_PLC or not self.dev.sink_linked():
            self.err(f'skyclient.c:main():1217: sky_sink_infoget(): {os.strerror(errno.ENOTCONN)}\n')
            raise Exit(255)
        self.print_info('Sink', '--json' in opts)

    def cmd_sink_start_charge(self, opts, args):
        self.dev.charge_start()

    def cmd_sink_stop_charge(self, opts, args):
        self.dev.charge_stop()

    def _sink_unsupported(self, func):
        self.err(f'skyclient.c:main():1272: {func}(): {os.strerror(errno.EOPNOTSUPP)}\n')
        raise Exit(255)

    def cmd_sink_flash_info(self, opts, args):
        self._sink_unsupported('sky_sink_flash_info')

    def cmd_sink_firmware_update(self, opts, args):
        self._sink_unsupported('sky_sink_flash_info')

    def cmd_sink_passthru_send(self, opts, args):
        self._sink_unsupported('sky_sink_passthru_msgsend')

    def cmd_sink_passthru_recv(self, opts, args):
        self._sink_unsupported('sky_sink_passthru_msgrecv')


def run_command_line(device, line, out, err, stop=None):
    """Run "skycharge-cli ..." given as one shell-quoted string. Returns the exit status."""
    argv = shlex.split(line)
    if argv and os.path.basename(argv[0]) == 'skycharge-cli':
        argv = argv[1:]
    return Cli(device, out, err, stop).run(argv)
