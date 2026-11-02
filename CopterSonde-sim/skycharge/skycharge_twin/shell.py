# AP_FLAKE8_CLEAN
"""
The little shell behind the twin's SSH server: what "ssh root@skydevice.local <command>" runs.

It knows skycharge-cli, a few commands an operator uses on the BeagleBone (cat /etc/skycharge.conf,
systemctl ... skycharged, hostname, uname, echo, true, false) and the twin's own control command
"skytwin", which plays the parts the real world plays: placing the drone on the pad, injecting faults,
cycling the mains. Commands can be chained with ";", "&&" and "||".
"""

import re
import shlex

from . import cli as cli_mod
from .device import FAULTS, ON_PAD

SKYTWIN_HELP = """skytwin - control the Skycharge digital twin

    skytwin status                  state, plant and faults
    skytwin place | remove          put the drone on the pad / take it off (standalone plant only)
    skytwin battery SOC             set the battery state of charge, 0..1 (standalone plant only)
    skytwin load AMPS               the drone's own current draw while on the pad (standalone plant only)
    skytwin fault list              faults that can be injected
    skytwin fault set NAME          inject a fault
    skytwin fault clear [NAME]      remove one fault, or all
    skytwin power-cycle [SECS]      mains off/on: the source reboots and reloads skycharge.conf
"""


class Shell:
    def __init__(self, twin):
        self.twin = twin
        self.device = twin.device

    def run(self, line, out, err, stop=None):
        """Run a command line. Returns the exit status of the last command."""
        rc = 0
        for op, cmd in self._split(line):
            if op == '&&' and rc != 0 or op == '||' and rc == 0:
                continue
            rc = self._run_one(cmd, out, err, stop)
        return rc

    @staticmethod
    def _split(line):
        parts = re.split(r'(;|&&|\|\|)', line)
        op = ';'
        for p in parts:
            if p in (';', '&&', '||'):
                op = p
                continue
            if p.strip():
                yield op, p.strip()

    def _run_one(self, cmd, out, err, stop):
        try:
            argv = shlex.split(cmd)
        except ValueError as e:
            err(f'bash: {e}\n')
            return 2
        if not argv:
            return 0
        prog = argv[0].rsplit('/', 1)[-1]
        if prog == 'skycharge-cli':
            return cli_mod.Cli(self.device, out, err, stop).run(argv[1:])
        if prog == 'skytwin':
            return self.skytwin(argv[1:], out, err)
        if prog == 'echo':
            out(' '.join(argv[1:]) + '\n')
            return 0
        if prog in ('true', ':'):
            return 0
        if prog == 'false':
            return 1
        if prog == 'hostname':
            out('skydevice\n')
            return 0
        if prog == 'uname':
            out('Linux skydevice 4.19.94-ti-r42 #1buster SMP PREEMPT armv7l GNU/Linux\n' if '-a' in argv
                else 'Linux\n')
            return 0
        if prog == 'cat' and argv[1:] == ['/etc/skycharge.conf']:
            out(self.twin.conf_text())
            return 0
        if prog == 'systemctl' and len(argv) == 3 and argv[2] in ('skycharged', 'skycharged.service'):
            return self.systemctl(argv[1], out, err)
        err(f'bash: {argv[0]}: command not found\n')
        return 127

    def systemctl(self, verb, out, err):
        d = self.device
        if verb == 'restart' or verb == 'start':
            d.daemon_running = True
            d.restart_daemon()
            return 0
        if verb == 'stop':
            d.daemon_running = False
            d.log('skycharged stopped')
            return 0
        if verb == 'status':
            active = 'active (running)' if d.daemon_running else 'inactive (dead)'
            out(f'* skycharged.service - Skycharge daemon\n   Active: {active}\n')
            return 0 if d.daemon_running else 3
        err(f'Unknown operation {verb}.\n')
        return 1

    def skytwin(self, argv, out, err):
        d = self.device
        plant = self.twin.plant
        standalone = hasattr(plant, 'place')
        if not argv or argv[0] in ('help', '-h', '--help'):
            out(SKYTWIN_HELP)
            return 0
        verb = argv[0]
        if verb == 'status':
            st = d.charging_state()
            m = d.measurement
            out(f"state:      {st['state'].name} ({st['reason'].name})\n"
                f"on pad:     {'yes' if st['state'] in ON_PAD else 'no'}\n"
                f'plant:      {type(plant).__name__}\n')
            if m is not None:
                soc = f'{m.pack_soc * 100:.1f} %' if m.pack_soc >= 0 else '?'
                out(f"contact:    {'yes' if m.contact else 'no'}{'' if m.valid else ' (no simulator data)'}\n"
                    f'output:     {m.voltage:.2f} V {m.current:.2f} A\n'
                    f'battery:    {m.pack_voltage:.2f} V, SoC {soc}\n'
                    f'ambient:    {m.ambient_temperature:.1f} C {m.ambient_humidity:.0f} %\n')
            out(f"faults:     {', '.join(sorted(d.faults)) or 'none'}\n"
                f"daemon:     {'running' if d.daemon_running else 'stopped'}"
                f"{' (booting)' if d.booting else ''}\n")
            return 0
        if verb in ('place', 'remove', 'battery', 'load'):
            if not standalone:
                err(f'skytwin {verb}: the {type(plant).__name__} decides that (move the drone in the simulator)\n')
                return 1
            if verb in ('place', 'remove'):
                plant.place(verb == 'place')
                d.log(f"twin: drone {'placed on' if verb == 'place' else 'removed from'} the pad")
                return 0
            if len(argv) != 2:
                err(f'usage: skytwin {verb} VALUE\n')
                return 2
            try:
                value = float(argv[1])
            except ValueError:
                err(f'skytwin {verb}: not a number: {argv[1]}\n')
                return 2
            if verb == 'battery':
                plant.soc = min(max(value, 0.0), 1.0)
            else:
                plant.load = max(value, 0.0)
            d.log(f'twin: {verb} set to {value:g}')
            return 0
        if verb == 'fault':
            if len(argv) < 2 or argv[1] == 'list':
                out(''.join(f'{k:<18} {v}\n' for k, v in FAULTS.items()))
                return 0
            if argv[1] == 'set' and len(argv) == 3 and argv[2] in FAULTS:
                with d.lock:
                    d.faults.add(argv[2])
                    d.log(f'twin: fault {argv[2]} injected')
                return 0
            if argv[1] == 'clear':
                with d.lock:
                    names = argv[2:] or sorted(d.faults)
                    for n in names:
                        d.faults.discard(n)
                        d.log(f'twin: fault {n} cleared')
                return 0
            err('usage: skytwin fault list | set NAME | clear [NAME]\n')
            return 2
        if verb == 'power-cycle':
            secs = float(argv[1]) if len(argv) > 1 else None
            self.twin.power_cycle(secs)
            return 0
        err(f'skytwin: unknown command {verb}\n' + SKYTWIN_HELP)
        return 2
