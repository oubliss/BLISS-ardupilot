# AP_FLAKE8_CLEAN
"""
Run the Skycharge digital twin.

  python3 -m skycharge_twin                      # bench: battery and contact inside the twin
  python3 -m skycharge_twin --gazebo             # the CopterSonde on the BOLOGNINI S1 in Gazebo

Then, as with the real pad:
  ssh -i id_rsa.skydevice -p 2222 root@127.0.0.1 skycharge-cli show-charging-state
  curl 'http://127.0.0.1:8088/v1/charging-state?user-uuid=...&device-uuid=...'
"""

import argparse
import os
import signal
import sys
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.dirname(HERE)


def host_port(text, default_host='127.0.0.1'):
    if text in ('off', 'none', ''):
        return None
    host, _, port = text.rpartition(':')
    return (host or default_host, int(port))


def main(argv=None):
    ap = argparse.ArgumentParser(prog='skycharge_twin', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--conf', default=os.path.join(DATA, 'skycharge.conf'),
                    help='skycharge.conf the source loads at start (default: %(default)s)')
    ap.add_argument('--ssh', default='127.0.0.1:2222', help='SSH listen address, or off (default: %(default)s)')
    ap.add_argument('--http', default='127.0.0.1:8088',
                    help='JSON API listen address, or off (default: %(default)s)')
    ap.add_argument('--http-rate-limit', type=float, default=0.0,
                    help='requests per second per client, as the cloud API (3); 0 = unlimited')
    ap.add_argument('--authorized-keys', action='append',
                    help='authorized_keys file or private key whose public half may log in '
                         '(default: the Skycharge vendor key in authorized_keys next to this package)')
    ap.add_argument('--host-key', default=os.path.expanduser('~/.cache/coptersonde/skycharge_twin/ssh_host_rsa_key'),
                    help='SSH host key, created if missing (default: %(default)s)')
    ap.add_argument('--modern-rsa', action='store_true',
                    help='also accept rsa-sha2-256/512 signatures (the real pad accepts only SHA-1 ssh-rsa)')
    ap.add_argument('--gazebo', action='store_true', help='use the CopterSonde in Gazebo as the plant')
    ap.add_argument('--command-topic', default='/skycharge/psu', help='gz topic to the plugin (default: %(default)s)')
    ap.add_argument('--sense-topic', default='/skycharge/sink', help='gz topic from the plugin (default: %(default)s)')
    bench = ap.add_argument_group('bench plant (without --gazebo)')
    bench.add_argument('--on-pad', action='store_true', help='start with the drone on the pad')
    bench.add_argument('--soc', type=float, default=0.6, help='battery state of charge 0..1 (default: %(default)s)')
    bench.add_argument('--cells', type=int, default=6)
    bench.add_argument('--capacity', type=float, default=9.0, help='Ah (default: %(default)s)')
    bench.add_argument('--chemistry', default='Li-Ion', choices=('Li-Ion', 'Li-Po'))
    bench.add_argument('--load', type=float, default=0.8, help="the drone's own current on the pad, A")
    args = ap.parse_args(argv)

    from .twin import Twin

    def log(text):
        print(text, flush=True)

    if args.gazebo:
        from .gz_plant import GazeboPlant
        plant = GazeboPlant(args.command_topic, args.sense_topic, log=log)
        where = f'Gazebo ({args.command_topic} -> plugin -> {args.sense_topic})'
    else:
        from .plant import StandalonePlant
        plant = StandalonePlant(cells=args.cells, capacity_ah=args.capacity, soc=args.soc, chemistry=args.chemistry,
                                load=args.load, contact=args.on_pad)
        where = f"bench battery {args.cells}S {args.capacity:g} Ah at {args.soc * 100:.0f} %, " \
                f"{'on' if args.on_pad else 'off'} the pad"
    conf = args.conf if os.path.exists(args.conf) else None
    twin = Twin(conf, plant, log=log)
    c = twin.device.conf
    log(f'Skycharge twin: device "{c.device_name}" id {c.dev_id}, plant: {where}')
    ssh = host_port(args.ssh)
    if ssh:
        from .ssh_server import SshServer
        keys = args.authorized_keys or [os.path.join(DATA, 'authorized_keys')]
        twin.ssh = SshServer(twin, ssh[0], ssh[1], keys, args.host_key, legacy_rsa_only=not args.modern_rsa,
                             log=log)
        log(f'  ssh  root@{ssh[0]} -p {ssh[1]}  (key only{"" if args.modern_rsa else ", ssh-rsa SHA-1 like the pad"})')
    http = host_port(args.http)
    if http:
        from .http_api import HttpApi
        twin.http = HttpApi(twin, http[0], http[1], rate_limit=args.http_rate_limit, log=log)
        log(f'  http http://{http[0]}:{http[1]}/v1/charging-state?user-uuid={c.user_uuid}&device-uuid={c.device_uuid}')
    twin.start()

    done = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: done.set())
    signal.signal(signal.SIGTERM, lambda *_: done.set())
    done.wait()
    twin.stop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
