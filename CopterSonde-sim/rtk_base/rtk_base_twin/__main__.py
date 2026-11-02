"""python -m rtk_base_twin [--listen HOST:PORT] [--location NAME] [--offset E N]"""

# AP_FLAKE8_CLEAN

import argparse
import math
import os
import sys

from .receiver import Receiver
from .server import BaseServer

LOCATIONS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'Tools', 'autotest',
                         'locations.txt')


def find_location(name):
    with open(LOCATIONS) as f:
        for line in f:
            if line.startswith(name + '='):
                lat, lon, alt = (float(v) for v in line.split('=', 1)[1].split(',')[:3])
                return lat, lon, alt
    sys.exit(f'rtk_base_twin: no location {name} in {LOCATIONS}')


def main():
    ap = argparse.ArgumentParser(prog='rtk_base_twin', description=__doc__)
    ap.add_argument('--listen', default='127.0.0.1:3101', help='TCP address LAAIRS connects to (default: %(default)s)')
    ap.add_argument('--location', default='CopterSonde_Norman',
                    help='Tools/autotest/locations.txt site the base stands at (default: %(default)s)')
    ap.add_argument('--offset', nargs=2, type=float, default=(12.0, -3.0), metavar=('EAST', 'NORTH'),
                    help='base position from the site, m (default: 12 -3)')
    ap.add_argument('--geoid', type=float, default=-27.0, help='geoid height at the site, m (default: %(default)s)')
    ap.add_argument('--station-id', type=int, default=0)
    ap.add_argument('--seed', type=int, default=None, help='random seed for the position errors')
    args = ap.parse_args()

    lat, lon, alt = find_location(args.location)
    east, north = args.offset
    lat += math.degrees(north / 6378137.0)
    lon += math.degrees(east / (6378137.0 * math.cos(math.radians(lat))))
    host, port = args.listen.rsplit(':', 1)

    def log(text):
        print(text, flush=True)

    receiver = Receiver(lat, lon, alt, geoid_m=args.geoid, station_id=args.station_id, log=log, seed=args.seed)
    server = BaseServer(receiver, host, int(port), log=log)
    log(f'RTK base twin (u-blox ZED-F9P) at {lat:.7f} {lon:.7f} {alt:.1f} m, {east:g} m E {north:g} m N of '
        f'{args.location}')
    log(f'  serial port socket://{server.address[0]}:{server.address[1]} (LAAIRS RTK Injection)')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()


if __name__ == '__main__':
    main()
