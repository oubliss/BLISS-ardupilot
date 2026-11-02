#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""
Compare a simulated CopterSonde profile with the air the Gazebo world gave it.

Reads the flight's DataFlash log (IMET, RHUM, WIND, POS) and the CopterSondePlugin truth log
(coptersonde_gazebo.sh --truth-log), splits the flight into ascent and descent, averages both in
height bins and reports, per bin and overall, the sensor minus truth difference of the iMet
temperature (median of the healthy beads), the corrected HYT271 humidity (mean of the healthy
sensors) and the wind-vane speed and direction (from the thrust-vector tilt).

  python3 profile_eval.py logs/00000003.BIN truth.csv [--bin 50] [--csv out.csv]
"""

import argparse
import math
import sys

import numpy as np
from pymavlink import mavutil


def read_log(path):
    m = mavutil.mavlink_connection(path)
    rows = {'IMET': [], 'RHUM': [], 'WIND': [], 'POS': []}
    while True:
        msg = m.recv_match(type=list(rows))
        if msg is None:
            break
        ty = msg.get_type()
        if ty == 'IMET':
            T = [getattr(msg, 'T%d' % i) for i in range(1, 5) if getattr(msg, 'Hth%d' % i)]
            rows[ty].append((msg.TimeUS * 1e-6, np.median(T) if T else np.nan))
        elif ty == 'RHUM':
            H = [getattr(msg, 'H%d' % i) for i in range(1, 5) if getattr(msg, 'Hth%d' % i)]
            rows[ty].append((msg.TimeUS * 1e-6, np.mean(H) if H else np.nan))
        elif ty == 'WIND':
            rows[ty].append((msg.TimeUS * 1e-6, msg.wspeed, msg.wdir))
        else:
            rows[ty].append((msg.TimeUS * 1e-6, msg.RelHomeAlt))
    return {k: np.array(v) for k, v in rows.items()}


def legs(t, z):
    """Index masks of the ascent (takeoff to top) and descent (top to landing)."""
    air = z > 2.0
    if not air.any():
        sys.exit('no flight in this log')
    first = np.argmax(air)
    last = len(z) - 1 - np.argmax(air[::-1])
    top = first + int(np.argmax(z[first:last + 1]))
    idx = np.arange(len(z))
    return (idx >= first) & (idx <= top), (idx > top) & (idx <= last)


def circ_mean(deg):
    r = np.radians(deg)
    return math.degrees(math.atan2(np.nanmean(np.sin(r)), np.nanmean(np.cos(r)))) % 360


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log', help='DataFlash log of the flight (.BIN)')
    ap.add_argument('truth', help='CopterSondePlugin truth CSV')
    ap.add_argument('--bin', type=float, default=50.0, help='height bin (m)')
    ap.add_argument('--csv', help='write the binned profiles here')
    args = ap.parse_args()

    log = read_log(args.log)
    tr = np.genfromtxt(args.truth, delimiter=',', names=True)
    tp, zp = log['POS'][:, 0], log['POS'][:, 1]
    T_s = np.interp(tp, log['IMET'][:, 0], log['IMET'][:, 1]) - 273.15
    H_s = np.interp(tp, log['RHUM'][:, 0], log['RHUM'][:, 1])
    W_s = np.interp(tp, log['WIND'][:, 0], log['WIND'][:, 1])
    D_s = log['WIND'][:, 2]
    D_s = np.degrees(np.interp(tp, log['WIND'][:, 0], np.unwrap(np.radians(D_s)))) % 360
    zt = tr['z_up_m']
    spd_t = np.hypot(tr['wind_e_ms'], tr['wind_n_ms'])
    dir_t = np.degrees(np.arctan2(-tr['wind_e_ms'], -tr['wind_n_ms'])) % 360
    up_l, dn_l = legs(tp, zp)
    up_t, dn_t = legs(tr['time_s'], zt)
    top = zp.max()
    edges = np.arange(0, top + args.bin, args.bin)
    out = []
    print('%-7s %6s | %7s %7s %6s | %6s %6s %6s | %5s %5s %5s | %4s %4s %5s' % (
        'leg', 'z m', 'T sens', 'T true', 'dT', 'RH sen', 'RH tru', 'dRH', 'U sen', 'U tru', 'dU', 'D sn', 'D tr', 'dD'))
    stats = {}
    for name, ml, mt in (('ascent', up_l, up_t), ('descent', dn_l, dn_t)):
        diffs = {'T': [], 'RH': [], 'U': [], 'D': []}
        for lo, hi in zip(edges[:-1], edges[1:]):
            a = ml & (zp >= lo) & (zp < hi)
            b = mt & (zt >= lo) & (zt < hi)
            if a.sum() < 3 or b.sum() < 3:
                continue
            row = (name, 0.5 * (lo + hi), np.nanmean(T_s[a]), np.nanmean(tr['T_K'][b]) - 273.15,
                   np.nanmean(H_s[a]), np.nanmean(tr['RH_pct'][b]), np.nanmean(W_s[a]), np.nanmean(spd_t[b]),
                   circ_mean(D_s[a]), circ_mean(dir_t[b]))
            out.append(row)
            dT, dH, dU = row[2] - row[3], row[4] - row[5], row[6] - row[7]
            dD = (row[8] - row[9] + 180) % 360 - 180
            diffs['T'].append(dT)
            diffs['RH'].append(dH)
            if lo >= 20:      # the wind vane works above 10-20 m
                diffs['U'].append(dU)
                diffs['D'].append(dD)
            print('%-7s %6.0f | %7.2f %7.2f %+6.2f | %6.1f %6.1f %+6.1f | %5.1f %5.1f %+5.1f | %4.0f %4.0f %+5.0f' % (
                row[0], row[1], row[2], row[3], dT, row[4], row[5], dH, row[6], row[7], dU, row[8], row[9], dD))
        stats[name] = {k: (np.mean(v), np.sqrt(np.mean(np.square(v)))) for k, v in diffs.items() if v}
    print()
    for name, s in stats.items():
        print('%-7s mean / RMS of sensor - truth: ' % name + ', '.join(
            '%s %+.2f / %.2f' % (k, *v) for k, v in s.items()))
    if args.csv:
        with open(args.csv, 'w') as f:
            f.write('leg,z_m,T_sensor_C,T_true_C,RH_sensor,RH_true,U_vane,U_true,D_vane,D_true\n')
            for r in out:
                f.write('%s,%.1f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f\n' % r)


if __name__ == '__main__':
    main()
