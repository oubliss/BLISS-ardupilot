#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""
Plot the CopterSonde Gazebo atmosphere.

Writes two figures next to the table:
- <table>_day.png: the day in time-height sections of temperature (departure from the daily mean at each
  height), relative humidity and wind speed;
- <table>_vs_cswx.png (with --cswx): profiles at 09, 11 and 13 CDT against CopterSonde ascents.

--cswx takes a folder of CopterSonde DataFlash logs (IMET, RHUM, WIND, POS, GPS), such as the
CSWX high-altitude flights. Their ascents are cached in ~/.cache/coptersonde, so the logs are
read once.

  python3 plot_atmosphere.py --cswx "<folder of CopterSonde logs>"
"""

import argparse
import datetime as dt
import glob
import os

import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402

SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ZB = np.arange(0, 3101, 50.0)
UTC_OFFSET = -5          # CDT
SURFACE = '#fcfcfb'
TEXT = '#0b0b0b'
TEXT2 = '#52514e'
GRID = '#e4e3df'
MODEL = '#2a78d6'
DATA = '#eb6834'
CONTEXT = '#c3c2b7'
SEQUENTIAL = LinearSegmentedColormap.from_list('blue', [
    '#cde2fb', '#9ec5f4', '#6da7ec', '#3987e5', '#256abf', '#184f95', '#0d366b'])
DIVERGING = LinearSegmentedColormap.from_list(
    'blue_red', ['#184f95', '#3987e5', '#9ec5f4', '#f0efec', '#f2a3a2', '#e34948', '#a32a2a'])


def load_table(path):
    lines = [ln for ln in open(path) if not ln.startswith('#')]
    names = lines[0].strip().split(',')
    d = np.loadtxt(lines[1:], delimiter=',')
    hours = np.unique(d[:, 0])
    heights = np.unique(d[:, 1])
    return hours, heights, {k: d[:, n].reshape(len(hours), len(heights)) for n, k in enumerate(names)}


def read_ascents(folder):
    """Ascent profiles (binned every 50 m) of every log in folder, cached."""
    cache_dir = os.path.join(os.path.expanduser('~'), '.cache', 'coptersonde')
    os.makedirs(cache_dir, exist_ok=True)
    cache = os.path.join(cache_dir, 'ascents_%s.npz' % ''.join(c if c.isalnum() else '_' for c in os.path.abspath(folder)))
    if os.path.exists(cache):
        c = np.load(cache)
        return [dict(hour=h, T=t, RH=r, ws=w) for h, t, r, w in zip(c['hour'], c['T'], c['RH'], c['ws'])]
    from pymavlink import mavutil
    out = []
    for f in sorted(glob.glob(os.path.join(folder, '*.BIN'))):
        m = mavutil.mavlink_connection(f)
        pos, imet, rhum, wind, gps = [], [], [], [], None
        while True:
            msg = m.recv_match(type=['POS', 'IMET', 'RHUM', 'WIND', 'GPS'])
            if msg is None:
                break
            ty = msg.get_type()
            t = msg.TimeUS * 1e-6
            if ty == 'POS':
                pos.append((t, msg.RelHomeAlt))
            elif ty == 'IMET':
                v = [getattr(msg, 'T%d' % i) for i in range(1, 5) if getattr(msg, 'Hth%d' % i)]
                imet.append((t, np.median(v) - 273.15 if v else np.nan))
            elif ty == 'RHUM':
                v = [getattr(msg, 'H%d' % i) for i in range(1, 5) if getattr(msg, 'Hth%d' % i)]
                rhum.append((t, np.mean(v) if v else np.nan))
            elif ty == 'WIND':
                wind.append((t, msg.wspeed))
            elif gps is None and msg.GWk > 0:
                gps = (t, msg.GWk, msg.GMS)
        if not pos or gps is None:
            continue
        pos, imet, rhum, wind = (np.array(a) for a in (pos, imet, rhum, wind))
        z = pos[:, 1]
        top = int(np.argmax(z))
        first = int(np.argmax(z > 2))
        ta = pos[first:top + 1, 0]
        za = np.maximum.accumulate(z[first:top + 1])
        utc = dt.datetime(1980, 1, 6) + dt.timedelta(weeks=gps[1], milliseconds=gps[2] - 18000 - gps[0] * 1000 + ta[0] * 1000)
        prof = {'hour': (utc.hour + utc.minute / 60 + UTC_OFFSET) % 24}
        for name, a in (('T', imet), ('RH', rhum), ('ws', wind)):
            y = np.interp(ta, a[:, 0], a[:, 1])
            prof[name] = np.array([np.nanmean(y[np.abs(za - b) < 25]) if np.any(np.abs(za - b) < 25) else np.nan
                                   for b in ZB])
        out.append(prof)
        print('%s: %.2f h CDT, top %.0f m' % (os.path.basename(f), prof['hour'], za[-1]), flush=True)
    np.savez_compressed(cache, hour=[p['hour'] for p in out], T=[p['T'] for p in out],
                        RH=[p['RH'] for p in out], ws=[p['ws'] for p in out])
    return out


def style(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.6)
    ax.tick_params(colors=TEXT2, labelsize=8, length=0)
    for s in ax.spines.values():
        s.set_visible(False)


def plot_day(table, out):
    hours, heights, g = table
    h = np.append(hours, 24.0)
    fields = (('T_K', 'Temperature minus its daily mean at that height (K)', lambda x: x - x.mean(axis=0)),
              ('RH_pct', 'Relative humidity (%)', lambda x: x),
              ('speed', 'Wind speed (m/s)', None))
    fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True, facecolor=SURFACE)
    for ax, (key, label, conv) in zip(axes, fields):
        if key == 'speed':
            v = np.hypot(g['u_east_ms'], g['v_north_ms'])
        else:
            v = conv(g[key])
        v = np.vstack([v, v[:1]])
        style(ax)
        ax.grid(False)
        if key == 'T_K':
            lim = np.abs(v).max()
            pc = ax.pcolormesh(h, heights, v.T, cmap=DIVERGING, vmin=-lim, vmax=lim, shading='gouraud')
        else:
            pc = ax.pcolormesh(h, heights, v.T, cmap=SEQUENTIAL, shading='gouraud')
        cb = fig.colorbar(pc, ax=ax, pad=0.01)
        cb.outline.set_visible(False)
        cb.ax.tick_params(colors=TEXT2, labelsize=8, length=0)
        ax.set_title(label, loc='left', color=TEXT, fontsize=10)
        ax.set_ylabel('Height above ground (m)', color=TEXT2, fontsize=8)
        ax.set_ylim(0, 3500)
        ax.axhline(3048, color=TEXT2, linewidth=0.8, linestyle=(0, (4, 3)))
        ax.text(0.3, 3100, '10 000 ft', color=TEXT2, fontsize=7)
    axes[-1].set_xticks(range(0, 25, 3))
    axes[-1].set_xlabel('Local time (CDT, the day repeats)', color=TEXT2, fontsize=8)
    fig.suptitle('CopterSonde Gazebo atmosphere, Norman OK, late summer', color=TEXT, fontsize=12, x=0.07, ha='left')
    fig.tight_layout()
    fig.savefig(out, dpi=130, facecolor=SURFACE)
    print('wrote', out)


def plot_vs_cswx(table, ascents, out):
    hours, heights, g = table
    model_speed = np.hypot(g['u_east_ms'], g['v_north_ms'])
    rows = (('T', 'Temperature (°C)', g['T_K'] - 273.15),
            ('RH', 'Relative humidity (%)', g['RH_pct']),
            ('ws', 'Wind speed (m/s)', model_speed))
    cols = (9, 11, 13)
    fig, axes = plt.subplots(3, 3, figsize=(10, 10), sharey=True, facecolor=SURFACE)
    for r, (key, label, model) in enumerate(rows):
        for c, hour in enumerate(cols):
            ax = axes[r, c]
            style(ax)
            sel = [p for p in ascents if abs(p['hour'] - hour) < 0.75]
            for p in sel:
                ax.plot(p[key], ZB, color=CONTEXT, linewidth=0.8)
            if sel:
                with np.errstate(all='ignore'):
                    mean = np.nanmean([p[key] for p in sel], axis=0)
                ax.plot(mean, ZB, color=DATA, linewidth=2, label='CSWX mean')
            i = int(np.argmin(abs(hours - hour)))
            ax.plot(model[i], heights, color=MODEL, linewidth=2, label='Model')
            ax.set_ylim(0, 3100)
            if r == 0:
                ax.set_title('%02d CDT (%d ascents)' % (hour, len(sel)), color=TEXT, fontsize=10)
            if c == 0:
                ax.set_ylabel('Height above ground (m)', color=TEXT2, fontsize=8)
            ax.set_xlabel(label, color=TEXT2, fontsize=8)
            if r == 0 and c == 0:
                ax.plot([], [], color=CONTEXT, linewidth=0.8, label='CSWX ascents')
                ax.legend(frameon=False, fontsize=8, labelcolor=TEXT2, loc='upper right')
    fig.suptitle('Model against the CSWX ascents (2026-08-27, 09-03, 09-17, 09-25)', color=TEXT, fontsize=12,
                 x=0.07, ha='left')
    fig.tight_layout()
    fig.savefig(out, dpi=130, facecolor=SURFACE)
    print('wrote', out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--table', default=os.path.join(SIM, 'atmosphere', 'norman_pbl.csv'))
    ap.add_argument('--cswx', help='folder of CopterSonde DataFlash logs to compare with')
    args = ap.parse_args()
    table = load_table(args.table)
    base = os.path.splitext(args.table)[0]
    plot_day(table, base + '_day.png')
    if args.cswx:
        plot_vs_cswx(table, read_ascents(args.cswx), base + '_vs_cswx.png')


if __name__ == '__main__':
    main()
