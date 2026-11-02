# AP_FLAKE8_CLEAN
"""Checks of the CopterSonde Gazebo atmosphere: the committed table and the PBL model behind it."""

import importlib.util
import os

import numpy as np
import pytest

SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TABLE = os.path.join(SIM, 'atmosphere', 'norman_pbl.csv')

spec = importlib.util.spec_from_file_location('pbl_model', os.path.join(SIM, 'tools', 'pbl_model.py'))
pbl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pbl)


@pytest.fixture(scope='module')
def table():
    lines = [ln for ln in open(TABLE) if not ln.startswith('#')]
    names = lines[0].strip().split(',')
    d = np.loadtxt(lines[1:], delimiter=',')
    hours = np.unique(d[:, 0])
    heights = np.unique(d[:, 1])
    grid = {k: d[:, n].reshape(len(hours), len(heights)) for n, k in enumerate(names)}
    return hours, heights, grid


def at(table, hour, z):
    hours, heights, grid = table
    i = int(np.argmin(abs(hours - hour)))
    k = int(np.argmin(abs(heights - z)))
    return {name: grid[name][i, k] for name in grid}, grid, i


def test_grid_is_complete_and_ordered(table):
    hours, heights, grid = table
    assert hours[0] == 0 and hours[-1] < 24
    assert np.allclose(np.diff(hours), hours[1] - hours[0])
    assert heights[0] == 0 and heights[-1] >= 3048, 'the table must reach 10000 ft'
    assert grid['T_K'].shape == (len(hours), len(heights))
    assert np.all(np.diff(grid['p_Pa'], axis=1) < 0), 'pressure falls with height'


def test_values_are_physical(table):
    _, _, g = table
    assert np.all((g['RH_pct'] >= 0) & (g['RH_pct'] <= 100))
    assert np.all((g['T_K'] > 260) & (g['T_K'] < 320))
    assert np.all(np.hypot(g['u_east_ms'], g['v_north_ms']) < 25)
    assert np.all(g['sigma_u_ms'] > 0) and np.all(g['sigma_w_ms'] > 0)


def test_day_wraps_without_a_jump(table):
    _, _, g = table
    # the cycle repeats: 23:30 -> 00:00 changes no more than a typical half-hour step
    steps = np.abs(np.diff(g['T_K'], axis=0)).max(axis=1)
    wrap = np.abs(g['T_K'][0] - g['T_K'][-1]).max()
    assert wrap <= 1.5 * np.percentile(steps, 90)


def test_surface_diurnal_cycle(table):
    _, _, g = table
    t2 = g['T_K'][:, 0] - 273.15
    assert 16 < t2.min() < 24, 'late-summer Norman dawn minimum'
    assert 31 < t2.max() < 38, 'late-summer Norman afternoon maximum'
    hours = np.arange(len(t2)) * 24 / len(t2)
    assert 5 <= hours[np.argmin(t2)] <= 8
    assert 14 <= hours[np.argmax(t2)] <= 18


def test_nocturnal_inversion_and_low_level_jet(table):
    dawn, g, i = at(table, 5.0, 0)
    _, heights, _ = table
    T = g['T_K'][i]
    assert T[heights == 300][0] - T[0] > 3, 'surface-based inversion before dawn'
    spd = np.hypot(g['u_east_ms'][i], g['v_north_ms'][i])
    low = (heights > 100) & (heights < 700)
    nose = heights[low][np.argmax(spd[low])]
    assert spd[low].max() > 1.5 * spd[heights == 1500][0], 'jet maximum stands out above the residual layer'
    assert 150 <= nose <= 600


def test_afternoon_mixed_layer(table):
    _, g, i = at(table, 16.0, 0)
    _, heights, _ = table
    T = g['T_K'][i]
    p = g['p_Pa'][i]
    theta = T * (1e5 / p) ** (pbl.RD / pbl.CP)
    ml = (heights >= 200) & (heights <= 1500)
    assert theta[ml].max() - theta[ml].min() < 1.0, 'well mixed potential temperature'
    rh = g['RH_pct'][i]
    assert np.argmax(rh[heights <= 3000]) > np.argmax(heights >= 1000), 'humidity peaks near the mixed-layer top'


def test_model_cycle_repeats():
    # a coarse run: the last day ends where it started (small residual spread over the day)
    f = pbl.Forcing()
    z = np.arange(0, 3001, 100.0)
    times, _, out, end = pbl.run(f, days=4, out_step_h=1.0, z_out=z, dt=120.0)
    start = pbl.Column(f).snapshot(z)
    assert len(times) == 24
    assert np.abs(end['T'] - out['T'][0]).max() < 2.0
    assert not np.allclose(start['T'], out['T'][0]), 'spin-up moved the column away from the reference'
