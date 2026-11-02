#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""
Diurnal planetary boundary layer (PBL) for the CopterSonde Gazebo simulator.

A one-dimensional column model (potential temperature, specific humidity and horizontal wind)
is run over repeated days with the same forcing until the daily cycle repeats. The last day is
written as a table of temperature, relative humidity, pressure, wind and turbulence against
height and local time, which the Gazebo AtmosphericProfilePlugin interpolates.

Physics:
- surface sensible and latent heat fluxes that follow the sun (clear-sky day), and at night a
  bulk heat flux towards the cooling ground;
- non-local K-profile mixing in the convective boundary layer (Holtslag and Boville 1993) and
  local Richardson-number mixing (long tail) at night, both with implicit diffusion;
- Coriolis force and a geostrophic wind that veers with height, so the nocturnal low-level jet
  forms as an inertial oscillation over the decoupled residual layer (Blackadar 1957);
- relaxation to a reference sounding (radiation and advection), which closes the daily budget.

The default forcing is tuned to the CopterSonde CSWX profiles flown at the Kessler/Norman area,
Oklahoma, on 2026-08-27, 09-03, 09-17 and 09-25 (09-13 CDT, up to 2438 m AGL).
Run with --help for the options.
"""

import argparse
import math
import os

import numpy as np
from scipy.linalg import solve_banded

G = 9.80665
RD = 287.05
CP = 1004.6
KAPPA_VK = 0.4
P0 = 100000.0
OMEGA = 7.2921e-5


class Forcing:
    """Site and season. Times are local clock hours (CDT for Norman in late summer).

    The fluxes, cooling, reference sounding and wind speeds were fitted to the composite of the
    32 CSWX ascents (09-13 CDT, temperature, humidity and wind every 100 m up to 2400 m), together
    with the season's mean dawn minimum (20.5 C) and afternoon maximum (34.5 C).
    """

    latitude = 35.24            # deg
    p_surface = 97000.0         # Pa at the ground (358 m MSL)
    sunrise = 7.15              # h
    sunset = 19.7               # h
    heat_flux_max = 0.242       # K m/s at solar noon (about 270 W/m2)
    # night: a bulk heat flux carries the air towards the ground, which cools towards its dawn value
    ground_dawn = 18.3          # degC
    ground_cooling_tau = 3 * 3600.0     # s
    night_exchange = 0.004      # bulk heat transfer coefficient
    moisture_flux_max = 0.105   # g/kg m/s at solar noon (about 300 W/m2 latent)
    z0 = 0.05                   # m, roughness length (grassland)
    # geostrophic wind: (height m, from-direction deg, speed m/s), linear in between
    geostrophic = [(0.0, 185.0, 7.7), (1000.0, 195.0, 6.1), (2000.0, 215.0, 3.6), (3500.0, 250.0, 6.1)]
    # reference sounding: potential temperature and specific humidity
    theta_ref_surface = 303.6   # K
    theta_ref_lapse = 3.4e-3    # K/m (free troposphere stability)
    q_ref_surface = 15.5        # g/kg
    q_ref_scale = 3000.0        # m
    # relaxation to the reference sounding: weak in the boundary layer, strong in the free troposphere
    relax_bl = 4 * 86400        # s
    relax_ft = 0.5 * 86400      # s
    free_troposphere = (1800.0, 2800.0)     # m, blend from boundary-layer to free-troposphere values
    # clear-air radiative cooling: all day below the free troposphere, and stronger near the ground at night
    cooling_background = 0.3 / 86400    # K/s
    cooling_night = 0.5 / 3600          # K/s at the ground
    cooling_night_scale = 537.0         # m
    damp_wind_above = 2800.0    # m, Rayleigh damping of inertial oscillations in the free troposphere
    damp_wind_tau = 6 * 3600.0  # s


def solar_shape(t_h, f):
    """0..1 shape of the daytime surface fluxes, positive between a little after sunrise and before sunset."""
    t_on = f.sunrise + 0.7
    t_off = f.sunset - 0.9
    if t_h <= t_on or t_h >= t_off:
        return 0.0
    return math.sin(math.pi * (t_h - t_on) / (t_off - t_on))


def daytime_fluxes(t_h, f):
    """Sensible (K m/s) and latent (kg/kg m/s) heat flux in the day, None at night."""
    s = solar_shape(t_h, f)
    if s > 0:
        return f.heat_flux_max * s, f.moisture_flux_max * 1e-3 * s
    return None


def geostrophic(z, f):
    zs, ds, ss = zip(*f.geostrophic)
    d = np.radians(np.interp(z, zs, ds))
    s = np.interp(z, zs, ss)
    return -s * np.sin(d), -s * np.cos(d)


def es_pa(T):
    """Saturation vapour pressure over water (Pa), Bolton 1980."""
    Tc = T - 273.15
    return 611.2 * np.exp(17.67 * Tc / (Tc + 243.5))


def hydrostatic(z, theta, q, p_s):
    """Pressure and temperature from potential temperature, integrating dp/dz = -g p / (Rd Tv)."""
    p = np.empty_like(z)
    T = np.empty_like(z)
    p[0] = p_s
    for k in range(len(z)):
        if k > 0:
            # one fixed-point refinement of the layer mean virtual temperature
            Tv0 = T[k - 1] * (1 + 0.61 * q[k - 1])
            pk = p[k - 1] * math.exp(-G * (z[k] - z[k - 1]) / (RD * Tv0))
            Tk = theta[k] * (pk / P0) ** (RD / CP)
            Tvm = 0.5 * (Tv0 + Tk * (1 + 0.61 * q[k]))
            p[k] = p[k - 1] * math.exp(-G * (z[k] - z[k - 1]) / (RD * Tvm))
        T[k] = theta[k] * (p[k] / P0) ** (RD / CP)
    return p, T


def relative_humidity(q, T, p):
    e = q * p / (0.622 + 0.378 * q)
    return 100.0 * e / es_pa(T)


def q_saturation(T, p):
    e = es_pa(T)
    return 0.622 * e / (p - 0.378 * e)


def stability_function(ri):
    """Local mixing reduction with the gradient Richardson number (long tail when stable)."""
    out = np.ones_like(ri)
    unstable = ri < 0
    out[unstable] = np.sqrt(1 - 16 * ri[unstable])
    stable = ~unstable
    out[stable] = 1.0 / (1 + 10 * ri[stable])
    return out


class Column:
    def __init__(self, f, dz=20.0, ztop=4000.0, dt=30.0):
        self.f = f
        self.dz = dz
        self.dt = dt
        self.z = np.arange(dz, ztop + dz / 2, dz)        # model levels
        self.zi = np.arange(dz * 1.5, ztop, dz)          # interfaces between levels
        self.fc = 2 * OMEGA * math.sin(math.radians(f.latitude))
        self.ug, self.vg = geostrophic(self.z, f)
        self.theta_ref = f.theta_ref_surface + f.theta_ref_lapse * self.z
        self.ft = np.clip((self.z - f.free_troposphere[0]) / (f.free_troposphere[1] - f.free_troposphere[0]), 0, 1)
        self.relax = 1.0 / (f.relax_bl + (f.relax_ft - f.relax_bl) * self.ft)
        self.q_ref = f.q_ref_surface * 1e-3 * np.exp(-self.z / f.q_ref_scale)
        self.theta = self.theta_ref.copy()
        self.q = self.q_ref.copy()
        self.u = self.ug.copy()
        self.v = self.vg.copy()
        self.ustar = 0.3
        self.h = 500.0
        self.wstar = 0.0
        self.H = 0.0
        self.E = 0.0
        self.Kh = np.zeros(len(self.zi))
        self.theta_ground = self.theta[0]
        self.night = False
        self.theta_ground_dawn = (f.ground_dawn + 273.15) * (P0 / f.p_surface) ** (RD / CP)

    def _bl_height(self, wm, H):
        """Bulk Richardson boundary-layer height, with the convective thermal excess."""
        th1 = self.theta[0]
        excess = 6.5 * H / wm if H > 0 else 0.0
        U2 = np.maximum((self.u - self.u[0]) ** 2 + (self.v - self.v[0]) ** 2, 1e-2) + 100 * self.ustar ** 2
        rib = G * (self.z - self.z[0]) * (self.theta - (th1 + excess)) / (th1 * U2)
        above = np.where((rib > 0.25) & (self.z > self.z[0]))[0]
        if len(above) == 0:
            return self.z[-1]
        k = above[0]
        if k == 0:
            return self.z[0]
        r0, r1 = rib[k - 1], rib[k]
        return self.z[k - 1] + (0.25 - r0) / max(r1 - r0, 1e-6) * self.dz

    def _diffuse(self, x, K, flux_sfc, counter=None):
        """Backward-Euler diffusion with a surface flux and an optional counter-gradient term."""
        n = len(x)
        a = K * self.dt / self.dz ** 2
        ab = np.zeros((3, n))
        ab[1, :] = 1.0
        ab[1, :-1] += a
        ab[1, 1:] += a
        ab[0, 1:] = -a
        ab[2, :-1] = -a
        rhs = x.copy()
        rhs[0] += flux_sfc * self.dt / self.dz
        if counter is not None:
            cflux = K * counter                         # K * gamma at the interfaces
            rhs[:-1] -= cflux * self.dt / self.dz
            rhs[1:] += cflux * self.dt / self.dz
        return solve_banded((1, 1), ab, rhs)

    def step(self, t_h):
        f = self.f
        z1 = self.z[0]
        U1 = max(math.hypot(self.u[0], self.v[0]), 0.3)
        day = daytime_fluxes(t_h, f)
        if day is not None:
            H, E = day
            self.night = False
        else:
            if not self.night:
                self.night = True
                self.theta_ground = self.theta[0]
            self.theta_ground += (self.theta_ground_dawn - self.theta_ground) * self.dt / f.ground_cooling_tau
            H = float(np.clip(f.night_exchange * U1 * (self.theta_ground - self.theta[0]), -0.06, 0.02))
            E = 0.0
        # surface layer: Monin-Obukhov with Businger-Dyer functions
        L = -self.ustar ** 3 * self.theta[0] / (KAPPA_VK * G * H) if abs(H) > 1e-6 else 1e9
        zeta = float(np.clip(z1 / L, -5.0, 1.0))
        if zeta < 0:
            x = (1 - 16 * zeta) ** 0.25
            psim = 2 * math.log((1 + x) / 2) + math.log((1 + x * x) / 2) - 2 * math.atan(x) + math.pi / 2
        else:
            psim = -5 * zeta
        self.ustar = max(KAPPA_VK * U1 / (math.log(z1 / f.z0) - psim), 0.02)
        self.H, self.E = H, E
        wstar = (G / self.theta[0] * H * self.h) ** (1 / 3) if H > 0 else 0.0
        wm = (self.ustar ** 3 + 0.28 * wstar ** 3) ** (1 / 3)
        self.h = float(np.clip(self._bl_height(wm, H), z1, self.z[-1]))
        self.wstar = wstar
        # local mixing from the gradient Richardson number
        dth = np.diff(self.theta) / self.dz
        du = np.diff(self.u) / self.dz
        dv = np.diff(self.v) / self.dz
        shear2 = np.maximum(du ** 2 + dv ** 2, 1e-6)
        ri = G / self.theta[:-1] * dth / shear2
        lam = 40.0 if H <= 0 else 150.0
        lmix = KAPPA_VK * self.zi / (1 + KAPPA_VK * self.zi / lam)
        K_local = lmix ** 2 * np.sqrt(shear2) * stability_function(ri)
        K = np.maximum(K_local, 0.05)
        counter_th = None
        counter_q = None
        if H > 0:
            inside = self.zi < self.h
            zr = self.zi[inside] / self.h
            K_nl = KAPPA_VK * wm * self.zi[inside] * (1 - zr) ** 2
            K[inside] = np.maximum(K[inside], K_nl)
            counter_th = np.zeros_like(K)
            counter_q = np.zeros_like(K)
            counter_th[inside] = 6.5 * H / (wm * self.h)
            counter_q[inside] = 6.5 * E / (wm * self.h)
        else:
            inside = self.zi < self.h
            K_nl = KAPPA_VK * self.ustar * self.zi[inside] * (1 - self.zi[inside] / self.h) ** 2
            K[inside] = np.maximum(K[inside], K_nl)
        self.Kh = K
        # momentum: Coriolis as an exact rotation of the ageostrophic wind, then friction and mixing
        c, s = math.cos(self.fc * self.dt), math.sin(self.fc * self.dt)
        ua, va = self.u - self.ug, self.v - self.vg
        self.u = self.ug + c * ua + s * va
        self.v = self.vg - s * ua + c * va
        tau = self.ustar ** 2 / U1
        self.u = self._diffuse(self.u, K, -tau * self.u[0])
        self.v = self._diffuse(self.v, K, -tau * self.v[0])
        damp = self.z > f.damp_wind_above
        self.u[damp] += (self.ug[damp] - self.u[damp]) * self.dt / f.damp_wind_tau
        self.v[damp] += (self.vg[damp] - self.v[damp]) * self.dt / f.damp_wind_tau
        # scalars
        self.theta = self._diffuse(self.theta, K, H, counter_th)
        self.q = self._diffuse(self.q, K, E, counter_q)
        self.theta += (self.theta_ref - self.theta) * self.dt * self.relax
        self.q += (self.q_ref - self.q) * self.dt * self.relax
        self.theta -= f.cooling_background * (1 - self.ft) * self.dt
        if solar_shape(t_h, f) == 0:
            self.theta -= f.cooling_night * np.exp(-self.z / f.cooling_night_scale) * self.dt
        # convective adjustment of any statically unstable layer above the surface layer
        for k in range(1, len(self.theta)):
            if self.theta[k] < self.theta[k - 1] - 1e-3 and self.z[k] > 60:
                self.theta[k - 1] = self.theta[k] = 0.5 * (self.theta[k] + self.theta[k - 1])
        # condensation: cap at saturation (no cloud physics)
        p, T = hydrostatic(self.z, self.theta, self.q, f.p_surface)
        self.q = np.minimum(self.q, q_saturation(T, p))

    def snapshot(self, z_out):
        """Profiles on z_out (m AGL), with a near-surface (2 m) value at z = 0."""
        f = self.f
        z1 = self.z[0]
        u_eff = max(self.ustar, 0.3 * self.wstar, 0.05)
        dth_sfc = float(np.clip(self.H / (KAPPA_VK * u_eff) * math.log(z1 / 2.0), -5.0, 3.0))
        dq_sfc = float(np.clip(self.E / (KAPPA_VK * u_eff) * math.log(z1 / 2.0), 0, 2e-3))
        zc = np.concatenate([[0.0], self.z])
        theta = np.concatenate([[self.theta[0] + dth_sfc], self.theta])
        q = np.concatenate([[self.q[0] + dq_sfc], self.q])
        # log wind profile below the first level
        u = np.concatenate([[0.0], self.u])
        v = np.concatenate([[0.0], self.v])
        th_o = np.interp(z_out, zc, theta)
        q_o = np.interp(z_out, zc, q)
        u_o = np.interp(z_out, self.z, self.u)
        v_o = np.interp(z_out, self.z, self.v)
        low = z_out < z1
        scale = np.log(np.maximum(z_out[low], 2.0) / f.z0) / math.log(z1 / f.z0)
        u_o[low] = self.u[0] * scale
        v_o[low] = self.v[0] * scale
        del u, v
        p, T = hydrostatic(z_out, th_o, q_o, f.p_surface)
        rh = np.clip(relative_humidity(q_o, T, p), 0.0, 100.0)
        # turbulence: convective scaling in the day, surface-layer scaling at night
        zr = np.clip(z_out / self.h, 0.0, 1.0)
        inside = z_out < self.h
        sig_u = np.where(inside, np.sqrt((2.0 * self.ustar * (1 - zr) ** 0.75) ** 2 + (0.6 * self.wstar) ** 2), 0.0)
        sig_w = np.where(inside, np.sqrt((1.3 * self.ustar * (1 - zr) ** 0.75) ** 2
                                         + (1.1 * self.wstar * np.maximum(zr, 0.01) ** (1 / 3) * (1 - 0.8 * zr)) ** 2), 0.0)
        sig_u = np.maximum(sig_u, 0.15)
        sig_w = np.maximum(sig_w, 0.08)
        return dict(T=T, RH=rh, p=p, u=u_o, v=v_o, sig_u=sig_u, sig_w=sig_w, theta=th_o, q=q_o)


def run(f, days=8, out_step_h=0.5, z_out=None, dt=30.0, progress=False):
    """Spin up for `days` days and return the last day's profiles every out_step_h hours."""
    col = Column(f, dt=dt)
    if z_out is None:
        z_out = np.arange(0.0, 3500.0 + 1, 25.0)
    steps_per_day = int(round(86400 / dt))
    every = int(round(out_step_h * 3600 / dt))
    times = []
    snaps = []
    h_series = []
    for day in range(days):
        for n in range(steps_per_day):
            t_h = n * dt / 3600.0
            if day == days - 1 and n % every == 0:
                times.append(t_h)
                snaps.append(col.snapshot(z_out))
                h_series.append(col.h)
            col.step(t_h)
        if progress:
            print('day %d: h %.0f m, theta1 %.2f K' % (day + 1, col.h, col.theta[0]), flush=True)
    # the snapshot at 24 h equals the next day's 0 h: spread the remaining mismatch over the day
    end = col.snapshot(z_out)
    times = np.array(times)
    out = {k: np.array([s[k] for s in snaps]) for k in snaps[0]}
    for k in out:
        mismatch = end[k] - out[k][0]
        out[k] = out[k] - (times[:, None] / 24.0) * mismatch[None, :]
    out['h'] = np.array(h_series)
    return times, z_out, out, end


def write_table(path, times, z, prof, f, cycle_note):
    rows = []
    for i, t in enumerate(times):
        for k, zz in enumerate(z):
            rows.append('%.3f,%.1f,%.3f,%.2f,%.1f,%.3f,%.3f,%.3f,%.3f' % (
                t, zz, prof['T'][i, k], prof['RH'][i, k], prof['p'][i, k],
                prof['u'][i, k], prof['v'][i, k], prof['sig_u'][i, k], prof['sig_w'][i, k]))
    with open(path, 'w') as fh:
        fh.write('# CopterSonde Gazebo atmosphere: diurnal planetary boundary layer\n')
        fh.write('# generated by CopterSonde-sim/tools/pbl_model.py (%s)\n' % cycle_note)
        fh.write('# site latitude %.2f deg, surface pressure %.0f Pa, sunrise %.2f h, sunset %.2f h (local)\n' % (
            f.latitude, f.p_surface, f.sunrise, f.sunset))
        fh.write('# rows: every time (local hour, the cycle repeats every 24 h) x every height (m above ground)\n')
        fh.write('# u_east/v_north: wind velocity (towards), sigma: turbulence standard deviations\n')
        fh.write('hour,z_agl_m,T_K,RH_pct,p_Pa,u_east_ms,v_north_ms,sigma_u_ms,sigma_w_ms\n')
        fh.write('\n'.join(rows) + '\n')


def main():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=os.path.join(here, 'atmosphere', 'norman_pbl.csv'), help='output table')
    ap.add_argument('--days', type=int, default=8, help='spin-up days (the last one is written)')
    ap.add_argument('--step', type=float, default=0.5, help='output time step (h)')
    ap.add_argument('--dz', type=float, default=25.0, help='output height step (m)')
    ap.add_argument('--top', type=float, default=3500.0, help='top of the table (m AGL)')
    args = ap.parse_args()
    f = Forcing()
    z_out = np.arange(0.0, args.top + args.dz / 2, args.dz)
    times, z, prof, end = run(f, days=args.days, out_step_h=args.step, z_out=z_out, progress=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    write_table(args.out, times, z, prof, f, '%d spin-up days' % args.days)
    print('wrote %s: %d times x %d heights' % (args.out, len(times), len(z)))


if __name__ == '__main__':
    main()
