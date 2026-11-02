#!/usr/bin/env python3
"""
Generate the CopterSonde Norman, OK world for Gazebo Sim (Jetty).

The site is the RFSonde Norman world: a 2 km radius around KCRI at Max Westheimer Airport
(KOUN/OUN), Norman, OK, with the same takeoff point. Real positions (WGS84) are converted to a
local ENU frame (x=East, y=North, z=Up) whose origin is the drone takeoff point. Gazebo
<spherical_coordinates> and the SITL home location (CopterSonde_Norman in
Tools/autotest/locations.txt) are both set to that origin so GPS lat/lon match reality.

Sources
  - Radar / tower positions: see RADARS below (NWS PNS, NEXRAD site surveys,
    RFSonde ATD survey, OpenStreetMap).
  - Buildings, runways, taxiways, roads, trees: OpenStreetMap (osm/norman_osm.json,
    downloaded with the Overpass API; (c) OpenStreetMap contributors, ODbL).
  - Building heights: OSM height / building:levels when present, otherwise a
    generic height per building type. Synthetic trees near the radar site are
    generic placements (not surveyed) and are seeded so the world is repeatable.

Usage
  python3 make_norman_world.py            # regenerates models/cs_norman_* and worlds/
"""
import json
import math
import os
import random

import make_pad_models

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OSM_FILE = os.path.join(ROOT, "osm", "norman_osm.json")
MODELS = os.path.join(ROOT, "models")
WORLDS = os.path.join(ROOT, "worlds")
WORLD_NAME = "norman_coptersonde"
DECK_H = make_pad_models.DECK_H     # the drone starts on the pad, on the deck
PAD_TOP = make_pad_models.PAD_TOP

# ------------------------------------------------------------------ geodesy
WGS84_A = 6378137.0
WGS84_F = 1 / 298.257223563
WGS84_E2 = WGS84_F * (2 - WGS84_F)


def _ecef(lat, lon, h=0.0):
    la, lo = math.radians(lat), math.radians(lon)
    n = WGS84_A / math.sqrt(1 - WGS84_E2 * math.sin(la) ** 2)
    return ((n + h) * math.cos(la) * math.cos(lo),
            (n + h) * math.cos(la) * math.sin(lo),
            (n * (1 - WGS84_E2) + h) * math.sin(la))


class ENU:
    """Exact WGS84 -> local East/North (metres) about an origin."""

    def __init__(self, lat0, lon0):
        self.x0 = _ecef(lat0, lon0)
        self.la, self.lo = math.radians(lat0), math.radians(lon0)

    def __call__(self, lat, lon):
        x, y, z = _ecef(lat, lon)
        dx, dy, dz = x - self.x0[0], y - self.x0[1], z - self.x0[2]
        sla, cla, slo, clo = math.sin(self.la), math.cos(self.la), math.sin(self.lo), math.cos(self.lo)
        e = -slo * dx + clo * dy
        n = -sla * clo * dx - sla * slo * dy + cla * dz
        return e, n


# ------------------------------------------------------------------ site data
GROUND_MSL = 358.6   # m (NAVD88). ATD survey: cal-tower base 1176.42 ft; KOUN survey: ~1170-1181 ft

ATD_LAT, ATD_LON = 35.2362801551, -97.4636934874   # RFSonde survey, WGS84 (G1150)
# Takeoff point: open grass field 200 m north / 50 m east of the ATD radome.
_tmp = ENU(ATD_LAT, ATD_LON)
ORIGIN_LAT = ATD_LAT + 200.0 / 110950.0            # refined below to an exact offset
ORIGIN_LON = ATD_LON + 50.0 / (111320.0 * math.cos(math.radians(ATD_LAT)))

RADARS = {
    # name: (lat, lon, description/source)
    "KCRI": (35.2382871, -97.4602619,
             "WSR-88D (OSF-3), 30 m tower. OSM node 2336571439; NEXRAD OSF-3 survey 1994: "
             "35 14 18 N, 97 27 37 W (NAD83), 2950 MHz"),
    "KOUN": (35.2360553, -97.4623481,
             "WSR-88D dual-pol, 20 m tower, antenna centre 81 ft AGL. NWS PNS: 35 14 09.81 N, "
             "97 27 44.46 W; OSM node 1666791024"),
    "ATD": (ATD_LAT, ATD_LON,
            "NSSL Advanced Technology Demonstrator, S-band dual-pol phased array (4.3 m). "
            "Radome centre 368.668 m ortho (RFSonde survey)"),
    "ATD_cal_tower": (35.2401106492, -97.4635615285,
                      "ATD calibration tower, probe at 403.23 m ortho (RFSonde survey)"),
}
ATD_RADOME_CENTER_AGL = 368.668 - GROUND_MSL       # 10.07 m
CAL_PROBE_AGL = 403.23 - GROUND_MSL                # 44.63 m
ATD_ARRAY_OFFSET = 1.63                            # m, array face forward of radome centre
ATD_ARRAY_AZ_DEG = 1.6189                          # default: facing the cal tower
TAKEOFF_HEADING_DEG = 180.0                        # drone nose at spawn, compass degrees (0 = N, 90 = E); into the wind
ATD_RADOME_DIAMETER = 11.89                        # m, not published: same 39 ft radome as the WSR-88Ds

WSR88D_RADOME_D = 11.89                            # 39 ft radome
WSR88D_ANTENNA_ABOVE_TOWER = 81 * 0.3048 - 20.0    # 4.69 m (KOUN survey: 20 m tower -> 81 ft AGL)

OTHER_STRUCTURES = [
    # (name, lat, lon, kind, height_m)
    ("osm_tower_3884889523", 35.2373517, -97.4603558, "mast", 15.0),
    ("water_tower_1", 35.2340797, -97.4526616, "water_tower", 40.0),
    ("water_tower_2", 35.2412782, -97.4575774, "water_tower", 40.0),
    ("comm_mast_1", 35.2276341, -97.4491058, "mast", 30.0),
    ("comm_mast_2", 35.2264273, -97.4586828, "mast", 30.0),
    ("comm_mast_3", 35.2416558, -97.4396069, "mast", 30.0),
    ("airport_control_tower", 35.2408676, -97.4682652, "control_tower", 16.0),
]

BUILDING_HEIGHT = {   # generic heights when OSM has none (m)
    "house": 5.0, "detached": 5.0, "residential": 5.0, "garage": 3.0, "shed": 3.0,
    "apartments": 8.0, "hangar": 9.0, "commercial": 6.0, "retail": 6.5,
    "supermarket": 7.0, "industrial": 8.0, "warehouse": 8.0, "university": 8.0,
    "college": 8.0, "school": 7.0, "church": 9.0, "public": 7.0, "hotel": 15.0,
    "office": 10.0, "roof": 4.0, "construction": 5.0,
}
BUILDING_MATERIAL = {
    "house": "residential", "detached": "residential", "residential": "residential",
    "apartments": "residential", "garage": "residential", "shed": "residential",
    "hangar": "metal", "industrial": "metal", "warehouse": "metal", "roof": "metal",
    "university": "brick", "college": "brick", "school": "brick", "church": "brick",
    "public": "brick",
}
MATERIALS = {
    "residential": (0.80, 0.74, 0.62), "commercial": (0.70, 0.70, 0.68),
    "metal": (0.82, 0.84, 0.86), "brick": (0.62, 0.33, 0.24), "roof": (0.35, 0.33, 0.32),
    "runway": (0.20, 0.20, 0.21), "taxiway": (0.30, 0.30, 0.30), "apron": (0.40, 0.40, 0.40),
    "road_major": (0.27, 0.27, 0.28), "road_minor": (0.42, 0.42, 0.42),
    "trunk": (0.36, 0.25, 0.15), "canopy": (0.18, 0.40, 0.16), "canopy_dark": (0.12, 0.30, 0.12),
}
ROAD_WIDTH = {
    "motorway": 16, "trunk": 14, "primary": 13, "secondary": 11, "tertiary": 9,
    "secondary_link": 7, "tertiary_link": 7, "unclassified": 7, "residential": 7, "service": 4,
}
AEROWAY_WIDTH = {"runway": 30.48, "taxiway": 15.0, "taxilane": 10.0, "stopway": 30.48}


# ------------------------------------------------------------------ geometry helpers
def signed_area(poly):
    return 0.5 * sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]))


def triangulate(poly):
    """Ear clipping for a simple CCW polygon. Returns index triples."""
    idx = list(range(len(poly)))
    tris = []

    def is_convex(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) > 1e-12

    def inside(p, a, b, c):
        d1 = (p[0] - b[0]) * (a[1] - b[1]) - (a[0] - b[0]) * (p[1] - b[1])
        d2 = (p[0] - c[0]) * (b[1] - c[1]) - (b[0] - c[0]) * (p[1] - c[1])
        d3 = (p[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (p[1] - a[1])
        neg = d1 < 0 or d2 < 0 or d3 < 0
        pos = d1 > 0 or d2 > 0 or d3 > 0
        return not (neg and pos)

    guard = 0
    while len(idx) > 3 and guard < 10000:
        guard += 1
        n = len(idx)
        for k in range(n):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % n]
            a, b, c = poly[i0], poly[i1], poly[i2]
            if not is_convex(a, b, c):
                continue
            if any(inside(poly[j], a, b, c) for j in idx if j not in (i0, i1, i2)):
                continue
            tris.append((i0, i1, i2))
            idx.pop(k)
            break
        else:   # degenerate polygon: fall back to a fan
            break
    if len(idx) >= 3:
        tris += [(idx[0], idx[k], idx[k + 1]) for k in range(1, len(idx) - 1)]
    return tris


class Obj:
    """Minimal OBJ writer with per-face normals and materials (Z up)."""

    def __init__(self):
        self.v, self.vn, self.groups = [], [], {}

    def _vert(self, p):
        self.v.append(p)
        return len(self.v)

    def _norm(self, n):
        self.vn.append(n)
        return len(self.vn)

    def face(self, mat, pts, normal):
        ni = self._norm(normal)
        ids = [self._vert(p) for p in pts]
        self.groups.setdefault(mat, []).append([(i, ni) for i in ids])

    def prism(self, mat_wall, mat_top, poly, z0, z1):
        poly = [p for k, p in enumerate(poly) if k == 0 or math.dist(p, poly[k - 1]) > 0.05]
        if len(poly) > 2 and math.dist(poly[0], poly[-1]) < 0.05:
            poly = poly[:-1]
        if len(poly) < 3:
            return
        if signed_area(poly) < 0:
            poly = poly[::-1]
        for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
            L = math.hypot(x1 - x0, y1 - y0) or 1
            self.face(mat_wall, [(x0, y0, z0), (x1, y1, z0), (x1, y1, z1), (x0, y0, z1)],
                      ((y1 - y0) / L, -(x1 - x0) / L, 0))
        for a, b, c in triangulate(poly):
            self.face(mat_top, [(*poly[a], z1), (*poly[b], z1), (*poly[c], z1)], (0, 0, 1))

    def flat_poly(self, mat, poly, z):
        if len(poly) > 2 and math.dist(poly[0], poly[-1]) < 0.05:
            poly = poly[:-1]
        if len(poly) < 3:
            return
        if signed_area(poly) < 0:
            poly = poly[::-1]
        for a, b, c in triangulate(poly):
            self.face(mat, [(*poly[a], z), (*poly[b], z), (*poly[c], z)], (0, 0, 1))

    def ribbon(self, mat, line, width, z):
        """Flat strip along a polyline, with octagonal joints to hide gaps."""
        h = width / 2
        for (x0, y0), (x1, y1) in zip(line, line[1:]):
            L = math.hypot(x1 - x0, y1 - y0)
            if L < 0.01:
                continue
            nx, ny = -(y1 - y0) / L * h, (x1 - x0) / L * h
            self.face(mat, [(x0 - nx, y0 - ny, z), (x1 - nx, y1 - ny, z),
                            (x1 + nx, y1 + ny, z), (x0 + nx, y0 + ny, z)], (0, 0, 1))
        for (x, y) in line[1:-1]:
            self.flat_poly(mat, [(x + h * math.cos(a * math.pi / 4), y + h * math.sin(a * math.pi / 4))
                                 for a in range(8)], z)

    def cylinder(self, mat, cx, cy, r, z0, z1, sides=8):
        self.prism(mat, mat, [(cx + r * math.cos(2 * math.pi * k / sides),
                               cy + r * math.sin(2 * math.pi * k / sides)) for k in range(sides)], z0, z1)

    def blob(self, mat, cx, cy, cz, rx, rz, rings=4, segs=8):
        """Low-poly ellipsoid (tree canopy)."""
        def p(i, j):
            th = math.pi * i / rings
            ph = 2 * math.pi * j / segs
            return (cx + rx * math.sin(th) * math.cos(ph), cy + rx * math.sin(th) * math.sin(ph),
                    cz + rz * math.cos(th))
        for i in range(rings):
            for j in range(segs):
                a, b, c, d = p(i, j), p(i + 1, j), p(i + 1, j + 1), p(i, j + 1)
                mx, my, mz = [(a[k] + b[k] + c[k] + d[k]) / 4 for k in range(3)]
                n = ((mx - cx) / rx, (my - cy) / rx, (mz - cz) / rz)
                L = math.sqrt(sum(q * q for q in n)) or 1
                self.face(mat, [a, b, c, d] if i else [a, b, c], tuple(q / L for q in n))

    def write(self, path, name):
        mtl = os.path.basename(path).replace(".obj", ".mtl")
        with open(path.replace(".obj", ".mtl"), "w") as f:
            for m in self.groups:
                r, g, b = MATERIALS[m]
                f.write(f"newmtl {m}\nKa {r*.6:.3f} {g*.6:.3f} {b*.6:.3f}\nKd {r:.3f} {g:.3f} {b:.3f}\n"
                        "Ks 0.05 0.05 0.05\nNs 10\nd 1\nillum 2\n\n")
        with open(path, "w") as f:
            f.write(f"# {name} - generated by make_norman_world.py\nmtllib {mtl}\no {name}\n")
            f.writelines(f"v {x:.3f} {y:.3f} {z:.3f}\n" for x, y, z in self.v)
            f.writelines(f"vn {x:.4f} {y:.4f} {z:.4f}\n" for x, y, z in self.vn)
            for m, faces in self.groups.items():
                f.write(f"usemtl {m}\n")
                f.writelines("f " + " ".join(f"{v}//{n}" for v, n in fc) + "\n" for fc in faces)


# ------------------------------------------------------------------ SDF helpers
def material(r, g, b, a=1.0):
    return (f"<material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse>"
            f"<specular>0.1 0.1 0.1 1</specular></material>")


def box(name, x, y, z, sx, sy, sz, rgb, roll=0.0, pitch=0.0, yaw=0.0, collide=True):
    pose = f"<pose>{x:.3f} {y:.3f} {z:.3f} {roll:.5f} {pitch:.5f} {yaw:.5f}</pose>"
    geo = f"<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>"
    s = f"<visual name='{name}'>{pose}{geo}{material(*rgb)}</visual>"
    if collide:
        s += f"<collision name='{name}_c'>{pose}{geo}</collision>"
    return s


def cyl(name, x, y, z, r, length, rgb, roll=0.0, pitch=0.0, yaw=0.0, collide=True, alpha=1.0):
    pose = f"<pose>{x:.3f} {y:.3f} {z:.3f} {roll:.5f} {pitch:.5f} {yaw:.5f}</pose>"
    geo = f"<geometry><cylinder><radius>{r:.3f}</radius><length>{length:.3f}</length></cylinder></geometry>"
    s = f"<visual name='{name}'>{pose}{geo}{material(*rgb, alpha)}</visual>"
    if collide:
        s += f"<collision name='{name}_c'>{pose}{geo}</collision>"
    return s


def sphere(name, x, y, z, r, rgb, alpha=1.0, collide=True):
    pose = f"<pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>"
    geo = f"<geometry><sphere><radius>{r:.3f}</radius></sphere></geometry>"
    s = f"<visual name='{name}'>{pose}{geo}{material(*rgb, alpha)}</visual>"
    if collide:
        s += f"<collision name='{name}_c'>{pose}{geo}</collision>"
    return s


def strut(name, p0, p1, r, rgb):
    """Cylinder between two 3D points (visual only)."""
    dx, dy, dz = (p1[i] - p0[i] for i in range(3))
    L = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    mid = [(p0[i] + p1[i]) / 2 for i in range(3)]
    return cyl(name, *mid, r, L, rgb, 0, pitch, yaw, collide=False)


STEEL = (0.62, 0.64, 0.66)
WHITE = (0.95, 0.95, 0.95)
CONCRETE = (0.70, 0.69, 0.66)


def lattice_tower(prefix, h, base_w, top_w, leg_r=0.15, brace_r=0.06, section=5.0, legs=4):
    """Tapered lattice tower centred on the model origin; one box collision."""
    s = []
    n_sec = max(1, round(h / section))
    corners = [(math.cos(2 * math.pi * k / legs + math.pi / legs), math.sin(2 * math.pi * k / legs + math.pi / legs))
               for k in range(legs)]
    half = lambda z: (base_w + (top_w - base_w) * z / h) / 2 / math.cos(math.pi / legs)
    pt = lambda k, z: (corners[k][0] * half(z), corners[k][1] * half(z), z)
    for k in range(legs):
        s.append(strut(f"{prefix}_leg{k}", pt(k, 0), pt(k, h), leg_r, STEEL))
    for i in range(n_sec):
        z0, z1 = h * i / n_sec, h * (i + 1) / n_sec
        for k in range(legs):
            k2 = (k + 1) % legs
            s.append(strut(f"{prefix}_h{i}_{k}", pt(k, z1), pt(k2, z1), brace_r, STEEL))
            s.append(strut(f"{prefix}_d{i}_{k}a", pt(k, z0), pt(k2, z1), brace_r, STEEL))
            s.append(strut(f"{prefix}_d{i}_{k}b", pt(k2, z0), pt(k, z1), brace_r, STEEL))
    w = (base_w + top_w) / 2
    s.append(f"<collision name='{prefix}_bbox'><pose>0 0 {h/2:.3f} 0 0 0</pose>"
             f"<geometry><box><size>{w:.2f} {w:.2f} {h:.2f}</size></box></geometry></collision>")
    return "".join(s)


def model(name, x, y, body, comment="", frames=""):
    return (f"\n    <!-- {comment} -->\n" if comment else "\n") + (
        f"    <model name='{name}'><static>true</static><pose>{x:.3f} {y:.3f} 0 0 0 0</pose>"
        f"<link name='link'>{body}</link>{frames}</model>\n")


def wsr88d(name, x, y, tower_h, comment):
    ant_z = tower_h + WSR88D_ANTENNA_ABOVE_TOWER
    body = lattice_tower("tower", tower_h, 9.0, 6.5, leg_r=0.25, brace_r=0.08)
    body += box("platform", 0, 0, tower_h + 0.2, 7.5, 7.5, 0.4, CONCRETE)
    body += sphere("radome", 0, 0, ant_z, WSR88D_RADOME_D / 2, WHITE)
    body += box("shelter", 9.5, 0, 1.5, 4.0, 7.0, 3.0, (0.85, 0.83, 0.78))
    body += box("generator", 9.5, -6.5, 1.2, 3.0, 2.5, 2.4, (0.80, 0.80, 0.80))
    frames = (f"<frame name='antenna_center' attached_to='link'><pose>0 0 {ant_z:.3f} 0 0 0</pose></frame>")
    return model(name, x, y, body, comment, frames)


def atd(name, x, y, comment):
    zc = ATD_RADOME_CENTER_AGL
    r = ATD_RADOME_DIAMETER / 2
    az = math.radians(ATD_ARRAY_AZ_DEG)
    yaw = math.pi / 2 - az                      # compass azimuth -> ENU yaw
    ax, ay = ATD_ARRAY_OFFSET * math.cos(yaw), ATD_ARRAY_OFFSET * math.sin(yaw)
    base_top = zc - math.sqrt(r * r - 3.8 ** 2)  # radome base ring ~7.6 m wide
    body = box("base", 0, 0, base_top / 2, 7.6, 7.6, base_top, CONCRETE)
    body += sphere("radome", 0, 0, zc, r, WHITE)
    body += box("array", ax, ay, zc, 0.35, 4.3, 4.3, (0.25, 0.27, 0.30), yaw=yaw)
    body += cyl("pedestal", 0, 0, (base_top + zc) / 2, 0.8, zc - base_top, STEEL, collide=False)
    body += box("shelter", 0, -9.0, 1.5, 6.0, 4.0, 3.0, (0.85, 0.83, 0.78))
    frames = (f"<frame name='radome_center' attached_to='link'><pose>0 0 {zc:.3f} 0 0 0</pose></frame>"
              f"<frame name='array_center' attached_to='link'><pose>{ax:.3f} {ay:.3f} {zc:.3f} 0 0 {yaw:.5f}</pose></frame>")
    return model(name, x, y, body, comment, frames)


def cal_tower(name, x, y, toward_xy, comment):
    h = CAL_PROBE_AGL
    yaw = math.atan2(toward_xy[1] - y, toward_xy[0] - x)
    body = lattice_tower("tower", h - 0.3, 2.0, 1.2, leg_r=0.08, brace_r=0.04, section=3.0, legs=3)
    body += box("probe_horn", 0.5 * math.cos(yaw), 0.5 * math.sin(yaw), h, 0.6, 0.4, 0.3, (0.85, 0.75, 0.2), yaw=yaw)
    body += box("hut", 3.0, 0, 1.2, 2.5, 2.0, 2.4, (0.85, 0.83, 0.78))
    frames = f"<frame name='probe' attached_to='link'><pose>0 0 {h:.3f} 0 0 {yaw:.5f}</pose></frame>"
    return model(name, x, y, body, comment, frames)


def mast(name, x, y, h):
    body = lattice_tower("mast", h, 1.5, 0.8, leg_r=0.06, brace_r=0.03, section=3.0, legs=3)
    body += box("light", 0, 0, h + 0.2, 0.3, 0.3, 0.4, (0.9, 0.1, 0.1), collide=False)
    return model(name, x, y, body, "generic communication mast (OSM)")


def water_tower(name, x, y, h):
    body = lattice_tower("legs", h - 8, 10.0, 6.0, leg_r=0.4, brace_r=0.12, section=8.0, legs=6)
    body += cyl("riser", 0, 0, (h - 8) / 2, 1.0, h - 8, (0.75, 0.78, 0.82))
    body += sphere("tank", 0, 0, h - 4, 7.5, (0.80, 0.84, 0.88))
    return model(name, x, y, body, "generic water tower (OSM position)")


def control_tower(name, x, y, h):
    body = box("shaft", 0, 0, h / 2, 4.0, 4.0, h, CONCRETE)
    body += box("cab", 0, 0, h + 1.5, 6.0, 6.0, 3.0, (0.25, 0.35, 0.45))
    body += box("cab_roof", 0, 0, h + 3.2, 6.6, 6.6, 0.4, (0.3, 0.3, 0.3))
    return model(name, x, y, body, "generic airport control tower (OSM position)")


def mesh_model(name, mesh_file, collide):
    path = os.path.join(MODELS, name)
    os.makedirs(os.path.join(path, "meshes"), exist_ok=True)
    col = (f"<collision name='collision'><geometry><mesh><uri>meshes/{mesh_file}</uri></mesh></geometry></collision>"
           if collide else "")
    with open(os.path.join(path, "model.sdf"), "w") as f:
        f.write(f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual"><cast_shadows>{'true' if collide else 'false'}</cast_shadows>
        <geometry><mesh><uri>meshes/{mesh_file}</uri></mesh></geometry></visual>
      {col}
    </link>
  </model>
</sdf>
""")
    with open(os.path.join(path, "model.config"), "w") as f:
        f.write(f"""<?xml version="1.0"?>
<model><name>{name}</name><version>1.0</version><sdf version="1.9">model.sdf</sdf>
<description>Generated by make_norman_world.py from OpenStreetMap data (ODbL).</description></model>
""")
    return os.path.join(path, "meshes", mesh_file)


# ------------------------------------------------------------------ main
def main():
    global ORIGIN_LAT, ORIGIN_LON
    # Refine the takeoff origin to exactly (+50 E, +200 N) of the ATD with the exact ENU transform.
    for _ in range(5):
        e, n = ENU(ORIGIN_LAT, ORIGIN_LON)(ATD_LAT, ATD_LON)
        ORIGIN_LAT += (-200.0 - n) / 110950.0
        ORIGIN_LON += (-50.0 - e) / (111320.0 * math.cos(math.radians(ATD_LAT)))
    enu = ENU(ORIGIN_LAT, ORIGIN_LON)
    osm = json.load(open(OSM_FILE))

    pos = {k: enu(lat, lon) for k, (lat, lon, _) in RADARS.items()}
    keep_clear = [(x, y, 25.0) for x, y in pos.values()] + [(0.0, 0.0, 40.0)]

    buildings, ground, trees = Obj(), Obj(), Obj()
    n_bld = n_road = 0
    building_polys = []
    tree_pts = []
    for el in osm["elements"]:
        t = el.get("tags", {})
        if el["type"] == "node":
            if t.get("natural") == "tree":
                tree_pts.append(enu(el["lat"], el["lon"]))
            continue
        if "geometry" not in el:
            continue
        pts = [enu(p["lat"], p["lon"]) for p in el["geometry"]]
        if "building" in t:
            btype = t["building"] if t["building"] != "yes" else t.get("aeroway", "commercial")
            try:
                h = float(t.get("height", "0").split()[0])
            except (ValueError, IndexError):
                h = 0
            if not h and t.get("building:levels"):
                try:
                    h = 3.2 * float(t["building:levels"]) + 1.0
                except ValueError:
                    h = 0
            h = h or BUILDING_HEIGHT.get(btype, 6.0)
            mat = BUILDING_MATERIAL.get(btype, "commercial")
            buildings.prism(mat, "roof", pts, 0.0, h)
            building_polys.append(pts)
            n_bld += 1
        elif t.get("aeroway") in AEROWAY_WIDTH:
            kind = "runway" if t["aeroway"] in ("runway", "stopway") else "taxiway"
            ground.ribbon(kind, pts, AEROWAY_WIDTH[t["aeroway"]], 0.05 if kind == "runway" else 0.04)
        elif t.get("aeroway") in ("apron", "helipad"):
            ground.flat_poly("apron", pts, 0.035)
        elif t.get("highway") in ROAD_WIDTH:
            major = t["highway"] in ("motorway", "trunk", "primary", "secondary")
            ground.ribbon("road_major" if major else "road_minor", pts, ROAD_WIDTH[t["highway"]],
                          0.03 if major else 0.025)
            n_road += 1
        elif t.get("natural") == "tree_row":
            for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
                L = math.hypot(x1 - x0, y1 - y0)
                for k in range(int(L // 8) + 1):
                    tree_pts.append((x0 + (x1 - x0) * k * 8 / max(L, 1), y0 + (y1 - y0) * k * 8 / max(L, 1)))

    # A few synthetic tree clusters around the radar compound (generic, not surveyed).
    rng = random.Random(2026)
    clusters = [(-170, 120), (-140, -60), (230, 40), (120, 330), (-90, 330), (380, 120), (300, -140)]
    for cx, cy in clusters:
        for _ in range(7):
            tree_pts.append((cx + rng.uniform(-25, 25), cy + rng.uniform(-25, 25)))

    def blocked(x, y):
        if any(math.hypot(x - a, y - b) < r for a, b, r in keep_clear):
            return True
        for poly in building_polys:     # point-in-polygon (skip trees inside buildings)
            xs = [p[0] for p in poly]
            ys = [p[1] for p in poly]
            if not (min(xs) - 3 < x < max(xs) + 3 and min(ys) - 3 < y < max(ys) + 3):
                continue
            inside = False
            for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
                if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
                    inside = not inside
            if inside:
                return True
        return False

    tree_cols = []
    for x, y in tree_pts:
        if math.hypot(x, y) > 2600 or blocked(x, y):
            continue
        h = rng.uniform(7, 14)
        cr = rng.uniform(2.2, 4.0)
        trees.cylinder("trunk", x, y, 0.25, 0.0, h * 0.45, sides=6)
        trees.blob("canopy" if rng.random() < 0.6 else "canopy_dark", x, y, h * 0.62, cr, h * 0.38)
        tree_cols.append((x, y, cr, h))

    buildings.write(mesh_model("cs_norman_buildings", "buildings.obj", True), "cs_norman_buildings")
    ground.write(mesh_model("cs_norman_ground", "ground.obj", False), "cs_norman_ground")
    trees.write(mesh_model("cs_norman_trees", "trees.obj", False), "cs_norman_trees")
    # Tree collisions as cheap cylinders.
    tsdf = os.path.join(MODELS, "cs_norman_trees", "model.sdf")
    cols = "".join(f"<collision name='t{i}'><pose>{x:.2f} {y:.2f} {h/2:.2f} 0 0 0</pose><geometry><cylinder>"
                   f"<radius>{cr*0.8:.2f}</radius><length>{h:.2f}</length></cylinder></geometry></collision>"
                   for i, (x, y, cr, h) in enumerate(tree_cols))
    s = open(tsdf).read().replace("\n    </link>", cols + "\n    </link>")
    open(tsdf, "w").write(s)

    # Radars and structures (inline models with named frames for RF calculations).
    radar_sdf = wsr88d("KCRI", *pos["KCRI"], 30.0, RADARS["KCRI"][2])
    radar_sdf += wsr88d("KOUN", *pos["KOUN"], 20.0, RADARS["KOUN"][2])
    radar_sdf += atd("ATD", *pos["ATD"], RADARS["ATD"][2] + f"; radome size assumed equal to WSR-88D ({ATD_RADOME_DIAMETER} m)")
    radar_sdf += cal_tower("ATD_cal_tower", *pos["ATD_cal_tower"], pos["ATD"], RADARS["ATD_cal_tower"][2])
    for name, lat, lon, kind, h in OTHER_STRUCTURES:
        x, y = enu(lat, lon)
        radar_sdf += {"mast": mast, "water_tower": water_tower, "control_tower": control_tower}[kind](name, x, y, h)

    world = f"""<?xml version="1.0" ?>
<!--
  CopterSonde at the Max Westheimer Airport radar site, Norman, Oklahoma (2 km around KCRI).
  Generated by CopterSonde-sim/tools/make_norman_world.py - edit the script, not this file.
  World origin / drone takeoff: lat {ORIGIN_LAT:.9f}, lon {ORIGIN_LON:.9f}, {GROUND_MSL} m MSL
  (50 m E, 200 m N of the ATD radome), drone heading {TAKEOFF_HEADING_DEG:g} deg. The drone starts on the
  Skycharge BOLOGNINI S1 charging pad, on the launch deck, {DECK_H + PAD_TOP:g} m above the ground.
  Frame: x = East, y = North, z = Up (metres).
  The air (temperature, humidity, wind, turbulence) comes from the coptersonde model's
  CopterSondePlugin and the table in CopterSonde-sim/atmosphere.
-->
<sdf version="1.9">
  <world name="{WORLD_NAME}">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.70 0.80 0.92 1</background>
      <sky></sky>
      <grid>false</grid>
    </scene>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{ORIGIN_LAT:.9f}</latitude_deg>
      <longitude_deg>{ORIGIN_LON:.9f}</longitude_deg>
      <elevation>{GROUND_MSL}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.9 0.9 0.88 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.4 0.3 -0.85</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>6000 6000</size></plane></geometry></collision>
        <visual name="visual"><cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>6000 6000</size></plane></geometry>
          <material><ambient>0.42 0.52 0.30 1</ambient><diffuse>0.42 0.52 0.30 1</diffuse><specular>0 0 0 1</specular></material>
        </visual>
      </link>
    </model>

    <include><uri>model://cs_norman_ground</uri><pose>0 0 0 0 0 0</pose></include>
    <include><uri>model://cs_norman_buildings</uri><pose>0 0 0 0 0 0</pose></include>
    <include><uri>model://cs_norman_trees</uri><pose>0 0 0 0 0 0</pose></include>
{radar_sdf}
    <!-- launch deck, Skycharge BOLOGNINI S1 at the take-off point and the CopterSonde resting on it -->
    <include><uri>model://cs_launch_pad</uri><pose degrees="true">0 0 0 0 0 {90.0 - TAKEOFF_HEADING_DEG:g}</pose></include>
    <include>
      <uri>model://cs_skycharge_bolognini</uri>
      <name>skycharge_bolognini</name>
      <pose degrees="true">0 0 {DECK_H:g} 0 0 {90.0 - TAKEOFF_HEADING_DEG:g}</pose>
    </include>
    <include>
      <uri>model://coptersonde</uri>
      <pose degrees="true">0 0 {DECK_H + PAD_TOP + 0.195:.3f} 0 0 {90.0 - TAKEOFF_HEADING_DEG:g}</pose>  <!-- heading {TAKEOFF_HEADING_DEG:g} deg -->
    </include>
  </world>
</sdf>
"""
    os.makedirs(WORLDS, exist_ok=True)
    with open(os.path.join(WORLDS, f"{WORLD_NAME}.sdf"), "w") as f:
        f.write(world)
    make_pad_models.launch_pad()
    make_pad_models.bolognini()

    print(f"origin (takeoff): {ORIGIN_LAT:.9f}, {ORIGIN_LON:.9f}, {GROUND_MSL} m")
    for k, (x, y) in pos.items():
        print(f"  {k:14s} E {x:8.1f}  N {y:8.1f}  range {math.hypot(x, y):6.1f} m")
    print(f"buildings {n_bld}, road segments {n_road}, trees {len(tree_cols)}")
    print(f"SITL location (Tools/autotest/locations.txt): CopterSonde_Norman="
          f"{ORIGIN_LAT:.7f},{ORIGIN_LON:.7f},{GROUND_MSL},{TAKEOFF_HEADING_DEG:g}")


if __name__ == "__main__":
    main()
