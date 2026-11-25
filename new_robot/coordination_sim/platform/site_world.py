"""Outdoor inspection site for the combined experiment (SIMULATION SIDE ONLY - never imported by autonomy code)."""
from __future__ import annotations

import math
import random

import numpy as np
import pybullet as p
import pybullet_data

from worlds.earthquake_world import _Tex, _box, _cyl, _sphere
from worlds.humans import build_human


def _paint(pb, a, b, width, rgba, z=0.004):
    """Visual-only painted line on the ground from a to b (no collision)."""
    a, b = np.array(a, float), np.array(b, float)
    d = b - a
    L = float(np.linalg.norm(d))
    if L < 1e-6:
        return None
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[L / 2, width / 2, 0.002], rgbaColor=rgba, physicsClientId=pb)
    q = p.getQuaternionFromEuler([0, 0, math.atan2(d[1], d[0])])
    c = (a + b) / 2
    return p.createMultiBody(0, -1, vis, [c[0], c[1], z], q, physicsClientId=pb)


def build_site_world(pb, mcfg, uav_cfg):
    seed = int(mcfg.get("seed", 4))
    rnd = random.Random(seed)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=pb)
    tex = _Tex(pb)
    site, W = mcfg["site"], mcfg["world"]
    thermal, obstacle_ids = {}, {}

    def add(body, tname, name=None):
        tex.apply(body, tname)
        if name:
            obstacle_ids[body] = name
        return body

    ground = _box(pb, (0, 0, -0.05), (32, 32, 0.05), rgba=(1, 1, 1, 1))
    tex.apply(ground, "ground")
    p.changeDynamics(ground, -1, lateralFriction=0.9, physicsClientId=pb)

    poly = [tuple(v) for v in site["boundary_xy"]]
    for i in range(len(poly)):
        a, b = np.array(poly[i]), np.array(poly[(i + 1) % len(poly)])
        n = int(np.ceil(np.linalg.norm(b - a) / 1.0))
        for k in range(n):
            pa, pb_ = a + (b - a) * k / n, a + (b - a) * (k + 1) / n
            _paint(pb, pa, pb_, 0.18, (0.85, 0.08, 0.08, 1) if k % 2 == 0 else (0.95, 0.95, 0.95, 1))
    for (x, y) in poly:
        _cyl(pb, (x, y, 0.6), 0.10, 1.2, rgba=(0.9, 0.15, 0.1, 1))
    for z in site["zones"]:
        cx, cy = z["center"]
        hx, hy = z["half"]
        cs = [(cx - hx, cy - hy), (cx + hx, cy - hy), (cx + hx, cy + hy), (cx - hx, cy + hy)]
        for i in range(4):
            _paint(pb, cs[i], cs[(i + 1) % 4], 0.08, (0.95, 0.80, 0.10, 1), z=0.003)

    bx, by = site["base_station_xy"]
    add(_box(pb, (bx, by, 0.6), (0.8, 0.6, 0.6), rgba=(1, 1, 1, 1)), "concrete", "base_station_shelter")
    _box(pb, (bx, by, 1.23), (0.85, 0.65, 0.03), rgba=(0.25, 0.3, 0.35, 1))
    _cyl(pb, (bx + 0.5, by + 0.3, 2.45), 0.05, 2.5, rgba=(0.75, 0.75, 0.78, 1))
    for h in (2.2, 2.9, 3.5):
        _box(pb, (bx + 0.5, by + 0.3, h), (0.35, 0.02, 0.02), rgba=(0.75, 0.75, 0.78, 1))
    _sphere(pb, (bx + 0.5, by + 0.3, 3.75), 0.09, (0.9, 0.1, 0.1, 1))
    vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.35, length=0.05, rgbaColor=[0.9, 0.9, 0.92, 1], physicsClientId=pb)
    p.createMultiBody(0, -1, vis, [bx + 0.5, by - 0.05, 3.0], p.getQuaternionFromEuler([math.pi / 2, 0, 0]), physicsClientId=pb)

    ux, uy = site["uav_pad"]["xy"]
    for a, b in [((ux - 0.8, uy - 0.8), (ux + 0.8, uy - 0.8)), ((ux + 0.8, uy - 0.8), (ux + 0.8, uy + 0.8)),
                 ((ux + 0.8, uy + 0.8), (ux - 0.8, uy + 0.8)), ((ux - 0.8, uy + 0.8), (ux - 0.8, uy - 0.8)),
                 ((ux - 0.35, uy - 0.45), (ux - 0.35, uy + 0.45)), ((ux + 0.35, uy - 0.45), (ux + 0.35, uy + 0.45)), ((ux - 0.35, uy), (ux + 0.35, uy))]:
        _paint(pb, a, b, 0.07, (0.95, 0.95, 0.95, 1))
    gx, gy = site["ugv_pad"]["xy"]
    for a, b in [((gx - 0.5, gy - 0.6), (gx + 0.5, gy - 0.6)), ((gx + 0.5, gy - 0.6), (gx + 0.5, gy + 0.6)),
                 ((gx + 0.5, gy + 0.6), (gx - 0.5, gy + 0.6)), ((gx - 0.5, gy + 0.6), (gx - 0.5, gy - 0.6))]:
        _paint(pb, a, b, 0.06, (0.2, 0.45, 0.95, 1))

    add(_box(pb, (1.15, 3.0, 1.3), (1.6, 0.6, 1.3), rgba=(1, 1, 1, 1)), "rust", "container_west")
    add(_box(pb, (5.65, 3.0, 1.3), (1.6, 0.6, 1.3), rgba=(1, 1, 1, 1)), "rust", "container_east")

    add(_box(pb, (6.4, 9.4, 1.3), (1.4, 1.4, 1.3), rgba=(1, 1, 1, 1)), "brick", "pump_house")
    add(_box(pb, (6.4, 9.4, 2.65), (1.55, 1.55, 0.06), rgba=(1, 1, 1, 1)), "concrete")
    vx, vy = W["valve_skid"]["xy"]
    vh = W["valve_skid"]["half"]
    add(_box(pb, (vx, vy, vh[2]), vh, rgba=(1, 1, 1, 1)), "rust", "valve_skid")
    _cyl(pb, (vx + 0.25, vy, 0.95), 0.13, 0.5, rgba=(0.9, 0.75, 0.1, 1))
    _cyl(pb, ((vx + 5.0) / 2, vy, 0.45), 0.07, 5.0 - vx, p.getQuaternionFromEuler([0, math.pi / 2, 0]), (0.55, 0.55, 0.6, 1))

    add(_box(pb, (-6.8, 11.5, 1.8), (2.4, 2.2, 1.8), rgba=(1, 1, 1, 1)), "concrete", "warehouse")
    add(_box(pb, (-4.35, 11.0, 1.0), (0.05, 0.9, 1.0), rgba=(1, 1, 1, 1)), "wood")

    gx_, gy_ = W["gate"]["xy"]
    for dx in (-0.9, 0.9):
        add(_cyl(pb, (gx_ + dx, gy_, 0.9), 0.08, 1.8, rgba=(1, 1, 1, 1)), "rust", "gate_post")
    _box(pb, (gx_, gy_, 1.78), (0.95, 0.04, 0.04), rgba=(0.85, 0.7, 0.1, 1))
    add(_box(pb, (gx_ - 1.85, gy_, 0.6), (0.9, 0.03, 0.6), rgba=(1, 1, 1, 1)), "wood", "fence_w")
    add(_box(pb, (gx_ + 1.85, gy_, 0.6), (0.9, 0.03, 0.6), rgba=(1, 1, 1, 1)), "wood", "fence_e")

    hx, hy = W["hot_spot"]["xy"]
    r = W["hot_spot"]["radius_m"]
    drum = _cyl(pb, (hx, hy, 0.45), 0.55, 0.9, p.getQuaternionFromEuler([math.pi / 2, 0, 0.4]), (0.15, 0.12, 0.10, 1))
    thermal[drum] = 160.0
    obstacle_ids[drum] = "cable_drum"
    for (dx, dy, h, rad, col) in [(0, 0, 1.0, 0.45, (1.0, 0.35, 0.02, 1)), (0.25, 0.15, 1.3, 0.3, (1.0, 0.6, 0.05, 1)),
                                  (-0.2, -0.1, 1.15, 0.32, (1.0, 0.45, 0.03, 1))]:
        vis = p.createVisualShape(p.GEOM_SPHERE, radius=rad * r, rgbaColor=list(col), physicsClientId=pb)
        fb = p.createMultiBody(0, -1, vis, (hx + dx, hy + dy, h * r), physicsClientId=pb)
        thermal[fb] = float(W["hot_spot"]["temp_c"])

    cas = W["casualty"]
    hb = build_human(pb, cas["xy"], cas.get("posture", "standing"), cas.get("yaw_deg", 0.0), 1)
    for b_, tc in hb["parts"]:
        thermal[b_] = tc

    for (x, y) in [(-10.0, 6.0), (10.5, 3.0), (-10.5, 18.0), (10.8, 22.5), (-1.0, 23.0)]:
        h = rnd.uniform(3.0, 4.5)
        add(_cyl(pb, (x, y, h / 2), 0.18, h, rgba=(1, 1, 1, 1)), "wood")
        _sphere(pb, (x, y, h + 0.5), rnd.uniform(1.0, 1.4), (rnd.uniform(0.1, 0.25), rnd.uniform(0.35, 0.55), 0.12, 1))
    for (x, y) in [(-7.5, 3.0), (9.6, 12.5), (-8.8, 20.0), (8.0, -4.5)]:
        add(_sphere(pb, (x, y, 0.35), rnd.uniform(0.45, 0.6), (1, 1, 1, 1)), "concrete")
    for (x, y) in [(-0.2, 6.8), (7.9, 6.0), (-5.0, 6.5), (9.8, 15.5)]:
        add(_cyl(pb, (x, y, 0.45), 0.3, 0.9, rgba=(1, 1, 1, 1)), "rust", "barrel")
    for (x, y, yaw) in [(-9.0, 14.5, 0.2), (10.0, 9.0, 1.4)]:
        add(_box(pb, (x, y, 0.4), (0.6, 0.5, 0.4), p.getQuaternionFromEuler([0, 0, yaw]), (1, 1, 1, 1)), "wood")

    return {
        "humans": [{"id": "C1", "xy_enu": list(cas["xy"]), "posture": cas.get("posture", "standing")}],
        "hazards": [{"id": "H1", "kind": "hot_spot", "xy_enu": [hx, hy], "radius_m": r, "temp_c": W["hot_spot"]["temp_c"]}],
        "thermal_c": thermal,
        "ground_body": ground,
        "obstacle_ids": obstacle_ids,
        "scenario": "static",
        "dynamic": None,
        "targets_gt": {"valve_skid": [vx, vy], "casualty": list(cas["xy"]), "gate": [gx_, gy_], "hot_spot": [hx, hy]},
        "description": "Outdoor inspection site: boundary, base station, containers (narrow passage), pump house + valve skid, "
                       "warehouse, casualty, perimeter gate, overheating cable drum.",
    }
