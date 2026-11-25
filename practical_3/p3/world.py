"""Practical 3 inspection site (simulator side only)."""
from __future__ import annotations

import math

import pybullet as p

from worlds.earthquake_world import _Tex, _box, _cyl

MARKER_RGBA = {
    "orange": (1.0, 0.45, 0.0, 1),
    "yellow": (1.0, 0.92, 0.0, 1),
    "cyan": (0.0, 0.9, 0.95, 1),
    "magenta": (0.95, 0.0, 0.85, 1),
}


class InspectionWorld:
    def __init__(self, pb, cfg):
        self.pb = pb
        self.off = cfg["mission_frame"]["world_from_mission_xy"]
        self.bodies = {}
        self.obstacles = []
        self._build(cfg)

    def w(self, x, y, z=0.0):
        return (x + self.off[0], y + self.off[1], z)

    def _build(self, cfg):
        pb = self.pb
        tex = _Tex(pb)
        ground = _box(pb, self.w(15, 10, -0.05), (30, 25, 0.05), rgba=(1, 1, 1, 1))
        tex.apply(ground, "ground")
        p.changeDynamics(ground, -1, lateralFriction=1.0, physicsClientId=pb)
        self.ground = ground

        x0, x1 = cfg["area"]["x_m"]
        y0, y1 = cfg["area"]["y_m"]
        red = (0.9, 0.1, 0.1, 1)
        for (ax, ay, bx, by) in [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]:
            cx, cy, L = (ax + bx) / 2, (ay + by) / 2, math.hypot(bx - ax, by - ay)
            yaw = math.atan2(by - ay, bx - ax)
            _box(pb, self.w(cx, cy, 0.01), (L / 2, 0.08, 0.01), p.getQuaternionFromEuler([0, 0, yaw]), red)
        for k in range(0, 41, 4):
            for (px, py) in [(x0 + k, y0), (x0 + k, y1)]:
                _cyl(pb, self.w(px, py, 0.4), 0.05, 0.8, rgba=(0.92, 0.92, 0.92, 1))

        bs = tex
        b = _box(pb, self.w(0, -1.2, 1.1), (1.6, 1.0, 1.1), rgba=(1, 1, 1, 1)); bs.apply(b, "concrete")
        _box(pb, self.w(0, -1.2, 2.25), (1.7, 1.1, 0.05), rgba=(0.25, 0.3, 0.35, 1))
        _cyl(pb, self.w(-1.0, -1.6, 3.8), 0.06, 3.0, rgba=(0.7, 0.7, 0.75, 1))
        _box(pb, self.w(-1.0, -1.6, 5.3), (0.03, 0.35, 0.25), rgba=(0.9, 0.9, 0.95, 1))
        self.obstacles.append(("base_station", 0, -1.2, 1.8))
        _box(pb, self.w(3, 2, 0.005), (0.9, 0.9, 0.005), rgba=(0.2, 0.2, 0.2, 1))
        _box(pb, self.w(3, 2, 0.011), (0.6, 0.08, 0.002), rgba=(1, 1, 1, 1))
        _box(pb, self.w(3, -2.5, 0.005), (0.7, 0.6, 0.005), rgba=(0.15, 0.3, 0.6, 1))

        tx, ty = 22.0, 16.0
        for dx, dy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]:
            leg = _box(pb, self.w(tx + dx, ty + dy, 2.5), (0.12, 0.12, 2.5), rgba=(1, 1, 1, 1)); tex.apply(leg, "rust")
        for z in (1.5, 3.0):
            for dx, dy, hx, hy in [(0, -1, 1, 0.06), (0, 1, 1, 0.06), (-1, 0, 0.06, 1), (1, 0, 0.06, 1)]:
                _box(pb, self.w(tx + dx, ty + dy, z), (hx, hy, 0.06), rgba=(0.45, 0.3, 0.2, 1))
        deck = _box(pb, self.w(tx, ty, 4.4), (1.3, 1.3, 0.1), rgba=(1, 1, 1, 1)); tex.apply(deck, "concrete")
        for dx, dy, hx, hy in [(0, -1.32, 0.9, 0.02), (0, 1.32, 0.9, 0.02), (-1.32, 0, 0.02, 0.9), (1.32, 0, 0.02, 0.9)]:
            _box(pb, self.w(tx + dx, ty + dy, 5.0), (hx, hy, 0.5), rgba=MARKER_RGBA["orange"])
        _cyl(pb, self.w(tx, ty, 6.0), 0.05, 1.6, rgba=(0.6, 0.6, 0.6, 1))
        self.obstacles.append(("tower", tx, ty, 1.6))

        ex, ey = 14.0, 5.0
        c = _box(pb, self.w(ex, ey, 0.7), (0.6, 0.4, 0.7), rgba=(0.35, 0.4, 0.38, 1))
        _box(pb, self.w(ex, ey, 1.42), (0.62, 0.42, 0.02), rgba=MARKER_RGBA["yellow"])
        _box(pb, self.w(ex - 0.61, ey, 0.9), (0.01, 0.3, 0.3), rgba=MARKER_RGBA["yellow"])
        _box(pb, self.w(ex, ey - 0.41, 0.9), (0.3, 0.01, 0.3), rgba=MARKER_RGBA["yellow"])
        _cyl(pb, self.w(ex + 1.2, ey + 0.6, 0.45), 0.3, 0.9, rgba=(0.3, 0.32, 0.3, 1))
        self.obstacles.append(("equipment", ex, ey, 1.1))

        mx, my = 9.0, 13.0
        _box(pb, self.w(mx, my, 0.03), (1.0, 1.0, 0.03), rgba=(0.55, 0.55, 0.55, 1))
        _cyl(pb, self.w(mx, my, 1.0), 0.05, 2.0, rgba=(0.8, 0.8, 0.8, 1))
        _box(pb, self.w(mx, my, 1.0), (0.22, 0.22, 0.22), rgba=MARKER_RGBA["cyan"])
        _box(pb, self.w(mx, my, 2.05), (0.3, 0.3, 0.02), rgba=MARKER_RGBA["cyan"])
        self.obstacles.append(("env_station", mx, my, 0.5))

        wall_x = 27.0
        for ya, yb in [(-5.0, 3.0), (5.0, 25.0)]:
            wl = _box(pb, self.w(wall_x, (ya + yb) / 2, 1.2), (0.15, (yb - ya) / 2, 1.2), rgba=(1, 1, 1, 1)); tex.apply(wl, "brick")
        self.passage = (wall_x, 4.0, 2.0)

        px, py = 32.0, 4.0
        _box(pb, self.w(px, py, 0.01), (0.9, 0.9, 0.01), rgba=MARKER_RGBA["magenta"])
        for k, (dx, dy) in enumerate([(1.4, 0.8), (1.2, -1.1), (-1.0, 1.4)]):
            rb = _box(pb, self.w(px + dx, py + dy, 0.25), (0.3, 0.25, 0.25), p.getQuaternionFromEuler([0, 0, 0.6 * k]), (1, 1, 1, 1))
            tex.apply(rb, "concrete")
        _box(pb, self.w(px, py, 0.3), (0.12, 0.12, 0.3), rgba=MARKER_RGBA["magenta"])

        for (bx, by) in [(8.0, 1.0), (9.5, -2.5), (11.0, 2.2), (18.5, 2.0), (20.0, 6.5), (23.5, 3.0), (6.0, 9.0), (12.5, 10.5)]:
            o = _cyl(pb, self.w(bx, by, 0.45), 0.3, 0.9, rgba=(1, 1, 1, 1)); tex.apply(o, "rust")
            self.obstacles.append(("barrel", bx, by, 0.3))
        for (bx, by, yaw) in [(17.0, 9.0, 0.3), (25.0, 9.0, -0.4), (4.0, 17.0, 0.0)]:
            o = _box(pb, self.w(bx, by, 0.4), (0.8, 0.5, 0.4), p.getQuaternionFromEuler([0, 0, yaw]), (1, 1, 1, 1)); tex.apply(o, "wood")
            self.obstacles.append(("crate", bx, by, 0.9))
