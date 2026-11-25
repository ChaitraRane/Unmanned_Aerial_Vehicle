"""Frame registration between the vehicles' own estimation frames and the common site frame."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

UGV_CAMERA_FORWARD_OFFSET_M = 0.25


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class UavRegistration:
    pad_xy: tuple

    def to_site(self, x, y):
        return (float(x) + self.pad_xy[0], float(y) + self.pad_xy[1])

    def from_site(self, x, y):
        return (float(x) - self.pad_xy[0], float(y) - self.pad_xy[1])

    def yaw_to_site(self, yaw_local):
        return wrap(float(yaw_local))


@dataclass(frozen=True)
class UgvRegistration:
    pad_xy: tuple
    yaw0: float

    @property
    def f(self):
        return np.array([math.cos(self.yaw0), math.sin(self.yaw0)])

    @property
    def r(self):
        return np.array([math.sin(self.yaw0), -math.cos(self.yaw0)])

    @property
    def cam0(self):
        return np.array(self.pad_xy, float) + UGV_CAMERA_FORWARD_OFFSET_M * self.f

    def to_site(self, X, Z):
        """UGV map (X right, Z forward) -> site (x, y)."""
        v = self.cam0 + float(X) * self.r + float(Z) * self.f
        return (float(v[0]), float(v[1]))

    def from_site(self, x, y):
        d = np.array([x, y], float) - self.cam0
        return (float(d @ self.r), float(d @ self.f))

    def heading_to_site(self, map_heading):
        """Stage-8 heading (atan2(fx, fz): 0 = map +Z, positive = toward +X / right) -> ENU yaw."""
        return wrap(self.yaw0 - float(map_heading))

    def site_bearing_to_map_heading(self, yaw_site):
        return wrap(self.yaw0 - float(yaw_site))


def point_in_polygon(x, y, poly):
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xin = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xin:
                inside = not inside
    return inside
