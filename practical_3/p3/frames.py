"""Shared mission frame (ENU, origin = base station) and the per-agent registrations."""
from __future__ import annotations

import math

import numpy as np

MISSION = "mission_enu"
UAV_LOCAL = "uav_local_ned"
UGV_ODOM = "ugv_odom"


def _rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class Transform:
    def __init__(self, parent, child, R, t, source):
        self.parent, self.child = parent, child
        self.R, self.t = np.asarray(R, float), np.asarray(t, float)
        self.source = source

    def apply(self, p):
        return self.R @ np.asarray(p, float) + self.t

    def inverse(self):
        return Transform(self.child, self.parent, self.R.T, -self.R.T @ self.t, self.source)

    def describe(self):
        yaw = math.degrees(math.atan2(self.R[1, 0], self.R[0, 0]))
        return {"parent": self.parent, "child": self.child, "t": [round(float(v), 3) for v in self.t],
                "yaw_deg": round(yaw, 2), "R": np.round(self.R, 4).tolist(), "source": self.source}


R_ENU_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


def register_uav(pad_mission_xyz, p0_ned):
    """mission <- uav_local_ned, from the surveyed pad and the FC position reported on the pad before take-off."""
    t = np.asarray(pad_mission_xyz, float) - R_ENU_NED @ np.asarray(p0_ned, float)
    return Transform(MISSION, UAV_LOCAL, R_ENU_NED, t, "surveyed take-off pad + FC LOCAL_POSITION_NED at arming")


def register_ugv(dock_xy, dock_heading_deg):
    """mission <- ugv_odom; odom origin is the dock, x along the docked heading."""
    return Transform(MISSION, UGV_ODOM, _rz(math.radians(dock_heading_deg)), [dock_xy[0], dock_xy[1], 0.0],
                     "surveyed UGV dock pose")


class FrameTree:
    def __init__(self):
        self.tf = {}

    def add(self, T):
        self.tf[(T.parent, T.child)] = T
        self.tf[(T.child, T.parent)] = T.inverse()

    def convert(self, p, src, dst):
        if src == dst:
            return np.asarray(p, float)
        if (dst, src) in self.tf:
            return self.tf[(dst, src)].apply(p)
        return self.tf[(dst, MISSION)].apply(self.tf[(MISSION, src)].apply(p))
