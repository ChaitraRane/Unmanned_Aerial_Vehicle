"""UGV vehicle side in the SHARED PyBullet world (simulation side only)."""
from __future__ import annotations

import math

import numpy as np
import pybullet as p

import stage2_pointcloud as S2
from viwo_sensors import SimulatedSensorSuite, read_robot_kinematics

from coordination_sim.bootstrap import UGV_URDF


class UgvBattery:
    def __init__(self, cfg):
        self.cap_j = cfg["capacity_wh"] * 3600.0
        self.e = self.cap_j
        self.idle_w = cfg["idle_w"]
        self.k = cfg["drive_w_per_mps"]

    def step(self, speed_mps, dt):
        self.e = max(0.0, self.e - (self.idle_w + self.k * abs(speed_mps)) * dt)

    @property
    def pct(self):
        return 100.0 * self.e / self.cap_j


class UgvPlatform:
    def __init__(self, pb, pad_xy, yaw_enu_rad, bat_cfg, seed):
        self.pb = pb
        orn = p.getQuaternionFromEuler([0, 0, yaw_enu_rad])
        self.robot = p.loadURDF(UGV_URDF, [pad_xy[0], pad_xy[1], 0.16], orn, useFixedBase=False, physicsClientId=pb)
        self.left, self.right = S2.find_wheel_joints(self.robot)
        self.kin = read_robot_kinematics(self.robot, self.left, self.right)
        self.sensors = SimulatedSensorSuite(self.robot, self.left, self.right, seed=seed)
        self.battery = UgvBattery(bat_cfg)
        self.cmd = (0.0, 0.0)
        self.intrinsics, self.camera_matrix = S2.calculate_camera_intrinsics()

    def apply(self, left_rad_s, right_rad_s):
        self.cmd = (float(left_rad_s), float(right_rad_s))
        S2.drive_robot(self.robot, self.left, self.right, self.cmd[0], self.cmd[1])

    def capture(self):
        bgr, depth, _cam_pose = S2.capture_camera(self.robot)
        return bgr, depth

    def sample(self, t):
        return self.sensors.sample_imu(t), self.sensors.sample_wheels(t)

    def step_battery(self, dt):
        v = 0.5 * (self.cmd[0] + self.cmd[1]) * self.kin.wheel_radius_m
        self.battery.step(v, dt)

    def truth_xy_yaw(self):
        pos, q = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.pb)
        return float(pos[0]), float(pos[1]), float(p.getEulerFromQuaternion(q)[2])

    def contacts(self):
        return p.getContactPoints(bodyA=self.robot, physicsClientId=self.pb)
