"""UGV agent: wheel-inertial odometry, depth occupancy mapping, A* (reused planning/astar.py), guarded path following,
ground inspection actions. The simulator side (body, wheels, sensor rendering) is in UgvVehicle."""
from __future__ import annotations

import math
import os

import cv2
import numpy as np
import pybullet as p

from planning.astar import AStarPlanner, FREE, OCC
from p3.frames import MISSION, UGV_ODOM
from p3.perception import marker_mask

UGV_URDF = os.path.join(os.path.dirname(__file__), "..", "..", "robot_vision", "robot_vision", "simulation", "simple_robot.urdf")
WHEEL_R, TRACK = 0.08, 0.38
CAM_POS_BODY = np.array([0.30, 0.0, 0.18])
CAM_W, CAM_H, CAM_VFOV, CAM_NEAR, CAM_FAR = 320, 240, 60.0, 0.15, 10.0


class UgvVehicle:
    """Simulator side only: PyBullet body, motors, encoder/gyro/camera sensor models."""

    def __init__(self, pb, world_xy, yaw, seed=5):
        self.pb = pb
        q = p.getQuaternionFromEuler([0, 0, yaw])
        self.body = p.loadURDF(os.path.abspath(UGV_URDF), [world_xy[0], world_xy[1], 0.13], q, physicsClientId=pb)
        names = {p.getJointInfo(self.body, j, physicsClientId=pb)[1].decode(): j for j in range(p.getNumJoints(self.body, physicsClientId=pb))}
        self.left = [names["left_front_wheel_joint"], names["left_rear_wheel_joint"]]
        self.right = [names["right_front_wheel_joint"], names["right_rear_wheel_joint"]]
        for j in self.left + self.right:
            p.changeDynamics(self.body, j, lateralFriction=1.2, physicsClientId=pb)
        self.rng = np.random.default_rng(seed)
        self.gyro_bias = 0.004
        f = (CAM_H / 2) / math.tan(math.radians(CAM_VFOV) / 2)
        self.K = (f, f, CAM_W / 2, CAM_H / 2)
        self.proj = p.computeProjectionMatrixFOV(CAM_VFOV, CAM_W / CAM_H, CAM_NEAR, CAM_FAR)

    def command(self, wl, wr):
        for j in self.left:
            p.setJointMotorControl2(self.body, j, p.VELOCITY_CONTROL, targetVelocity=wl, force=20, physicsClientId=self.pb)
        for j in self.right:
            p.setJointMotorControl2(self.body, j, p.VELOCITY_CONTROL, targetVelocity=wr, force=20, physicsClientId=self.pb)

    def encoders(self):
        js = p.getJointStates(self.body, self.left + self.right, physicsClientId=self.pb)
        w = np.array([s[1] for s in js]) + self.rng.normal(0, 0.05, 4)
        return float(np.mean(w[:2])), float(np.mean(w[2:]))

    def gyro_z(self):
        _, ang = p.getBaseVelocity(self.body, physicsClientId=self.pb)
        _, orn = p.getBasePositionAndOrientation(self.body, physicsClientId=self.pb)
        R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        return float((R.T @ np.array(ang))[2] + self.gyro_bias + self.rng.normal(0, 0.003))

    def truth(self):
        pos, orn = p.getBasePositionAndOrientation(self.body, physicsClientId=self.pb)
        return np.array(pos), p.getEulerFromQuaternion(orn)[2]

    def capture(self):
        pos, orn = p.getBasePositionAndOrientation(self.body, physicsClientId=self.pb)
        R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        eye = np.array(pos) + R @ CAM_POS_BODY
        fwd = R @ np.array([1.0, 0, -0.08])
        view = p.computeViewMatrix(eye.tolist(), (eye + fwd).tolist(), (R @ np.array([0, 0, 1.0])).tolist(), physicsClientId=self.pb)
        img = p.getCameraImage(CAM_W, CAM_H, view, self.proj, renderer=p.ER_TINY_RENDERER, flags=p.ER_NO_SEGMENTATION_MASK,
                               physicsClientId=self.pb)
        rgb = np.asarray(img[2], np.uint8).reshape(CAM_H, CAM_W, 4)[:, :, :3].copy()
        buf = np.asarray(img[3], np.float32).reshape(CAM_H, CAM_W)
        z = (CAM_FAR * CAM_NEAR) / (CAM_FAR - (CAM_FAR - CAM_NEAR) * buf)
        sky = buf >= 0.9999
        rgb[sky] = (200, 215, 235)
        z = z + self.rng.normal(0, 1, z.shape).astype(np.float32) * 0.002 * z * z
        z[sky | (z >= CAM_FAR)] = np.nan
        return rgb, z.astype(np.float32)


def env_sensor_sim(xy_mission, t, rng):
    """SIMULATED environmental sensor (no such hardware exists in the robot_vision UGV); field centred on P3."""
    d2 = (xy_mission[0] - 9.0) ** 2 + (xy_mission[1] - 13.0) ** 2
    return {"temperature_c": 24.0 + 5.0 * math.exp(-d2 / 30) + rng.normal(0, 0.15),
            "humidity_pct": 48.0 - 6.0 * math.exp(-d2 / 30) + rng.normal(0, 0.4),
            "co2_ppm": 420 + 180 * math.exp(-d2 / 20) + rng.normal(0, 4)}


class UgvAgent:
    RES = 0.2
    EXT = (-12.0, 42.0)

    def __init__(self, cfg, bus, frames, vehicle, out_dir, log):
        self.cfg, self.bus, self.frames, self.veh, self.out, self.log = cfg, bus, frames, vehicle, out_dir, log
        a = cfg["agents"]["UGV"]
        self.vmax, self.standoff, self.hold_s = a["max_speed_mps"], a["inspect_standoff_m"], a["inspect_hold_s"]
        self.x = np.zeros(3)
        n = int((self.EXT[1] - self.EXT[0]) / self.RES)
        self.L = np.zeros((n, n), np.float32)
        self.planner = AStarPlanner({"clearance_m": 0.45, "turn_penalty": 0.1, "max_nodes": 90000, "dynamic_mode": "block"},
                                    self.RES, (self.EXT[0], self.EXT[0]))
        self.state = "BOOT"
        self.task = None
        self.path = []
        self.goal = None
        self.last_plan = -9
        self.last_status = -9
        self.last_base = 0.0
        self.battery = 100.0
        self.rng = np.random.default_rng(11)
        self.stuck_t = None
        self.reverse_until = 0
        self.inspect = None
        self.traj = []
        self.last_rgb = None
        self.log_done = set()
        self.cmd = (0.0, 0.0)
        self.bias = 0.0
        self.stalled = False
        self.zprev = None
        self.stall_n = 0
        self.stall_events = 0

    def pose_mission(self):
        m = self.frames.convert([self.x[0], self.x[1], 0.0], UGV_ODOM, MISSION)
        yaw = self.x[2] + math.radians(self.cfg["agents"]["UGV"]["dock_heading_deg"])
        return m, yaw

    def odometry(self, dt):
        wl, wr = self.veh.encoders()
        v = WHEEL_R * (wl + wr) / 2
        if self.stalled:
            v = 0.0
        g = self.veh.gyro_z()
        if abs(wl) < 0.08 and abs(wr) < 0.08 and abs(self.cmd[0]) < 1e-3 and abs(self.cmd[1]) < 1e-3:
            self.bias = 0.99 * self.bias + 0.01 * g
            return
        self.x[2] += (g - self.bias) * dt
        self.x[0] += v * math.cos(self.x[2]) * dt
        self.x[1] += v * math.sin(self.x[2]) * dt

    def _stall_check(self, z):
        wl, wr = self.veh.encoders()
        moving = abs(WHEEL_R * (wl + wr) / 2) > 0.12 and abs(wl - wr) < 1.0
        if self.zprev is not None and moving:
            both = np.isfinite(z) & np.isfinite(self.zprev)
            change = float(np.mean(np.abs(z[both] - self.zprev[both]))) if both.any() else 1.0
            self.stall_n = self.stall_n + 1 if change < 0.01 else 0
        else:
            self.stall_n = 0
        was = self.stalled
        self.stalled = self.stall_n >= 2
        if self.stalled and not was:
            self.stall_events += 1
            self.reverse_until = self.t_now + 1.5
            self.log("[UGV] wheel-slip detected (encoders moving, depth image static) -> translation not integrated, backing off")
        self.zprev = z

    def map_update(self, z):
        f, _, cx, cy = self.veh.K
        vs, us = np.mgrid[4:CAM_H:6, 0:CAM_W:6]
        d = z[vs, us]
        ok = np.isfinite(d) & (d < 7.0)
        u, v, d = us[ok], vs[ok], d[ok]
        xo, yo = (u - cx) / f * d, (v - cy) / f * d
        pitch = math.atan2(0.08, 1.0)
        fwd = d * math.cos(pitch) - yo * math.sin(pitch)
        up = d * -math.sin(pitch) - yo * math.cos(pitch) + 0.13 + CAM_POS_BODY[2]
        left = -xo
        bx, by = fwd + CAM_POS_BODY[0], left
        c, s = math.cos(self.x[2]), math.sin(self.x[2])
        wx, wy = self.x[0] + c * bx - s * by, self.x[1] + s * bx + c * by
        obst = (up > 0.12) & (up < 1.6)
        gx0, gy0 = self.x[0] + c * CAM_POS_BODY[0], self.x[1] + s * CAM_POS_BODY[0]
        for k in np.linspace(0.1, 0.92, 9):
            fi = ((gx0 + k * (wx - gx0) - self.EXT[0]) / self.RES).astype(int)
            fj = ((gy0 + k * (wy - gy0) - self.EXT[0]) / self.RES).astype(int)
            m = (fi >= 0) & (fi < self.L.shape[0]) & (fj >= 0) & (fj < self.L.shape[1])
            np.add.at(self.L, (fi[m], fj[m]), -0.15)
        oi = ((wx[obst] - self.EXT[0]) / self.RES).astype(int)
        oj = ((wy[obst] - self.EXT[0]) / self.RES).astype(int)
        m = (oi >= 0) & (oi < self.L.shape[0]) & (oj >= 0) & (oj < self.L.shape[1])
        np.add.at(self.L, (oi[m], oj[m]), 0.9)
        np.clip(self.L, -2.0, 4.0, out=self.L)
        front = obst & (bx > 0) & (bx < 0.75) & (np.abs(by) < 0.32)
        return int(front.sum())

    def grid(self):
        g = np.full(self.L.shape, FREE, np.int16)
        g[self.L > 0.7] = OCC
        return g

    def occupied_mission(self):
        ii, jj = np.nonzero(self.L > 0.7)
        xs, ys = self.EXT[0] + (ii + 0.5) * self.RES, self.EXT[0] + (jj + 0.5) * self.RES
        th = math.radians(self.cfg["agents"]["UGV"]["dock_heading_deg"])
        t = self.frames.tf[(MISSION, UGV_ODOM)].t
        return np.c_[math.cos(th) * xs - math.sin(th) * ys + t[0], math.sin(th) * xs + math.cos(th) * ys + t[1]]

    def plan(self, t):
        res = self.planner.plan(self.grid(), (self.x[0], self.x[1]), self.goal)
        self.last_plan = t
        if res.found:
            self.path = list(res.path)
        return res

    def status_body(self, t):
        pm, yaw = self.pose_mission()
        return {"agent_id": "UGV", "pose": [pm[0], pm[1], 0.0], "yaw_deg": math.degrees(yaw), "frame_id": MISSION,
                "pose_odom": [self.x[0], self.x[1], math.degrees(self.x[2])], "odom_frame_id": UGV_ODOM,
                "battery_pct": round(self.battery, 1), "health": "OK", "localisation": "VALID",
                "localisation_source": "wheel encoders + gyro odometry, registered at dock",
                "current_task": self.task["task_id"] if self.task else None, "state": self.state,
                "comm": "OK" if self.state != "COMM_LOSS_HOLD" else "LOST"}

    def step(self, t, dt, frame=None):
        self.t_now = t
        self.odometry(dt)
        self.battery -= 0.01 * dt
        front = 0
        if frame is not None:
            rgb, z = frame
            self.last_rgb = rgb
            self._stall_check(z)
            front = self.map_update(z)
            self.front = front
            pm, _ = self.pose_mission()
            self.traj.append((t, pm[0], pm[1]))
        for m in self.bus.take("UGV"):
            self.last_base = t
            self._handle(t, m)
        if self.state == "BOOT":
            self.bus.send("UGV", "BASE", "REGISTER", self.status_body(t))
            self.state = "IDLE"
            self.last_status = t
        if t - self.last_status >= self.cfg["comms"]["heartbeat_period_s"]:
            self.last_status = t
            self.bus.send("UGV", "BASE", "STATUS", self.status_body(t))
            if self.task and self.state == "NAVIGATING":
                pm, _ = self.pose_mission()
                dist = math.hypot(self.goal[0] - self.x[0], self.goal[1] - self.x[1])
                prog = max(0.0, min(0.95, 1 - dist / max(self.task["d0"], 1e-3)))
                self.bus.send("UGV", "BASE", "TASK_PROGRESS", {"task_id": self.task["task_id"], "agent_id": "UGV",
                              "status": "IN_PROGRESS", "progress": round(prog, 2), "pose": [pm[0], pm[1], 0.0], "frame_id": MISSION})
        lost = t - self.last_base > self.cfg["comms"]["timeout_s"]
        if lost and self.state in ("NAVIGATING", "INSPECTING", "IDLE"):
            self.resume_state = self.state
            self.state = "COMM_LOSS_HOLD"
            self.log("[UGV] onboard fail-safe: no base-station heartbeat -> COMM_LOSS_HOLD (stopped)")
        elif not lost and self.state == "COMM_LOSS_HOLD":
            self.state = self.resume_state
            self.log(f"[UGV] link restored -> resume {self.state}")
        if self.state == "NAVIGATING":
            self._navigate(t, front)
        elif self.state == "INSPECTING":
            self._inspect(t, frame)
        else:
            self._drive(0.0, 0.0)

    def _handle(self, t, m):
        mt = m["msg_type"]
        if mt == "TASK_ASSIGNMENT":
            tgt_o = np.array(m["target_xyz_ugv_odom"], float)
            chk = self.frames.convert(m["target_xyz"], MISSION, UGV_ODOM)
            valid = bool(np.all(np.isfinite(tgt_o)) and np.linalg.norm(chk - tgt_o) < 0.05 and m["ugv_frame_id"] == UGV_ODOM)
            d = tgt_o[:2] - self.x[:2]
            dist = float(np.linalg.norm(d))
            goal = tgt_o[:2] - d / max(dist, 1e-6) * self.standoff
            self.goal = (float(goal[0]), float(goal[1]))
            res = self.plan(t) if valid else None
            accepted = valid and res is not None and res.found
            self.bus.send("UGV", "BASE", "TASK_ACK", {"task_id": m["task_id"], "agent_id": "UGV",
                          "status": "ACCEPTED" if accepted else "REJECTED", "goal_frame": UGV_ODOM,
                          "goal_xyz": [self.goal[0], self.goal[1], 0.0], "target_frame_check_m": float(np.linalg.norm(chk - tgt_o)),
                          "note": f"coordinates valid, A* path {len(self.path)} pts" if accepted else "invalid target / no path"})
            if accepted:
                self.task = {**m, "target_odom": tgt_o, "d0": dist, "t0": t}
                self.state = "NAVIGATING"
                self.log(f"[UGV] accepted {m['task_id']}: target odom {np.round(tgt_o, 2).tolist()} goal {np.round(self.goal, 2).tolist()}")
        elif mt == "FINAL_STATUS_REQUEST":
            self.bus.send("UGV", "BASE", "STATUS", {**self.status_body(t), "final": True,
                          "occupied_cells": int((self.L > 0.7).sum()), "distance_travelled_m": self._dist()})
        elif mt == "COMMAND" and m["command"] == "END_MISSION_SAFE":
            self.state = "PARKED"
            self.log("[UGV] END_MISSION_SAFE -> PARKED (motors stopped)")

    def _dist(self):
        if len(self.traj) < 2:
            return 0.0
        a = np.array(self.traj)[:, 1:]
        return float(np.sum(np.linalg.norm(np.diff(a, axis=0), axis=1)))

    def _drive(self, v, w):
        self.cmd = (v, w)
        self.veh.command((v - w * TRACK / 2) / WHEEL_R, (v + w * TRACK / 2) / WHEEL_R)

    def _navigate(self, t, front):
        gx, gy = self.goal
        dist = math.hypot(gx - self.x[0], gy - self.x[1])
        if dist < 0.35:
            self._drive(0, 0)
            self.state = "INSPECTING"
            self.inspect = {"t0": t, "phase": "face", "data": [], "images": [], "seen": 0, "meas": []}
            self.log(f"[UGV] reached inspection standoff for {self.task['task_id']} (odom {np.round(self.x[:2], 2).tolist()})")
            return
        if t < self.reverse_until:
            self._drive(-0.25, 0.6)
            return
        if t - self.last_plan > 1.0:
            self.plan(t)
        while len(self.path) > 1 and math.hypot(self.path[0][0] - self.x[0], self.path[0][1] - self.x[1]) < 0.4:
            self.path.pop(0)
        tx, ty = self.path[0] if self.path else (gx, gy)
        err = math.atan2(math.sin(math.atan2(ty - self.x[1], tx - self.x[0]) - self.x[2]), math.cos(math.atan2(ty - self.x[1], tx - self.x[0]) - self.x[2]))
        v = self.vmax * max(0.0, math.cos(err)) ** 2 * min(1.0, dist / 1.0 + 0.3)
        w = float(np.clip(2.0 * err, -1.2, 1.2))
        if front > 6 and v > 0.05:
            v = 0.0
            if self.stuck_t is None:
                self.stuck_t = t
                self.plan(t)
            elif t - self.stuck_t > 4.0:
                self.reverse_until = t + 1.5
                self.stuck_t = None
        else:
            self.stuck_t = None
        self._drive(v, w)

    def _inspect(self, t, frame):
        ins, task = self.inspect, self.task
        tgt = task["target_odom"]
        bearing = math.atan2(tgt[1] - self.x[1], tgt[0] - self.x[0])
        err = math.atan2(math.sin(bearing - self.x[2]), math.cos(bearing - self.x[2]))
        action = task["action"]
        if ins["phase"] == "face":
            self._drive(0, float(np.clip(1.5 * err, -0.8, 0.8)))
            if abs(err) < 0.08:
                ins["phase"], ins["t1"] = "collect", t
            return
        if ins["phase"] == "collect":
            self._drive(0, float(np.clip(1.5 * err, -0.4, 0.4)))
            if frame is not None:
                rgb, z = frame
                mk = task["observations"]["marker"]
                mask = marker_mask(rgb, mk)
                frac = float(mask.mean())
                if frac > 0.004:
                    ins["seen"] += 1
                    vv, uu = np.nonzero(mask)
                    dd = z[vv, uu]
                    good = np.isfinite(dd)
                    if good.sum() > 10:
                        f, _, cx, cy = self.veh.K
                        dm = float(np.median(dd[good]))
                        u0, v0 = float(np.median(uu[good])), float(np.median(vv[good]))
                        fwd, left = dm + CAM_POS_BODY[0], -(u0 - cx) / f * dm
                        c, s = math.cos(self.x[2]), math.sin(self.x[2])
                        ins["meas"].append([self.x[0] + c * fwd - s * left, self.x[1] + s * fwd + c * left])
                if len(ins["images"]) < 2 and frac > 0.004:
                    path = os.path.join(self.out, "images", f"ugv_{task['task_id']}_{len(ins['images'])}.png")
                    cv2.imwrite(path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                    ins["images"].append(os.path.relpath(path, self.out))
                if action == "environmental_sampling":
                    pm, _ = self.pose_mission()
                    ins["data"].append(env_sensor_sim(pm, t, self.rng))
            if t - ins["t1"] >= self.hold_s:
                if action == "unexplored_region_inspection":
                    ins["phase"], ins["yaw0"], ins["acc"], ins["cells0"] = "scan", self.x[2], 0.0, int((self.L > 0.7).sum() + (self.L < -0.5).sum())
                else:
                    self._complete(t)
            return
        if ins["phase"] == "scan":
            self._drive(0, 0.7)
            ins["acc"] = abs(self.x[2] - ins["yaw0"])
            if ins["acc"] >= 2 * math.pi:
                self._complete(t)

    def _complete(self, t):
        ins, task = self.inspect, self.task
        self._drive(0, 0)
        meas = np.array(ins["meas"]) if ins["meas"] else None
        sensor = {}
        if meas is not None:
            mo = meas.mean(axis=0)
            mm = self.frames.convert([mo[0], mo[1], 0.0], UGV_ODOM, MISSION)
            sensor["ugv_observed_target_odom"] = [float(mo[0]), float(mo[1])]
            sensor["ugv_observed_target_mission"] = [float(mm[0]), float(mm[1])]
            sensor["uav_reported_target_mission"] = task["target_xyz"][:2]
            sensor["cross_platform_residual_m"] = float(np.hypot(mm[0] - task["target_xyz"][0], mm[1] - task["target_xyz"][1]))
        if ins["data"]:
            keys = ins["data"][0].keys()
            sensor["environment_SIMULATED_SENSOR"] = {k: round(float(np.mean([d[k] for d in ins["data"]])), 2) for k in keys}
            sensor["environment_samples"] = len(ins["data"])
        if task["action"] == "unexplored_region_inspection":
            sensor["local_map_cells_added"] = int((self.L > 0.7).sum() + (self.L < -0.5).sum()) - ins["cells0"]
            sensor["scan_deg"] = 360
        sensor["marker_frames_seen"] = ins["seen"]
        performed = ins["seen"] >= 3
        pm, _ = self.pose_mission()
        self.bus.send("UGV", "BASE", "TASK_COMPLETE", {"task_id": task["task_id"], "agent_id": "UGV",
                      "status": "COMPLETED" if performed else "FAILED", "frame_id": MISSION, "pose": [pm[0], pm[1], 0.0],
                      "results": {"images": ins["images"], "sensor_data": sensor, "inspection_performed": performed,
                                  "duration_s": round(t - task["t0"], 1)},
                      "note": "" if performed else "target marker not observed"})
        self.log(f"[UGV] {task['task_id']} inspection done: {sensor}")
        self.task, self.inspect, self.state = None, None, "IDLE"
