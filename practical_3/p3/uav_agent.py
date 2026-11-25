"""UAV agent: companion-computer side. Talks to the simulated FC only over MAVLink (companion/mavlink_client.py)."""
from __future__ import annotations

import math
import os

import cv2
import numpy as np
from pymavlink.dialects.v20 import common as mavlink

from common.frames import R_from_euler, flu_to_frd
from p3.frames import MISSION, UAV_LOCAL, register_uav
from p3.perception import HSV, marker_mask


class TargetTrack:
    def __init__(self, marker):
        self.marker, self.obs, self.reported = marker, [], False

    def add(self, p):
        if self.obs and np.linalg.norm(np.mean(self.obs, axis=0) - p) > 4.0:
            return
        self.obs.append(p)

    def mean(self):
        return np.mean(self.obs, axis=0)

    def sigma(self):
        return float(np.sqrt(np.mean(np.sum((np.array(self.obs) - self.mean()) ** 2, axis=1)))) if len(self.obs) > 1 else 9.9


class UavAgent:
    def __init__(self, cfg, bus, frames, mav, cam, model, out_dir, log):
        self.cfg, self.bus, self.frames, self.mav, self.cam, self.model = cfg, bus, frames, mav, cam, model
        self.out, self.log = out_dir, log
        a = cfg["agents"]["UAV"]
        self.alt, self.speed = a["survey_altitude_m"], a["survey_speed_mps"]
        self.state = "WAIT_LINK"
        self.task = None
        self.wps = []
        self.tracks = {k: TargetTrack(k) for k in HSV}
        self.last_status = -9
        self.last_base = 0.0
        self.t_state = 0.0
        self.traj = []
        self.yaw_goal = None
        self.inspect = None
        self.last_rgb = None
        self.reg = None

    def tel(self):
        return self.mav.telemetry()

    def pose_local(self):
        tm = self.tel()
        return None if tm is None else (tm.local_ned, tm.roll_rad, tm.pitch_rad, tm.yaw_rad)

    def to_local(self, mission_xyz):
        return self.frames.convert(mission_xyz, MISSION, UAV_LOCAL)

    def pose_mission(self):
        pl = self.pose_local()
        return self.frames.convert(pl[0], UAV_LOCAL, MISSION), pl[3]

    def _set(self, s, t):
        self.state, self.t_state = s, t
        self.log(f"[UAV] state -> {s}")

    def status_body(self, t):
        tm = self.tel()
        pm, yaw = self.pose_mission()
        armed = bool(tm.armed)
        return {"agent_id": "UAV", "pose": pm, "frame_id": MISSION, "pose_local": tm.local_ned, "local_frame_id": UAV_LOCAL,
                "yaw_deg": 90 - math.degrees(yaw), "battery_pct": int(tm.battery_pct) if tm.battery_pct >= 0 else 100,
                "health": "OK", "localisation": "VALID", "localisation_source": "FC EKF LOCAL_POSITION_NED via MAVLink",
                "current_task": self.task["task_id"] if self.task else None, "state": self.state,
                "fc_mode": tm.mode, "armed": armed, "comm": "OK"}

    def step(self, t, frame=None):
        tm = self.tel()
        if tm is None:
            return
        if self.reg is None:
            self.reg = register_uav(self.cfg["agents"]["UAV"]["pad_mission_xyz"], tm.local_ned)
            self.frames.add(self.reg)
            self.log(f"[FRAME] registered {UAV_LOCAL}: {self.reg.describe()}")
            self.bus.send("UAV", "BASE", "REGISTER", self.status_body(t))
            self._set("IDLE_ON_PAD", t)
            self.last_status = t
        pm, yaw = self.pose_mission()
        if frame is not None:
            self.traj.append((t, pm[0], pm[1], pm[2]))
            self.last_rgb = frame[0]
            if self.state in ("SURVEY", "INSPECT_GOTO", "INSPECT_HOLD"):
                self._detect(t, frame)
        for m in self.bus.take("UAV"):
            self.last_base = t
            self._handle(t, m)
        if t - self.last_status >= self.cfg["comms"]["heartbeat_period_s"]:
            self.last_status = t
            self.bus.send("UAV", "BASE", "STATUS", self.status_body(t))
            if self.task and self.state in ("SURVEY", "INSPECT_GOTO", "INSPECT_HOLD", "TAKEOFF"):
                self.bus.send("UAV", "BASE", "TASK_PROGRESS", {"task_id": self.task["task_id"], "agent_id": "UAV",
                              "status": "IN_PROGRESS", "progress": round(self._progress(), 2), "pose": pm, "frame_id": MISSION})
        self._fly(t, tm, pm, yaw)

    def _progress(self):
        if self.task["task_id"] == "T1":
            return 1 - len(self.wps) / max(1, self.task["n_wp"])
        if self.state == "INSPECT_HOLD":
            return 0.5 + 0.5 * min(1, (self.inspect["t"] - self.inspect["t0"]) / self.cfg["agents"]["UAV"]["inspect_hold_s"])
        return 0.3

    def _handle(self, t, m):
        if m["msg_type"] == "TASK_ASSIGNMENT":
            self.task = m
            self.bus.send("UAV", "BASE", "TASK_ACK", {"task_id": m["task_id"], "agent_id": "UAV", "status": "ACCEPTED",
                          "goal_frame": UAV_LOCAL, "goal_xyz": [0, 0, 0], "note": "UAV accepted"})
            if m["task_id"] == "T1":
                a = self.cfg["agents"]["UAV"]
                x0, x1 = a["survey_x_range"]
                wps = []
                for k, y in enumerate(a["survey_lanes_y"]):
                    xs = (x0, x1) if k % 2 == 0 else (x1, x0)
                    wps += [(xs[0], y, self.alt), (xs[1], y, self.alt)]
                self.wps = wps
                m["n_wp"] = len(wps)
                self._arm_takeoff(t)
            elif m["task_id"] == "T2":
                tg = np.array(m["target_xyz"], float)
                so = self.cfg["agents"]["UAV"]["inspect_standoff_m"]
                self.wps = [(tg[0], tg[1] - so, self.alt)]
                self.inspect = {"target": tg, "images": [], "frac": [], "t0": None, "t": 0, "range": []}
                self._set("INSPECT_GOTO", t)
        elif m["msg_type"] == "COMMAND" and m["command"] == "RETURN_AND_STANDBY":
            pad = self.cfg["agents"]["UAV"]["pad_mission_xyz"]
            self.wps = [(pad[0], pad[1], self.alt), (pad[0], pad[1], 1.5)]
            self._set("RETURN", t)
        elif m["msg_type"] == "FINAL_STATUS_REQUEST":
            body = self.status_body(t)
            body.update(final=True, targets_reported=[k for k, v in self.tracks.items() if v.reported])
            self.bus.send("UAV", "BASE", "STATUS", body)
        elif m["msg_type"] == "COMMAND" and m["command"] == "END_MISSION_SAFE":
            self._set("END_SAFE", t)

    def _arm_takeoff(self, t):
        self._set("ARMING", t)

    def _vel(self, vn, ve, vd, yaw_rate=0.0):
        self.mav.send_velocity_ned(float(vn), float(ve), float(vd), float(yaw_rate))

    def _yaw_rate_to(self, yaw, yaw_goal):
        e = math.atan2(math.sin(yaw_goal - yaw), math.cos(yaw_goal - yaw))
        return float(np.clip(1.2 * e, -0.8, 0.8)), e

    def _goto(self, pm, yaw, wp, speed):
        d = np.array(wp) - pm
        dist = float(np.linalg.norm(d))
        v_enu = d / max(dist, 1e-6) * min(speed, 0.8 * dist + 0.2)
        if np.linalg.norm(d[:2]) > 1.5:
            self.yaw_goal = math.atan2(d[0], d[1])
        yr, _ = self._yaw_rate_to(yaw, self.yaw_goal if self.yaw_goal is not None else yaw)
        self._vel(v_enu[1], v_enu[0], -v_enu[2], yr)
        return dist

    def _fly(self, t, tm, pm, yaw):
        s = self.state
        if s == "ARMING":
            self._vel(0, 0, 0)
            if t - self.t_state > 1.0 and not tm.armed:
                self.mav.command_long(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, wait_ack=False)
            if tm.armed and tm.mode != "OFFBOARD":
                from common.fc_modes import mode_to_custom_mode
                cm = mode_to_custom_mode("OFFBOARD")
                self.mav.command_long(mavlink.MAV_CMD_DO_SET_MODE, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                      (cm >> 16) & 0xFF, (cm >> 24) & 0xFF, wait_ack=False)
            if tm.armed and tm.mode == "OFFBOARD":
                self.log("[UAV] FC armed + OFFBOARD (MAVLink COMMAND_LONG acknowledged)")
                self._set("TAKEOFF", t)
        elif s == "TAKEOFF":
            if self._goto(pm, yaw, (pm[0], pm[1], self.alt), 1.2) < 0.3:
                self._set("SURVEY", t)
        elif s == "SURVEY":
            if not self.wps:
                self._complete_t1(t)
                self._vel(0, 0, 0)
                return
            if self._goto(pm, yaw, self.wps[0], self.speed) < 0.8:
                self.wps.pop(0)
        elif s == "HOVER":
            self._vel(0, 0, 0)
        elif s == "INSPECT_GOTO":
            tg = self.inspect["target"]
            self.yaw_goal = math.atan2(tg[0] - pm[0], tg[1] - pm[1])
            if self._goto(pm, yaw, self.wps[0], 1.8) < 0.5:
                self.inspect["t0"] = t
                self._set("INSPECT_HOLD", t)
        elif s == "INSPECT_HOLD":
            tg = self.inspect["target"]
            yr, e = self._yaw_rate_to(yaw, math.atan2(tg[0] - pm[0], tg[1] - pm[1]))
            hold = np.array(self.wps[0]) - pm
            self._vel(0.6 * hold[1], 0.6 * hold[0], -0.6 * hold[2], yr)
            self.inspect["t"] = t
            if t - self.inspect["t0"] >= self.cfg["agents"]["UAV"]["inspect_hold_s"]:
                self._complete_t2(t, pm)
        elif s == "RETURN":
            if len(self.wps) > 0:
                if self._goto(pm, yaw, self.wps[0], 2.0) < 0.4:
                    self.wps.pop(0)
            else:
                self.mav.command_long(mavlink.MAV_CMD_NAV_LAND, wait_ack=False)
                self._set("LANDING", t)
        elif s == "LANDING":
            if not tm.armed:
                self._set("STANDBY_LANDED", t)
            elif t - self.t_state > 1.0 and tm.mode != "LAND":
                self.mav.command_long(mavlink.MAV_CMD_NAV_LAND, wait_ack=False)
                self.t_state = t

    def _detect(self, t, frame):
        rgb, z = frame
        pl = self.pose_local()
        Rn = R_from_euler(pl[1], pl[2], pl[3])
        K = self.cam.intrinsics()
        fx, fy, cx, cy = K["fx"], K["fy"], K["cx"], K["cy"]
        for name, tr in self.tracks.items():
            mask = marker_mask(rgb, name)
            if mask.sum() < 25:
                continue
            vv, uu = np.nonzero(mask)
            dd = z[vv, uu]
            good = np.isfinite(dd)
            if good.sum() < 15:
                continue
            u, v, d = np.median(uu[good]), np.median(vv[good]), float(np.median(dd[good]))
            p_opt = np.array([(u - cx) / fx * d, (v - cy) / fy * d, d])
            p_flu = self.model.cam_optical_pos_flu + self.model.cam_R_body_optical @ p_opt
            p_ned = pl[0] + Rn @ flu_to_frd(p_flu)
            tr.add(p_ned)
            if not tr.reported and len(tr.obs) >= 5 and tr.sigma() < 1.5:
                tr.reported = True
                path = os.path.join(self.out, "images", f"uav_detect_{name}.png")
                vis = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                x0, y0, x1, y1 = uu.min(), vv.min(), uu.max(), vv.max()
                cv2.rectangle(vis, (int(x0), int(y0)), (int(x1), int(y1)), (0, 0, 255), 2)
                cv2.putText(vis, f"ROI {name}", (int(x0), max(12, int(y0) - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
                cv2.imwrite(path, vis)
                mean = tr.mean()
                self.bus.send("UAV", "BASE", "TARGET_REPORT", {"agent_id": "UAV", "task_id": "T1", "marker": name,
                              "xyz": mean, "frame_id": UAV_LOCAL, "sigma_m": tr.sigma(), "n_obs": len(tr.obs),
                              "image": os.path.relpath(path, self.out),
                              "uav_pose_local": pl[0], "mapping_update": {"roi_count": sum(v.reported for v in self.tracks.values())}})
                self.log(f"[UAV] ROI {name} confirmed: {UAV_LOCAL} {np.round(mean, 2).tolist()} sigma {tr.sigma():.2f} m n={len(tr.obs)}")
            if self.state == "INSPECT_HOLD" and name == "orange":
                self.inspect["frac"].append(float(mask.mean()))
                self.inspect["range"].append(d)
                if len(self.inspect["images"]) < 2 and mask.mean() > 0.01:
                    path = os.path.join(self.out, "images", f"uav_T2_inspection_{len(self.inspect['images'])}.png")
                    cv2.imwrite(path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                    self.inspect["images"].append(os.path.relpath(path, self.out))

    def _complete_t1(self, t):
        pm, _ = self.pose_mission()
        rep = [k for k, v in self.tracks.items() if v.reported]
        self.bus.send("UAV", "BASE", "TASK_COMPLETE", {"task_id": "T1", "agent_id": "UAV", "status": "COMPLETED",
                      "frame_id": MISSION, "pose": pm,
                      "results": {"images": [f"images/uav_detect_{k}.png" for k in rep], "inspection_performed": True,
                                  "sensor_data": {"rois_reported": rep, "lanes_flown": len(self.cfg['agents']['UAV']['survey_lanes_y']),
                                                  "observations": {k: len(v.obs) for k, v in self.tracks.items()}}}})
        self.task = None
        self._set("HOVER", t)

    def _complete_t2(self, t, pm):
        ins = self.inspect
        frames_seen = sum(f > 0.01 for f in ins["frac"])
        ok = frames_seen >= 5
        tr = self.tracks["orange"]
        self.bus.send("UAV", "BASE", "TASK_COMPLETE", {"task_id": "T2", "agent_id": "UAV", "status": "COMPLETED" if ok else "FAILED",
                      "frame_id": MISSION, "pose": pm, "target_xyz": ins["target"],
                      "results": {"images": ins["images"], "inspection_performed": ok,
                                  "sensor_data": {"marker_frames_seen": frames_seen,
                                                  "mean_marker_fraction": float(np.mean(ins["frac"])) if ins["frac"] else 0.0,
                                                  "mean_range_m": float(np.mean(ins["range"])) if ins["range"] else None,
                                                  "refined_target_mission": self.frames.convert(tr.mean(), UAV_LOCAL, MISSION)},
                                  "mapping_update": {"orange_observations": len(tr.obs)}}})
        self.task = None
        self._set("HOVER", t)
