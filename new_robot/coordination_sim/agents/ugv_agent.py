"""UGV onboard agent = the UGV project's EXISTING localisation / mapping / planning classes + a small task executor."""
from __future__ import annotations

import math
import os

import cv2
import numpy as np

import stage8_stabilized_navigation as S8
from viwo_estimator import ViwoEstimator, VisualPoseMeasurement

from coordination_sim.comms.messages import Observation, Register, TaskAck, Telemetry

CONF = {"LOCALIZATION_CONFIDENT": 0.95, "LOCALIZATION_CONTROL_ONLY": 0.7, "LOCALIZATION_UNCERTAIN": 0.45, "LOCALIZATION_CRITICAL": 0.1}


def configure_ugv_map_bounds(b):
    """The Stage-8 grid extent is a set of module-level constants read at call time; widen it for this larger site."""
    S8.MAP_X_MIN, S8.MAP_X_MAX = float(b["x_min"]), float(b["x_max"])
    S8.MAP_Z_MIN, S8.MAP_Z_MAX = float(b["z_min"]), float(b["z_max"])
    S8.GRID_WIDTH = int(round((S8.MAP_X_MAX - S8.MAP_X_MIN) / S8.GRID_RESOLUTION))
    S8.GRID_HEIGHT = int(round((S8.MAP_Z_MAX - S8.MAP_Z_MIN) / S8.GRID_RESOLUTION))


def _wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


class UgvAgent:
    def __init__(self, cfg, agent_cfg, radio, kinematics, intrinsics, camera_matrix, out_dir):
        self.cfg, self.a = cfg, agent_cfg
        self.id = agent_cfg["id"]
        self.radio = radio
        self.intrinsics, self.K = intrinsics, camera_matrix
        configure_ugv_map_bounds(agent_cfg["map_bounds"])
        self.vo = S8.VisualOdometry()
        self.est = ViwoEstimator(kinematics.wheel_radius_m, kinematics.wheel_separation_m, camera_forward_offset_m=S8.CAMERA_FORWARD_OFFSET_M)
        self.ground = S8.GroundPlaneEstimator()
        self.occ = S8.OccupancyMap()
        self.planner = S8.GlobalPlanner()
        self.local = S8.LocalPlanner()
        self.initialised = False
        self.pose = None
        self.seq = 0
        self.mode = "BOOT"
        self.task = None
        self.path = []
        self.path_len0 = None
        self.last_plan_t = -1e9
        self.no_route_since = None
        self.inspect_t0 = None
        self.last_hb_rx = None
        self.comm_ok = True
        self.keepouts = []
        self.next_tel = 0.0
        self.next_map = 0.0
        self.registered = False
        self.cmd = (0.0, 0.0)
        self.events = []
        self.evidence_dir = os.path.join(out_dir, "evidence_ugv")
        os.makedirs(self.evidence_dir, exist_ok=True)
        self.battery_pct = 100.0
        self.last_bgr = None
        self.last_vo_ok = False
        self._t = 0.0
        self.stats ={"vo_updates": 0, "vo_success": 0, "plans": 0, "plan_failures": 0, "emergency_stops": 0, "comm_hold_s": 0.0}

    def _send(self, t, msg):
        self.seq += 1
        msg.seq, msg.stamp, msg.agent_id = self.seq, t, self.id
        self.radio.send(t, self.id, "BASE", msg)

    def _event(self, t, kind, **kw):
        self.events.append({"t": round(t, 2), "event": kind, **kw})

    def handle_messages(self, t):
        for m in self.radio.receive(self.id):
            if m.kind == "HEARTBEAT":
                self.last_hb_rx = t
            elif m.kind == "TASK_COMMAND":
                self._on_command(t, m)

    def _on_command(self, t, m):
        if m.command in ("ASSIGN", "RESUME"):
            goal, target = tuple(m.goal[0]), tuple(m.goal[-1])
            self.keepouts = m.params.get("keepouts", self.keepouts)
            if self.task and self.task["id"] == m.task_id and self.task["state"] == "HELD":
                self.task["state"] = self.task.get("state_before_hold", "EN_ROUTE")
                self._event(t, "TASK_RESUMED", task=m.task_id)
                self._send(t, TaskAck(task_id=m.task_id, accepted=True, reason="resumed from HELD"))
            else:
                self.task = {"id": m.task_id, "type": m.task_type, "goal": goal, "target": target, "dwell_s": float(m.params.get("dwell_s", 4.0)),
                             "state": "EN_ROUTE", "progress": 0.0, "name": m.params.get("name", "")}
                self.path, self.path_len0, self.no_route_since = [], None, None
                self._event(t, "TASK_ACCEPTED", task=m.task_id, goal=goal)
                self._send(t, TaskAck(task_id=m.task_id, accepted=True))
            self.last_plan_t = -1e9
        elif m.command == "CANCEL":
            if self.task and self.task["id"] == m.task_id:
                self._event(t, "TASK_CANCELLED", task=m.task_id, reason=m.params.get("reason", ""))
                self.task = None
                self.path = []
                self._send(t, TaskAck(task_id=m.task_id, accepted=True, reason="cancelled"))
        elif m.command == "KEEPOUT":
            self.keepouts.append({"xy": tuple(m.goal[0]), "radius_m": float(m.params["radius_m"])})
            self.last_plan_t = -1e9
            self._event(t, "KEEPOUT_RECEIVED", xy=m.goal[0])

    def feed_inertial(self, imu, wheel):
        if not self.initialised:
            return
        if imu is not None:
            self.est.propagate(imu)
        if wheel is not None:
            self.est.update_wheels(wheel)

    def bootstrap(self, t, bgr, depth):
        if self.vo.initialise(bgr, depth):
            self.ground.update(depth, self.intrinsics)
            self.est.initialise(self.vo.pose, t)
            self.occ.integrate(depth, self.intrinsics, self.vo.pose, self.ground)
            self.initialised = True
            self.pose = self.est.pose()
            self.mode = "IDLE"
            self._event(t, "LOCALISATION_INITIALISED")
        return self.initialised

    def perceive(self, t, bgr, depth):
        self.last_bgr = bgr
        res = self.vo.update(bgr, depth, self.intrinsics, self.K)
        self.stats["vo_updates"] += 1
        self.last_vo_ok = bool(res.get("success"))
        if res.get("success") and self.vo.pose is not None:
            self.stats["vo_success"] += 1
            self.est.update_visual(VisualPoseMeasurement(t=t, T_world_from_body=self.vo.pose.copy(), inliers=int(res.get("pnp_inliers", 0) or 0),
                                                         inlier_ratio=float(res.get("inlier_ratio", 0.0) or 0.0), reprojection_px=1.0,
                                                         tile_coverage=float(res.get("tile_coverage", 0.0) or 0.0),
                                                         relocalized=bool(res.get("relocalized"))))
        self.pose = self.est.pose()
        self.ground.update(depth, self.intrinsics)
        if self.est.may_write_map():
            self.occ.integrate(depth, self.intrinsics, self.pose, self.ground)

    def _xz(self):
        return S8.pose_position_xz(self.pose)

    def _heading(self):
        return S8.pose_heading(self.pose)

    def _keepout_cells(self):
        cells = []
        for k in self.keepouts:
            X, Z = k["xy"]
            r = k["radius_m"]
            n = int(r / S8.GRID_RESOLUTION) + 1
            c0 = S8.world_to_grid(X, Z)
            if c0 is None:
                continue
            for dr in range(-n, n + 1):
                for dc in range(-n, n + 1):
                    if (dr * dr + dc * dc) * S8.GRID_RESOLUTION ** 2 <= r * r:
                        cells.append((c0[0] + dr, c0[1] + dc))
        return cells

    def _plan(self, t):
        start = S8.world_to_grid(*self._xz())
        goal = S8.world_to_grid(*self.task["goal"])
        self.stats["plans"] += 1
        if start is None or goal is None:
            self.stats["plan_failures"] += 1
            return False
        res = self.planner.plan(self.occ, start, goal, additional_blocked=self._keepout_cells() or None)
        if not res.get("success"):
            self.stats["plan_failures"] += 1
            return False
        self.path = [S8.grid_to_world(r, c) for r, c in res["smoothed"]]
        L = res["length_m"]
        if self.path_len0 is None:
            self.path_len0 = max(L, 0.5)
        self.path_remaining = L
        return True

    def _carrot(self, pos, L=0.65):
        pts = self.path
        if len(pts) < 2:
            return pts[-1] if pts else None
        best, bi, bt = 1e9, 0, 0.0
        for i in range(len(pts) - 1):
            a, b = np.array(pts[i]), np.array(pts[i + 1])
            ab = b - a
            tt = float(np.clip(np.dot(pos - a, ab) / max(np.dot(ab, ab), 1e-9), 0, 1))
            d = float(np.linalg.norm(a + tt * ab - pos))
            if d < best:
                best, bi, bt = d, i, tt
        p_ = np.array(pts[bi]) + bt * (np.array(pts[bi + 1]) - np.array(pts[bi]))
        rem, i = L, bi
        while rem > 0 and i < len(pts) - 1:
            nxt = np.array(pts[i + 1])
            seg = float(np.linalg.norm(nxt - p_))
            if seg >= rem:
                return tuple(p_ + (nxt - p_) / max(seg, 1e-9) * rem)
            rem -= seg
            p_, i = nxt, i + 1
        return tuple(pts[-1])

    def _bearing_deg(self, xz):
        pos = np.array(self._xz())
        d = np.array(xz) - pos
        return _wrap_deg(math.degrees(math.atan2(d[0], d[1]) - self._heading()))

    def control(self, t, depth):
        a = self.a
        self._t = t
        hb_age = 1e9 if self.last_hb_rx is None else t - self.last_hb_rx
        was_ok = self.comm_ok
        self.comm_ok = self.last_hb_rx is not None and hb_age <= a["link_watchdog_s"]
        if was_ok and not self.comm_ok and self.registered:
            self._event(t, "COMM_WATCHDOG", heartbeat_age_s=round(hb_age, 2))
            if self.task and self.task["state"] in ("EN_ROUTE", "INSPECTING"):
                self.task["state_before_hold"] = self.task["state"]
                self.task["state"] = "HELD"
        if not self.initialised:
            return self._stop("BOOT")
        if not self.comm_ok and self.registered:
            self.stats["comm_hold_s"] += 0.2
            return self._stop("COMM_HOLD")
        if not self.est.may_translate():
            return self._stop("LOC_LOST_HOLD")
        if self.task is None or self.task["state"] in ("COMPLETED", "FAILED"):
            return self._stop("IDLE")
        if self.task["state"] == "HELD":
            return self._stop("TASK_HELD")
        pos = np.array(self._xz())
        goal = np.array(self.task["goal"])
        dist = float(np.linalg.norm(goal - pos))
        if self.task["state"] == "EN_ROUTE":
            if dist < a["goal_tolerance_m"]:
                self.task["state"] = "INSPECTING"
                self.inspect_t0 = None
                self._event(t, "GOAL_REACHED", task=self.task["id"], dist=round(dist, 2))
            else:
                return self._drive(t, depth, pos, dist)
        if self.task["state"] == "INSPECTING":
            return self._inspect(t, depth)
        return self._stop("IDLE")

    def _drive(self, t, depth, pos, dist):
        if t - self.last_plan_t > 2.0 or not self.path:
            self.last_plan_t = t
            ok = self._plan(t)
            if not ok:
                self.path = []
                self.no_route_since = self.no_route_since or t
                if t - self.no_route_since > 45.0:
                    self.task["state"] = "FAILED"
                    self.task["fail_reason"] = "no A* route to the goal for 45 s"
                    self._event(t, "TASK_FAILED", task=self.task["id"], reason=self.task["fail_reason"])
                    return self._stop("IDLE")
                b = self._bearing_deg(self.task["goal"])
                if abs(b) > 15:
                    return self._set(S8.spin_wheel_speeds(1 if b > 0 else -1, 0.35), "NO_ROUTE_SCAN")
                return self._stop("NO_ROUTE_HOLD")
            self.no_route_since = None
        carrot = self._carrot(pos) or tuple(self.task["goal"])
        target_bearing = self._bearing_deg(carrot)
        analysis = self.local.analyse(depth, self.intrinsics, self.ground, target_bearing, self.pose, None, self.planner.view(self.occ))
        speed = self.a["max_speed_mps"] * self.est.speed_scale() * float(np.clip(dist / 1.2, 0.35, 1.0))
        if analysis["emergency"]:
            self.stats["emergency_stops"] += 1
            self.last_plan_t = -1e9
            return self._set(S8.reverse_wheel_speeds(0.25), "EMERGENCY_BACKOFF") if analysis["front_min_m"] < 0.25 else self._stop("EMERGENCY_STOP")
        if abs(target_bearing) > S8.LOCAL_MAX_ANGLE_DEG:
            return self._set(S8.spin_wheel_speeds(1 if target_bearing > 0 else -1, 0.4), "EN_ROUTE")
        gap = self.local.select_gap(analysis, None, ignore_lane=True)
        if gap is None:
            self.last_plan_t = -1e9
            return self._set(S8.spin_wheel_speeds(1 if target_bearing > 0 else -1, 0.3), "LOCAL_BLOCKED")
        heading = float(gap["angle_deg"])
        L, R = S8.arc_wheel_speeds(heading, float(gap.get("depth_m", 2.0)), speed_scale=1.0, target_speed_mps=speed)
        if self.path_len0:
            rem = getattr(self, "path_remaining", dist)
            self.task["progress"] = max(self.task["progress"], 0.85 * float(np.clip(1.0 - rem / self.path_len0, 0, 1)))
        return self._set((L, R), "EN_ROUTE")

    def _inspect(self, t, depth):
        b = self._bearing_deg(self.task["target"])
        if self.inspect_t0 is None:
            if abs(b) > 8.0:
                return self._set(S8.spin_wheel_speeds(1 if b > 0 else -1, 0.3), "INSPECT_ALIGN")
            self.inspect_t0 = t
            self._event(t, "INSPECTION_STARTED", task=self.task["id"])
        frac = (t - self.inspect_t0) / max(self.task["dwell_s"], 1e-3)
        self.task["progress"] = max(self.task["progress"], 0.85 + 0.15 * min(frac, 1.0))
        if frac >= 1.0:
            H, W = depth.shape
            patch = depth[int(H * 0.35):int(H * 0.65), int(W * 0.4):int(W * 0.6)]
            valid = patch[np.isfinite(patch) & (patch > 0.05)]
            rng = float(np.median(valid)) if valid.size else float("nan")
            expect = float(np.linalg.norm(np.array(self.task["target"]) - np.array(self._xz())))
            ev = os.path.join(self.evidence_dir, f"{self.task['id']}_t{int(t):03d}.png")
            if self.last_bgr is not None:
                cv2.imwrite(ev, self.last_bgr)
            observed = bool(np.isfinite(rng) and abs(rng - expect) < 0.8)
            self.task["result"] = {"task_id": self.task["id"], "target_range_m": round(rng, 2), "expected_range_m": round(expect, 2),
                                   "target_observed": observed, "evidence_image": os.path.relpath(ev, os.path.dirname(self.evidence_dir)),
                                   "summary": f"UGV ground view: object {'observed' if observed else 'NOT confirmed'} at {rng:.2f} m "
                                              f"(expected {expect:.2f} m), evidence {os.path.basename(ev)}"}
            self.task["state"] = "COMPLETED"
            self.task["progress"] = 1.0
            self._event(t, "TASK_COMPLETED", task=self.task["id"], **self.task["result"])
            self._send(t, Observation(obs_type="INSPECTION_RESULT", frame="ugv_map", position=tuple(self.task["target"]), data=self.task["result"]))
            return self._stop("IDLE")
        return self._stop("INSPECTING")

    def _stop(self, mode):
        return self._set((0.0, 0.0), mode)

    def _set(self, cmd, mode):
        if mode != self.mode:
            self._event(self._t, "MODE", mode=mode)
        self.mode = mode
        self.cmd = (float(cmd[0]), float(cmd[1]))
        return self.cmd

    def report(self, t, battery_pct):
        self._t = t
        self.battery_pct = battery_pct
        if not self.registered:
            self._send(t, Register(vehicle_type="ugv", capabilities=["ugv"], frame="ugv_map",
                                   sensors=["rgbd_camera 640x480", "imu", "wheel_encoders"],
                                   software={"localisation": "VisualOdometry + ViwoEstimator (ESKF)", "mapping": "Stage-8 OccupancyMap",
                                             "planning": "Stage-8 GlobalPlanner A* + LocalPlanner gaps"}))
            self.registered = True
        if t >= self.next_tel:
            self.next_tel = t + 1.0 / self.cfg["comms"]["telemetry_hz"]
            tier = self.est.confidence() if self.initialised else "LOCALIZATION_CRITICAL"
            pos = tuple(float(v) for v in self.pose[:3, 3]) if self.pose is not None else (0.0, 0.0, 0.0)
            tk = self.task or {}
            self._send(t, Telemetry(vehicle_type="ugv", frame="ugv_map", position=pos, heading_rad=self._heading() if self.pose is not None else 0.0,
                                    loc_valid=self.initialised and self.est.may_translate(), loc_confidence=CONF.get(tier, 0.0) if self.initialised else 0.0,
                                    loc_detail=f"{tier} sigma={self.est.position_sigma():.2f}m" if self.initialised else "not initialised",
                                    battery_pct=round(battery_pct, 1), health="OK" if battery_pct > 5 else "FAULT", mode=self.mode,
                                    comm_ok=self.comm_ok, task_id=tk.get("id"), task_state=tk.get("state", "IDLE"),
                                    task_progress=round(tk.get("progress", 0.0), 3),
                                    failsafe={"COMM_HOLD": "comm watchdog", "LOC_LOST_HOLD": "localisation critical"}.get(self.mode, ""),
                                    extra={"fail_reason": tk.get("fail_reason", ""), "vo_ok": self.last_vo_ok}))
        if t >= self.next_map and self.initialised:
            self.next_map = t + self.cfg["comms"]["map_summary_period_s"]
            _, occupied, _, _ = self.occ.masks()
            rr, cc = np.nonzero(occupied)
            if len(rr) > 1500:
                idx = np.linspace(0, len(rr) - 1, 1500).astype(int)
                rr, cc = rr[idx], cc[idx]
            pts = [S8.grid_to_world(int(r), int(c)) for r, c in zip(rr, cc)]
            self._send(t, Observation(obs_type="MAP_SUMMARY", frame="ugv_map", data={"points": pts, "n_occupied": int(occupied.sum())}))
