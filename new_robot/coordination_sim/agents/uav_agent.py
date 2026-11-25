"""UAV onboard (companion-computer) agent = the UAV project's EXISTING autonomy classes + a coordination interface."""
from __future__ import annotations

import copy
import math
import os
import threading

import cv2
import numpy as np

from common.frames import euler_from_R, ros_twist_to_ned
from common.mav import mavlink
from companion.mavlink_client import CompanionMavlink
from mapping.geo_anchor import GpsAnchor
from mapping.pipeline import PerceptionMappingPipeline
from mission.disaster_demo import MissionController
from slam.adapter import SlamFrontEnd
from slam.orbslam3_backend import OrbSlam3Backend, resolve_backend
from slam.vo_backend import OrbPnpVO

from coordination_sim.comms.messages import Observation, Register, TaskAck, Telemetry

FAILSAFE_STATES = {"LOST_HOLD": "SLAM localisation lost (hover)", "LINK_LOST": "MAVLink link lost (hover)", "MAP_STALE": "map stale (hover)",
                   "ABORTED": "aborted (disarmed in flight)", "NO_ROUTE_HOLD": "", "VFH_BLOCKED": "", "AVOID_RETREAT": ""}


class UavAgent:
    def __init__(self, uav_cfg, mcfg, radio, K_rgb, K_th, out_dir, mavlink_mode="udp"):
        self.ucfg, self.mcfg = uav_cfg, mcfg
        self.id = mcfg["agents"]["uav"]["id"]
        self.radio = radio
        self.out_dir = out_dir
        kind, note = resolve_backend(uav_cfg["slam"]["backend"], os.environ.get("UAV_RIG_SLAM", "auto"))
        self.backend = OrbSlam3Backend(K_rgb, uav_cfg) if kind == "orbslam3" else OrbPnpVO(K_rgb, uav_cfg["slam"])
        self.backend_note = note
        self.fe = SlamFrontEnd(uav_cfg, self.backend)
        self.pipe = PerceptionMappingPipeline(uav_cfg, K_rgb, K_th, evidence_dir=os.path.join(out_dir, "evidence_uav"))
        self.anchor = GpsAnchor(uav_cfg["sensor_extrinsics"]["gps"]["xyz_body_flu_m"])
        self.mc = None
        self.sortie = 0
        self.tasks = {}
        self.cur = {"pose": None, "slam_ok": False, "slam_t": -1.0, "front": (-99.0, None), "grid_t": -99.0}
        self.seq = 0
        self.registered = False
        self.next_tel = 0.0
        self.next_map = 0.0
        self.sent_survivors = set()
        self.sent_hazards = set()
        self.last_bgr = None
        self.heading0 = None
        self.mav_mode = mavlink_mode
        self.mav = None
        self.mav_ready = False
        self.mav_hb_count = 0
        self.mav_hb_sim_t = None
        self.events = []
        self.status = {}
        self.cmd = (0.0, 0.0, 0.0, 0.0)

    def connect_mavlink(self, conn, log_path):
        """Starts the companion link in a thread (connect() waits for the FC heartbeat, which needs the sim to be stepping)."""
        self.mav = CompanionMavlink(conn, source_system=self.ucfg["mavlink"]["companion_sys_id"],
                                    source_component=self.ucfg["mavlink"]["companion_comp_id"], log_path=log_path,
                                    heartbeat_hz=self.ucfg["mavlink"]["heartbeat_hz"])

        def _c():
            self.mav.connect(timeout=60.0)
            self.mav_ready = True
        threading.Thread(target=_c, daemon=True, name="uav-mav-connect").start()

    def fc_state(self, t):
        """armed / mode / battery / link age from MAVLink telemetry; link age measured on the simulation clock."""
        tel = self.mav.telemetry() if self.mav_ready else None
        n = self.mav.msg_counts.get("HEARTBEAT", 0) if self.mav_ready else 0
        if n != self.mav_hb_count:
            self.mav_hb_count, self.mav_hb_sim_t = n, t
        age = 99.0 if self.mav_hb_sim_t is None else t - self.mav_hb_sim_t
        if tel is None:
            return False, "UNKNOWN", -1.0, age
        return tel.armed, tel.mode, float(tel.battery_pct), age

    def feed_imu(self, t, gyro, accel):
        self.fe.feed_imu(t, gyro, accel)

    def feed_gps(self, gps):
        self.anchor.add(gps.lat_deg, gps.lon_deg, gps.alt_m, gps.fix_type)

    def set_initial_heading(self, yaw_ned0):
        self.fe.set_initial_heading(yaw_ned0)
        self.heading0 = math.pi / 2 - yaw_ned0

    def on_frame(self, t, rgb, depth, thermal):
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        self.last_bgr = bgr
        pose = self.fe.process(t, bgr, depth)
        if not self.pipe.map.geo.ready:
            r = self.anchor.try_finish(self.heading0)
            if r:
                self.pipe.map.geo.set_anchor(*r)
        self.cur["slam_ok"] = pose.T_map_body is not None and pose.state == "OK"
        if self.cur["slam_ok"]:
            self.cur["pose"], self.cur["slam_t"] = pose.T_map_body, t
            if self.pipe.map.geo.ready:
                res = self.pipe.process(t, bgr, depth, thermal, pose.T_map_body)
                self.cur["front"] = (t, res.get("front_clearance_m"))

    def _new_sortie(self, t, wps):
        c = copy.deepcopy(self.ucfg)
        c["demo"]["search_waypoints_local"] = [{"name": n, "xy": list(xy)} for n, xy in wps]
        c["demo"]["return_home"] = True
        c["demo"]["end_action"] = "land"
        self.mc = MissionController(c, self.pipe.map.metric.v, self.pipe.map.metric.origin[:2])
        self.sortie += 1
        self.events.append({"t": round(t, 2), "event": "SORTIE_START", "sortie": self.sortie, "waypoints": [n for n, _ in wps]})

    def _insert(self, wps):
        mc = self.mc
        k = next((i for i, (n, _) in enumerate(mc.wps) if n.startswith("RETURN")), len(mc.wps))
        k = max(k, mc.idx)
        for j, w in enumerate(wps):
            mc.wps.insert(k + j, w)

    def _on_command(self, t, m):
        if m.command not in ("ASSIGN", "RESUME", "CANCEL"):
            return
        if m.command == "CANCEL":
            tk = self.tasks.get(m.task_id)
            if tk and self.mc:
                self.mc.wps = [w for w in self.mc.wps if not w[0].startswith(m.task_id + ":") or self.mc.wps.index(w) < self.mc.idx]
                tk["state"] = "CANCELLED"
            self._send(t, TaskAck(task_id=m.task_id, accepted=True, reason="cancelled"))
            return
        if m.task_id in self.tasks and self.tasks[m.task_id]["state"] not in ("FAILED", "CANCELLED"):
            self._send(t, TaskAck(task_id=m.task_id, accepted=True, reason="already active"))
            return
        if m.task_type == "AERIAL_SURVEY":
            wps = [(f"{m.task_id}:WP{i + 1}", tuple(g)) for i, g in enumerate(m.goal)]
        else:
            wps = [(f"{m.task_id}:CHECK", tuple(m.goal[0]))]
        state = self.mc.state if self.mc else "NONE"
        if self.mc is None or self.mc.done and state == "DONE":
            self._new_sortie(t, wps)
            how = f"new sortie #{self.sortie}"
        elif state in ("LANDING", "HOLD_END", "ABORTED"):
            self._send(t, TaskAck(task_id=m.task_id, accepted=False, reason=f"mission in {state}; available after landing"))
            return
        else:
            self._insert(wps)
            how = f"{len(wps)} waypoint(s) inserted before the return leg"
        self.tasks[m.task_id] = {"type": m.task_type, "wps": [n for n, _ in wps], "state": "ACCEPTED", "progress": 0.0, "target": m.goal[-1]}
        self.events.append({"t": round(t, 2), "event": "TASK_ACCEPTED", "task": m.task_id, "how": how})
        self._send(t, TaskAck(task_id=m.task_id, accepted=True, reason=how))

    def _update_tasks(self, t):
        if self.mc is None:
            return
        reached = {e["target"] for e in self.mc.events if e["event"] == "WAYPOINT_REACHED"}
        skipped = {e["target"] for e in self.mc.events if e["event"] == "WAYPOINT_SKIPPED"}
        cur_target = self.mc.target()[0] if self.mc.state in ("SEARCH", "RETURN") else None
        for tid, tk in self.tasks.items():
            if tk["state"] in ("COMPLETED", "FAILED", "CANCELLED"):
                continue
            n = len(tk["wps"])
            done = [w for w in tk["wps"] if w in reached or w in skipped]
            tk["progress"] = len(done) / n
            if cur_target in tk["wps"] and tk["state"] == "ACCEPTED":
                tk["state"] = "EN_ROUTE"
            if len(done) == n:
                if all(w in skipped for w in tk["wps"]):
                    tk["state"], tk["fail_reason"] = "FAILED", "waypoint(s) unreachable (skipped by the mission controller)"
                else:
                    tk["state"] = "COMPLETED"
                    if tk["type"] != "AERIAL_SURVEY" and self.last_bgr is not None:
                        p_ = os.path.join(self.out_dir, "evidence_uav", f"{tid}_check_t{int(t):03d}.png")
                        os.makedirs(os.path.dirname(p_), exist_ok=True)
                        cv2.imwrite(p_, self.last_bgr)
                        pos = self.cur["pose"][:3, 3] if self.cur["pose"] is not None else np.zeros(3)
                        rng = float(np.linalg.norm(np.array(tk["target"]) - pos[:2]))
                        res = {"task_id": tid, "evidence_image": os.path.relpath(p_, self.out_dir), "range_to_target_m": round(rng, 2),
                               "summary": f"UAV aerial visual check at {rng:.1f} m from the target, altitude {pos[2]:.1f} m, image {os.path.basename(p_)}"}
                        tk["result"] = res
                        self._send(t, Observation(obs_type="INSPECTION_RESULT", frame="uav_local", position=tuple(tk["target"]), data=res))
                self.events.append({"t": round(t, 2), "event": "TASK_" + tk["state"], "task": tid})

    def nav_step(self, t):
        for m in self.radio.receive(self.id):
            if m.kind == "TASK_COMMAND":
                self._on_command(t, m)
        armed, mode, batt, link_age = self.fc_state(t)
        self.fc = {"armed": armed, "mode": mode, "battery": batt, "link_age": link_age}
        link_ok = link_age < self.ucfg["nav"]["fc_link_timeout_s"]
        if self.mc is None:
            self.status = {"state": "IDLE_ON_PAD"}
            self._update_tasks(t)
            return
        pipe, cur = self.pipe, self.cur
        pose = cur["pose"]
        if pose is not None and pipe.map.geo.ready:
            grid = pipe.map.planning_grid(self.ucfg["demo"]["flight_altitude_m"])
            yaw = math.atan2(pose[1, 0], pose[0, 0])
            confirmed = [s.track_id for s in pipe.map.survivors.confirmed()]
            st = self.mc.step(t, pose[:3, 3], yaw, grid, pipe.map.metric.version, cur["slam_ok"], t - cur["slam_t"], armed, mode, confirmed,
                              pipe.map.geo.ready, fc_link_ok=link_ok, front_clear_m=cur["front"][1], front_age_s=t - cur["front"][0])
        else:
            st = self.mc.step(t, None, None, np.zeros((2, 2), np.int8), 0, False, 99.0, armed, mode, fc_link_ok=link_ok)
        self.status = st.status
        if self.mav_ready:
            for a in st.actions:
                if a == "ARM" and not armed:
                    self.mav.command_long(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, wait_ack=False)
                elif a == "OFFBOARD" and mode != "OFFBOARD":
                    self.mav.set_mode_message("OFFBOARD")
                elif a == "LAND" and mode != "LAND":
                    self.mav.command_long(mavlink.MAV_CMD_NAV_LAND, wait_ack=False)
            c = st.cmd
            vn, ve, vd, yr = ros_twist_to_ned("enu", (c.vx, c.vy, c.vz), c.yaw_rate, 0.0)
            self.mav.send_velocity_ned(vn, ve, vd, yr)
            self.cmd = (c.vx, c.vy, c.vz, c.yaw_rate)
        self._update_tasks(t)

    def _send(self, t, msg):
        self.seq += 1
        msg.seq, msg.stamp, msg.agent_id = self.seq, t, self.id
        self.radio.send(t, self.id, "BASE", msg)

    def mode_name(self):
        if self.mc is None:
            return "IDLE_ON_PAD"
        s = self.status.get("state", self.mc.state)
        return "LANDED_READY" if s == "DONE" else s

    def report(self, t):
        if not self.registered:
            self._send(t, Register(vehicle_type="uav", capabilities=["uav"], frame="uav_local",
                                   sensors=["rgbd_camera 640x480 @5Hz", "thermal 160x120", "imu", "gps"],
                                   software={"slam": self.backend.name, "slam_note": self.backend_note, "mission": "MissionController A*+VFH+",
                                             "fc_link": "MAVLink 2 over UDP (companion_mavlink -> MavlinkFcServer)"}))
            self.registered = True
        if t >= self.next_tel:
            self.next_tel = t + 1.0 / self.mcfg["comms"]["telemetry_hz"]
            pose = self.cur["pose"]
            fresh = self.cur["slam_ok"] and t - self.cur["slam_t"] < self.ucfg["nav"]["stale_pose_s"]
            pos = tuple(float(v) for v in pose[:3, 3]) if pose is not None else (0.0, 0.0, 0.0)
            yaw = math.atan2(pose[1, 0], pose[0, 0]) if pose is not None else 0.0
            fc = getattr(self, "fc", {"armed": False, "mode": "UNKNOWN", "battery": -1.0, "link_age": 99.0})
            mode = self.mode_name()
            fs = FAILSAFE_STATES.get(mode, "")
            if fc["mode"] in ("FAILSAFE_HOLD",):
                fs = f"FC {fc['mode']}"
            health = "FAULT" if mode == "ABORTED" or fc["mode"] == "FAILSAFE_HOLD" else ("DEGRADED" if fs else "OK")
            active = [(k, v) for k, v in self.tasks.items() if v["state"] not in ("COMPLETED", "FAILED", "CANCELLED")]
            main = active[0] if active else (list(self.tasks.items())[-1] if self.tasks else (None, {}))
            reports = [{"id": k, "state": v["state"], "progress": round(v["progress"], 3), "fail_reason": v.get("fail_reason", "")} for k, v in self.tasks.items()]
            self._send(t, Telemetry(vehicle_type="uav", frame="uav_local", position=pos, heading_rad=yaw, loc_valid=bool(fresh),
                                    loc_confidence=0.9 if fresh else 0.0, loc_detail=f"{self.backend.name} {'OK' if fresh else 'not tracking'}",
                                    battery_pct=fc["battery"], health=health, mode=mode, comm_ok=True, task_id=main[0],
                                    task_state=main[1].get("state", "IDLE"), task_progress=round(main[1].get("progress", 0.0), 3), failsafe=fs,
                                    extra={"tasks": reports, "fc_mode": fc["mode"], "armed": fc["armed"], "mavlink_link_age_s": round(fc["link_age"], 2),
                                           "sortie": self.sortie, "target": self.status.get("target", "")}))
            for s in self.pipe.map.survivors.confirmed():
                if s.track_id not in self.sent_survivors:
                    self.sent_survivors.add(s.track_id)
                    self._send(t, Observation(obs_type="SURVIVOR", frame="uav_local", position=(s.local_x, s.local_y, s.local_z),
                                              data={"track_id": int(s.track_id), "confidence": round(float(s.confidence), 2), "lat": s.latitude, "lon": s.longitude,
                                                    "sigma_m": round(float(s.position_uncertainty), 2), "observations": int(s.observation_count),
                                                    "evidence": s.evidence_frame}))
            for h in self.pipe.map.hazards.confirmed():
                if h.hazard_id not in self.sent_hazards:
                    self.sent_hazards.add(h.hazard_id)
                    self._send(t, Observation(obs_type="HAZARD", frame="uav_local", position=(h.x, h.y, 0.0),
                                              data={"hazard_id": int(h.hazard_id), "kind": h.kind, "temp_c": round(float(h.max_temp_c), 1)}))
        if t >= self.next_map and self.pipe.map.geo.ready:
            self.next_map = t + self.mcfg["comms"]["map_summary_period_s"]
            g = self.pipe.map.planning_grid(self.ucfg["demo"]["flight_altitude_m"])
            ix, iy = np.nonzero(g == 100)
            if len(ix) > 1500:
                k = np.linspace(0, len(ix) - 1, 1500).astype(int)
                ix, iy = ix[k], iy[k]
            m = self.pipe.map.metric
            pts = [(float(m.origin[0] + (a + 0.5) * m.v), float(m.origin[1] + (b + 0.5) * m.v)) for a, b in zip(ix, iy)]
            self._send(t, Observation(obs_type="MAP_SUMMARY", frame="uav_local", data={"points": pts}))

    def close(self):
        try:
            self.backend.close()
        except Exception:
            pass
        if self.mav:
            try:
                self.mav.close()
            except Exception:
                pass
