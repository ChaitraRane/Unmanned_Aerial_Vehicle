"""Base station / central coordination server."""
from __future__ import annotations

import csv
import json
import math
import os

import numpy as np

from coordination_sim.base_station.allocator import Allocator
from coordination_sim.comms.messages import Heartbeat, TaskCommand
from coordination_sim.frames import UavRegistration, UgvRegistration, point_in_polygon

TERMINAL = ("COMPLETED", "FAILED", "CANCELLED")


class Task:
    def __init__(self, d, source="operator", created_t=0.0):
        self.task_id = d["id"]
        self.name = d["name"]
        self.type = d["type"]
        self.capabilities = list(d["capabilities"])
        self.priority = float(d["priority"])
        self.preferred = d.get("preferred")
        self.target_xy = tuple(d["target_xy"]) if d.get("target_xy") else None
        self.approach_xy = tuple(d["approach_xy"]) if d.get("approach_xy") else self.target_xy
        self.waypoints = [tuple(w) for w in d.get("waypoints", [])]
        self.zone = d.get("zone")
        self.requires_clearance = bool(d.get("requires_aerial_clearance", False))
        self.source = source
        self.state = "CANDIDATE" if self.requires_clearance else "READY"
        self.assigned_to = None
        self.dispatched = False
        self.progress = 0.0
        self.note = "awaiting UAV aerial clearance of zone" if self.requires_clearance else ""
        self.history = [{"t": round(created_t, 2), "state": self.state, "note": f"created by {source}"}]
        self.created_t = created_t
        self.completed_t = None
        self.completed_by = None
        self.est_s = 0.0
        self.result = None
        self.interruptions = 0

    def first_goal_xy(self):
        return self.waypoints[0] if self.waypoints else self.approach_xy

    def last_goal_xy(self):
        return self.waypoints[-1] if self.waypoints else self.approach_xy

    def extra_path_m(self):
        w = self.waypoints
        return sum(math.hypot(w[i + 1][0] - w[i][0], w[i + 1][1] - w[i][1]) for i in range(len(w) - 1))

    def dwell_s(self):
        return 6.0 if self.type in ("GROUND_INSPECT", "VISUAL_CHECK") else 0.0

    def set_state(self, t, state, note=""):
        self.state = state
        self.note = note
        self.history.append({"t": round(t, 2), "state": state, "agent": self.assigned_to, "note": note})

    def summary(self):
        return {"id": self.task_id, "name": self.name, "type": self.type, "capabilities": self.capabilities, "priority": self.priority,
                "state": self.state, "assigned_to": self.assigned_to, "progress": round(self.progress, 3), "note": self.note,
                "source": self.source, "target_xy": self.target_xy, "approach_xy": self.approach_xy, "zone": self.zone,
                "created_t": round(self.created_t, 2), "completed_t": self.completed_t, "completed_by": self.completed_by,
                "interruptions": self.interruptions, "result": self.result, "history": self.history}


class AgentRecord:
    def __init__(self, agent_id, vtype, caps, registration):
        self.agent_id, self.vtype, self.caps = agent_id, vtype, caps
        self.reg = registration
        self.registered_t = None
        self.last = None
        self.last_stamp = -1e9
        self.telemetry_age = 1e9
        self.link = "UNKNOWN"
        self.fresh_count = 0
        self.pos_site = None
        self.pos_local = None
        self.yaw_site = None
        self.battery = -1.0
        self.loc_valid = False
        self.loc_conf = 0.0
        self.health = "UNKNOWN"
        self.mode = "UNREGISTERED"
        self.failsafe = ""
        self.queue = []
        self.completed = 0
        self.failed = 0
        self.tasks_ref = None
        self.comm_ok = True

    def available_for(self, task):
        if self.vtype == "uav":
            return self.mode not in ("LANDING", "HOLD_END", "ABORTED", "LOST_HOLD", "LINK_LOST", "MAP_STALE", "UNREGISTERED")
        return self.mode not in ("LOC_LOST_HOLD", "FAULT", "UNREGISTERED")

    def queue_end_xy(self):
        if not self.queue:
            return None
        return self.tasks_ref[self.queue[-1]].last_goal_xy()

    def reserve(self, task):
        self.queue.append(task.task_id)


class BaseStation:
    def __init__(self, cfg, radio, out_dir):
        self.cfg = cfg
        self.radio = radio
        self.ccfg = cfg["comms"]
        site = cfg["site"]
        self.boundary = [tuple(p_) for p_ in site["boundary_xy"]]
        self.zones = {z["id"]: z for z in site["zones"]}
        self.uav_reg = UavRegistration(tuple(site["uav_pad"]["xy"]))
        self.ugv_reg = UgvRegistration(tuple(site["ugv_pad"]["xy"]), math.radians(site["ugv_pad"]["yaw_enu_deg"]))
        a = cfg["agents"]
        self.agents = {a["uav"]["id"]: AgentRecord(a["uav"]["id"], "uav", a["uav"]["capabilities"], self.uav_reg),
                       a["ugv"]["id"]: AgentRecord(a["ugv"]["id"], "ugv", a["ugv"]["capabilities"], self.ugv_reg)}
        self.tasks = {}
        for td in cfg["tasks"]:
            self.tasks[td["id"]] = Task(td)
        for ag in self.agents.values():
            ag.tasks_ref = self.tasks
        self.allocator = Allocator(cfg["allocator"], cfg["agents"])
        self.seq = 0
        self.next_hb = 0.0
        self.need_alloc = True
        self.next_alloc_t = 0.0
        self.keepouts = []
        self.survivors = {}
        self.map_points = {}
        self.inspection_results = []
        xs = [p_[0] for p_ in self.boundary]
        ys = [p_[1] for p_ in self.boundary]
        self.cov_origin = (min(xs), min(ys))
        self.cov = np.zeros((int(max(ys) - min(ys)) + 1, int(max(xs) - min(xs)) + 1), np.uint8)
        self.events = []
        self.decisions = []
        os.makedirs(out_dir, exist_ok=True)
        self.out_dir = out_dir
        self._ev_csv = open(os.path.join(out_dir, "events.csv"), "w", newline="", buffering=1)
        self._ev_w = csv.writer(self._ev_csv)
        self._ev_w.writerow(["t_s", "category", "agent", "task", "message", "details_json"])
        self._tel_csv = open(os.path.join(out_dir, "telemetry_rx.csv"), "w", newline="", buffering=1)
        self._tel_w = csv.writer(self._tel_csv)
        self._tel_w.writerow(["t_rx_s", "agent", "stamp_s", "age_s", "seq", "site_x", "site_y", "z", "yaw_site_deg", "loc_valid",
                              "loc_conf", "battery_pct", "health", "mode", "comm_ok", "task", "task_state", "task_progress", "failsafe"])
        self._status_csv = open(os.path.join(out_dir, "agent_status.csv"), "w", newline="", buffering=1)
        self._status_w = csv.writer(self._status_csv)
        self._status_w.writerow(["t_s", "agent", "link", "telemetry_age_s", "site_x", "site_y", "battery_pct", "mode", "task", "queue"])
        self._dec_f = open(os.path.join(out_dir, "allocation_decisions.jsonl"), "w", buffering=1)
        self.t = 0.0
        for t_ in self.tasks.values():
            self.event(0.0, "TASK_CREATED", None, t_.task_id, f"{t_.name} ({t_.type}, priority {t_.priority}, capable: {'/'.join(t_.capabilities)})"
                       + (" - waiting for aerial clearance" if t_.state == "CANDIDATE" else ""))

    def event(self, t, category, agent, task, message, **details):
        ev = {"t": round(t, 2), "category": category, "agent": agent, "task": task, "message": message, **({"details": details} if details else {})}
        self.events.append(ev)
        self._ev_w.writerow([f"{t:.2f}", category, agent or "", task or "", message, json.dumps(details, default=float) if details else ""])
        print(f"[{t:7.2f}] {category:<22} {agent or '':<6} {task or '':<4} {message}", flush=True)
        return ev

    def _send(self, agent_id, msg):
        self.seq += 1
        msg.seq = self.seq
        msg.stamp = self.t
        return self.radio.send(self.t, "BASE", agent_id, msg)

    def to_site(self, ag, pos):
        if ag.vtype == "uav":
            return ag.reg.to_site(pos[0], pos[1])
        return ag.reg.to_site(pos[0], pos[2])

    def goal_for(self, ag, xy):
        if ag.vtype == "uav":
            return ag.reg.from_site(*xy)
        return ag.reg.from_site(*xy)

    def _on_register(self, t, m):
        ag = self.agents.get(m.agent_id)
        if ag is None:
            self.event(t, "REGISTER_REJECTED", m.agent_id, None, "unknown agent id")
            return
        ag.registered_t = t
        ag.mode = "REGISTERED"
        self.event(t, "AGENT_REGISTERED", m.agent_id, None, f"{m.vehicle_type} registered, frame {m.frame}, capabilities {m.capabilities}",
                   sensors=m.sensors, software=m.software)

    def _on_telemetry(self, t, m):
        ag = self.agents.get(m.agent_id)
        if ag is None or ag.registered_t is None:
            return
        if m.stamp <= ag.last_stamp:
            return
        first = ag.last is None
        ag.last, ag.last_stamp = m, m.stamp
        age = t - m.stamp
        ag.pos_local = m.position
        ag.pos_site = self.to_site(ag, m.position)
        ag.yaw_site = ag.reg.yaw_to_site(m.heading_rad) if ag.vtype == "uav" else ag.reg.heading_to_site(m.heading_rad)
        ag.battery, ag.loc_valid, ag.loc_conf = m.battery_pct, m.loc_valid, m.loc_confidence
        ag.health, ag.failsafe, ag.comm_ok = m.health, m.failsafe, m.comm_ok
        prev_mode = ag.mode
        ag.mode = m.mode
        if age < self.ccfg["stale_timeout_s"]:
            ag.fresh_count += 1
        self._tel_w.writerow([f"{t:.2f}", ag.agent_id, f"{m.stamp:.2f}", f"{age:.3f}", m.seq, f"{ag.pos_site[0]:.3f}", f"{ag.pos_site[1]:.3f}",
                              f"{m.position[2]:.3f}" if ag.vtype == "uav" else "0.000", f"{math.degrees(ag.yaw_site):.1f}", int(m.loc_valid),
                              f"{m.loc_confidence:.2f}", f"{m.battery_pct:.1f}", m.health, m.mode, int(m.comm_ok), m.task_id or "", m.task_state,
                              f"{m.task_progress:.3f}", m.failsafe])
        if first:
            self.event(t, "INITIAL_STATUS", ag.agent_id, None,
                       f"pos site=({ag.pos_site[0]:.1f},{ag.pos_site[1]:.1f}) battery={m.battery_pct:.0f}% loc_valid={m.loc_valid} mode={m.mode}")
        if prev_mode != m.mode and not first:
            self.event(t, "AGENT_MODE", ag.agent_id, m.task_id, f"{prev_mode} -> {m.mode}" + (f" (fail-safe: {m.failsafe})" if m.failsafe else ""))
        if ag.vtype == "uav" and m.loc_valid and ag.link == "HEALTHY":
            self._mark_coverage(ag.pos_site, ag.yaw_site, 7.0, math.radians(35), 1)
        if ag.vtype == "ugv" and m.loc_valid and ag.link == "HEALTHY":
            self._mark_coverage(ag.pos_site, ag.yaw_site, 3.0, math.radians(30), 2)
        reports = m.extra.get("tasks") or ([{"id": m.task_id, "state": m.task_state, "progress": m.task_progress,
                                             "fail_reason": m.extra.get("fail_reason", "")}] if m.task_id else [])
        for rep in reports:
            self._task_report(t, ag, rep)

    def _task_report(self, t, ag, rep):
        tid, state, prog = rep["id"], rep["state"], float(rep.get("progress", 0.0))
        if tid not in self.tasks:
            return
        task = self.tasks[tid]
        if task.assigned_to != ag.agent_id:
            if task.state not in TERMINAL and ag.link == "HEALTHY" and state not in ("COMPLETED", "FAILED", "IDLE", "CANCELLED") \
                    and task.assigned_to is not None and not getattr(task, "_cancel_sent", {}).get(ag.agent_id):
                self._send(ag.agent_id, TaskCommand(agent_id=ag.agent_id, command="CANCEL", task_id=tid,
                                                    params={"reason": f"task re-allocated to {task.assigned_to}"}))
                task._cancel_sent = {ag.agent_id: True}
                self.event(t, "TASK_CANCEL_SENT", ag.agent_id, tid, f"vehicle still holds task now owned by {task.assigned_to} -> CANCEL")
            return
        if task.state in TERMINAL:
            return
        if state != "HELD":
            task.progress = max(task.progress, prog)
        if state in ("EN_ROUTE", "INSPECTING", "ACCEPTED") and task.state == "ASSIGNED":
            task.set_state(t, "ACTIVE", f"{ag.agent_id} executing ({state})")
            self.event(t, "TASK_ACTIVE", ag.agent_id, tid, f"{ag.agent_id} reports {state}")
        if state == "COMPLETED":
            self._complete(t, ag, task)
        elif state == "FAILED":
            ag.failed += 1
            self._release(t, ag, task, "READY", f"failed on {ag.agent_id}: {rep.get('fail_reason', '')}")
            self.event(t, "TASK_FAILED", ag.agent_id, tid, rep.get("fail_reason", ""))

    def _on_observation(self, t, m):
        ag = self.agents.get(m.agent_id)
        if ag is None:
            return
        oc = self.cfg["observation_tasks"]
        if m.obs_type == "SURVIVOR":
            tid = m.data["track_id"]
            xy = self.to_site(ag, m.position)
            if tid in self.survivors:
                self.survivors[tid] = xy
                return
            self.survivors[tid] = xy
            dup = [t_ for t_ in self.tasks.values() if t_.target_xy and math.hypot(t_.target_xy[0] - xy[0], t_.target_xy[1] - xy[1]) < oc["duplicate_radius_m"]
                   and t_.type == oc["survivor"]["type"]]
            self.event(t, "OBSERVATION", ag.agent_id, None, f"UAV confirmed person (track #{tid}) at site ({xy[0]:.1f},{xy[1]:.1f}), "
                       f"lat {m.data.get('lat')}, lon {m.data.get('lon')}, sigma {m.data.get('sigma_m')} m", **m.data)
            if dup:
                self.event(t, "DUPLICATE_SUPPRESSED", ag.agent_id, dup[0].task_id, f"person #{tid} within {oc['duplicate_radius_m']} m of existing task")
                return
            ugv = next(a_ for a_ in self.agents.values() if a_.vtype == "ugv")
            ref = ugv.pos_site or self.ugv_reg.pad_xy
            d = np.array(ref) - np.array(xy)
            d = d / max(np.linalg.norm(d), 1e-6)
            approach = tuple(np.array(xy) + oc["survivor"]["standoff_m"] * d)
            n = 1 + max([int(k[1:]) for k in self.tasks if k[1:].isdigit()] + [0])
            task = Task({"id": f"T{n}", "name": f"Casualty check: person #{tid} reported by UAV", "type": oc["survivor"]["type"],
                         "capabilities": oc["survivor"]["capabilities"], "priority": oc["survivor"]["priority"], "target_xy": xy,
                         "approach_xy": approach, "zone": self._zone_of(xy)}, source=f"{ag.agent_id} observation", created_t=t)
            self.tasks[task.task_id] = task
            self.need_alloc = True
            self.event(t, "TASK_CREATED", ag.agent_id, task.task_id, f"{task.name} -> {task.type} priority {task.priority}, approach ({approach[0]:.1f},{approach[1]:.1f})")
        elif m.obs_type == "HAZARD":
            xy = self.to_site(ag, m.position)
            if any(math.hypot(k["xy"][0] - xy[0], k["xy"][1] - xy[1]) < 2.0 for k in self.keepouts):
                return
            r = self.cfg["observation_tasks"]["hazard"]["keepout_radius_m"]
            self.keepouts.append({"xy": xy, "radius_m": r, "source": ag.agent_id, "t": round(t, 2), "kind": m.data.get("kind")})
            self.event(t, "OBSERVATION", ag.agent_id, None, f"UAV thermal hazard ({m.data.get('kind')}) at site ({xy[0]:.1f},{xy[1]:.1f}) -> UGV keep-out r={r} m",
                       **m.data)
            ugv = next(a_ for a_ in self.agents.values() if a_.vtype == "ugv")
            self._send(ugv.agent_id, TaskCommand(agent_id=ugv.agent_id, command="KEEPOUT", goal_frame="ugv_map",
                                                 goal=[self.goal_for(ugv, xy)], params={"radius_m": r}))
        elif m.obs_type == "INSPECTION_RESULT":
            self.inspection_results.append({"t": round(t, 2), "agent": ag.agent_id, **m.data})
            if m.data.get("task_id") in self.tasks:
                self.tasks[m.data["task_id"]].result = m.data
            self.event(t, "INSPECTION_RESULT", ag.agent_id, m.data.get("task_id"), m.data.get("summary", ""))
        elif m.obs_type == "MAP_SUMMARY":
            pts = np.asarray(m.data.get("points", []), float).reshape(-1, 2)
            if len(pts):
                if ag.vtype == "uav":
                    site = pts + np.array(self.uav_reg.pad_xy)
                else:
                    site = np.array([self.ugv_reg.to_site(X, Z) for X, Z in pts])
                self.map_points[ag.agent_id] = site

    def _on_ack(self, t, m):
        task = self.tasks.get(m.task_id)
        if task is None:
            return
        if m.accepted:
            self.event(t, "TASK_ACK", m.agent_id, m.task_id, "accepted" + (f" ({m.reason})" if m.reason else ""))
        else:
            ag = self.agents[m.agent_id]
            self._release(t, ag, task, "READY", f"rejected by {m.agent_id}: {m.reason}")
            self.event(t, "TASK_REJECTED", m.agent_id, m.task_id, m.reason)

    def _zone_of(self, xy):
        for z in self.zones.values():
            if abs(xy[0] - z["center"][0]) <= z["half"][0] and abs(xy[1] - z["center"][1]) <= z["half"][1]:
                return z["id"]
        return None

    def _release(self, t, ag, task, state, note):
        if task.task_id in ag.queue:
            ag.queue.remove(task.task_id)
        task.assigned_to = None
        task.dispatched = False
        task.set_state(t, state, note)
        self.need_alloc = True

    def _complete(self, t, ag, task):
        task.progress = 1.0
        task.completed_t = round(t, 2)
        task.completed_by = ag.agent_id
        task.set_state(t, "COMPLETED", f"completed by {ag.agent_id}")
        if task.task_id in ag.queue:
            ag.queue.remove(task.task_id)
        ag.completed += 1
        self.event(t, "TASK_COMPLETED", ag.agent_id, task.task_id, f"{task.name} completed in {t - task.history[0]['t']:.1f} s since creation")
        self.need_alloc = True
        self._next_task_decision(t, ag)

    def _next_task_decision(self, t, ag):
        """After a completion: rank what this agent could do next (recommendation logged for the operator)."""
        cands = [tk for tk in self.tasks.values() if tk.state in ("READY",) or (tk.state == "ASSIGNED" and tk.assigned_to == ag.agent_id and not tk.dispatched)]
        ranked = []
        for tk in cands:
            ev = self.allocator.evaluate(tk, ag, 0.0) if tk.assigned_to is None else {"feasible": True, "utility": None, "reason": "already queued"}
            ranked.append({"task": tk.task_id, "feasible": ev["feasible"], "utility": ev.get("utility"), "reason": ev.get("reason", "")})
        ranked.sort(key=lambda r: (not r["feasible"], -(r["utility"] or 0)))
        queued = [q for q in ag.queue]
        rec = queued[0] if queued else next((r["task"] for r in ranked if r["feasible"]), None)
        cand_txt = ", ".join(f"{r['task']}:{'%.2f' % r['utility'] if r['utility'] is not None else r['reason'] or '-'}" for r in ranked) or "none"
        self.event(t, "NEXT_TASK_DECISION", ag.agent_id, rec, f"recommended next for {ag.agent_id}: {rec or 'none (idle / hold)'}; candidates [{cand_txt}]",
                   ranked=ranked, queued=queued)

    def queued_s(self, ag):
        s = 0.0
        for tid in ag.queue:
            tk = self.tasks[tid]
            s += (1.0 - tk.progress) * tk.est_s
        return s

    def _monitor_links(self, t):
        st, lt = self.ccfg["stale_timeout_s"], self.ccfg["lost_timeout_s"]
        for ag in self.agents.values():
            if ag.registered_t is None:
                continue
            if ag.last is None:
                if t - ag.registered_t > lt and ag.link != "LOST":
                    ag.link = "LOST"
                    self.event(t, "LINK_LOST", ag.agent_id, None, f"no telemetry since registration ({t - ag.registered_t:.1f} s)")
                continue
            ag.telemetry_age = t - ag.last_stamp
            prev = ag.link
            if ag.telemetry_age > lt:
                new = "LOST"
            elif ag.telemetry_age > st:
                new = "STALE"
            elif prev in ("STALE", "LOST") and ag.fresh_count < self.ccfg["recover_fresh_msgs"]:
                new = prev
            else:
                new = "HEALTHY"
            if ag.telemetry_age > st:
                ag.fresh_count = 0
            if new != prev:
                ag.link = new
                if new in ("STALE", "LOST") and prev in ("HEALTHY", "UNKNOWN"):
                    self.event(t, "FAULT_DETECTED", ag.agent_id, ag.queue[0] if ag.queue else None,
                               f"telemetry stale: last message {ag.telemetry_age:.2f} s old > timeout {st} s -> {ag.agent_id} marked UNAVAILABLE; "
                               f"last known position ({ag.pos_site[0]:.1f},{ag.pos_site[1]:.1f}) is NOT treated as current",
                               telemetry_age_s=round(ag.telemetry_age, 3), last_stamp=round(ag.last_stamp, 3))
                    self._handle_unavailable(t, ag)
                elif new == "LOST":
                    self.event(t, "LINK_LOST", ag.agent_id, None, f"no telemetry for {ag.telemetry_age:.1f} s (> {lt} s)")
                elif new == "HEALTHY" and prev in ("STALE", "LOST"):
                    self.event(t, "LINK_RECOVERED", ag.agent_id, None, f"{ag.fresh_count} consecutive fresh telemetry messages; "
                               f"vehicle reports mode {ag.mode}, task {ag.last.task_id} ({ag.last.task_state}) -> agent AVAILABLE again")
                    self.need_alloc = True
                elif new == "HEALTHY":
                    self.event(t, "LINK_HEALTHY", ag.agent_id, None, "telemetry fresh")
                    self.need_alloc = True

    def _handle_unavailable(self, t, ag):
        for tid in list(ag.queue):
            task = self.tasks[tid]
            was = task.state
            task.interruptions += 1 if was == "ACTIVE" else 0
            self._release(t, ag, task, "READY", f"interrupted: {ag.agent_id} link {ag.link} (was {was} at {task.progress * 100:.0f}%)")
            self.event(t, "TASK_INTERRUPTED" if was == "ACTIVE" else "TASK_UNQUEUED", ag.agent_id, tid,
                       f"{task.name}: {was} -> PENDING (progress kept {task.progress * 100:.0f}%); stop issuing work to {ag.agent_id}")
        self.need_alloc = True

    def _mark_coverage(self, xy, yaw, rng, half_ang, bit):
        ox, oy = self.cov_origin
        H, W = self.cov.shape
        cx, cy = xy
        for gy in range(max(0, int(cy - oy - rng)), min(H, int(cy - oy + rng) + 1)):
            for gx in range(max(0, int(cx - ox - rng)), min(W, int(cx - ox + rng) + 1)):
                px, py = ox + gx + 0.5, oy + gy + 0.5
                dx, dy = px - cx, py - cy
                d = math.hypot(dx, dy)
                if d <= 1.0 or (d <= rng and abs((math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi) <= half_ang):
                    self.cov[gy, gx] |= bit

    def zone_coverage(self, zid, bit=1):
        z = self.zones[zid]
        ox, oy = self.cov_origin
        x0, x1 = int(z["center"][0] - z["half"][0] - ox), int(z["center"][0] + z["half"][0] - ox)
        y0, y1 = int(z["center"][1] - z["half"][1] - oy), int(z["center"][1] + z["half"][1] - oy)
        patch = self.cov[max(0, y0):y1 + 1, max(0, x0):x1 + 1]
        return float(np.mean((patch & bit) > 0)) if patch.size else 0.0

    def _release_cleared(self, t):
        for task in self.tasks.values():
            if task.state == "CANDIDATE" and task.zone:
                c = self.zone_coverage(task.zone)
                if c >= 0.35:
                    task.set_state(t, "READY", f"zone {task.zone} aerially covered ({c * 100:.0f}%)")
                    self.event(t, "TASK_RELEASED", None, task.task_id, f"UAV aerial coverage of {task.zone} = {c * 100:.0f}% -> task released for allocation")
                    self.need_alloc = True

    def _allocate(self, t):
        incumbents = {}
        for ag in self.agents.values():
            for tid in list(ag.queue):
                tk = self.tasks[tid]
                if not tk.dispatched and tk.state == "ASSIGNED":
                    ag.queue.remove(tid)
                    incumbents[tid] = ag.agent_id
        ready = [tk for tk in self.tasks.values() if tk.state == "READY"] + [self.tasks[k] for k in incumbents]
        if not ready:
            return
        assignments, decisions = self.allocator.allocate(ready, self.agents, self.queued_s, incumbents)
        placed = set()
        for task, aid, ev in assignments:
            placed.add(task.task_id)
            prev = incumbents.get(task.task_id)
            task.est_s = ev["eta_s"]
            if prev == aid:
                continue
            if getattr(task, "last_agent", None) not in (None, aid):
                task.progress = 0.0
            task.last_agent = aid
            task.assigned_to = aid
            reassigned = prev is not None or any(h.get("note", "").startswith("interrupted") for h in task.history)
            task.set_state(t, "ASSIGNED", f"to {aid}, utility {ev['utility']:.2f}, ETA {ev['eta_s']:.0f} s")
            others = [c for c in decisions if c["task"] == task.task_id][0]["candidates"]
            why = "; ".join(f"{c['agent']}={c['utility']:.2f}" if c["feasible"] else f"{c['agent']} infeasible ({c['reason']})" for c in others)
            self.event(t, "TASK_REASSIGNED" if reassigned else "TASK_ALLOCATED", aid, task.task_id,
                       f"{task.name} -> {aid}" + (f" (was {prev})" if prev else "") +
                       f" (utility {ev['utility']:.2f}, ETA {ev['eta_s']:.0f} s, energy {ev['energy_pct']:.1f}%) | {why}",
                       terms=ev["terms"], candidates=others)
        for tid, prev in incumbents.items():
            if tid not in placed:
                tk = self.tasks[tid]
                tk.assigned_to = None
                tk.set_state(t, "READY", f"un-queued from {prev} in re-auction")
        for d in decisions:
            d["t"] = round(t, 2)
            self._dec_f.write(json.dumps(d, default=float) + "\n")
            if d["chosen"] is None:
                task = self.tasks[d["task"]]
                msg = f"no feasible agent -> stays PENDING: {d['reason']}"
                if task.note != msg:
                    task.note = msg
                    task.history.append({"t": round(t, 2), "state": "READY", "note": msg})
                    self.event(t, "TASK_PENDING", None, task.task_id, f"{task.name}: {msg}")
        self.decisions.extend(decisions)

    def _dispatch(self, t):
        for ag in self.agents.values():
            if ag.link != "HEALTHY":
                continue
            for i, tid in enumerate(ag.queue):
                task = self.tasks[tid]
                if task.dispatched:
                    continue
                concurrent = ag.vtype == "uav"
                if i > 0 and not concurrent:
                    head = self.tasks[ag.queue[0]]
                    if head.state not in TERMINAL:
                        break
                if task.waypoints:
                    goal = [self.goal_for(ag, w) for w in task.waypoints]
                else:
                    goal = [self.goal_for(ag, task.approach_xy), self.goal_for(ag, task.target_xy or task.approach_xy)]
                params = {"name": task.name, "dwell_s": self.cfg["agents"]["ugv"]["inspect_dwell_s"] if ag.vtype == "ugv" else 2.0}
                if ag.vtype == "ugv":
                    params["keepouts"] = [{"xy": self.goal_for(ag, k["xy"]), "radius_m": k["radius_m"]} for k in self.keepouts]
                resume = ag.last is not None and ag.last.task_id == tid and ag.last.task_state == "HELD"
                self._send(ag.agent_id, TaskCommand(agent_id=ag.agent_id, command="RESUME" if resume else "ASSIGN", task_id=tid, task_type=task.type,
                                                    priority=task.priority, goal_frame="uav_local" if ag.vtype == "uav" else "ugv_map",
                                                    goal=goal, params=params))
                task.dispatched = True
                self.event(t, "TASK_DISPATCHED", ag.agent_id, tid, f"{'RESUME' if resume else 'ASSIGN'} sent; goal in {ag.agent_id} frame: "
                           + ", ".join(f"({g[0]:.1f},{g[1]:.1f})" for g in goal[:3]) + (" ..." if len(goal) > 3 else ""))
                if not concurrent:
                    break

    def step(self, t):
        self.t = t
        for m in self.radio.receive("BASE"):
            h = {"REGISTER": self._on_register, "TELEMETRY": self._on_telemetry, "OBSERVATION": self._on_observation,
                 "TASK_ACK": self._on_ack}.get(m.kind)
            if h:
                h(t, m)
        if t >= self.next_hb:
            self.next_hb = t + 1.0 / self.ccfg["heartbeat_hz"]
            for ag in self.agents.values():
                if ag.registered_t is not None:
                    self._send(ag.agent_id, Heartbeat())
        self._monitor_links(t)
        self._release_cleared(t)
        if (self.need_alloc or t >= self.next_alloc_t) and any(a.link == "HEALTHY" for a in self.agents.values()):
            self.need_alloc = False
            self.next_alloc_t = t + 2.0
            self._allocate(t)
        self._dispatch(t)

    def log_status(self, t):
        for ag in self.agents.values():
            self._status_w.writerow([f"{t:.2f}", ag.agent_id, ag.link, f"{min(ag.telemetry_age, 9999):.2f}",
                                     f"{ag.pos_site[0]:.2f}" if ag.pos_site else "", f"{ag.pos_site[1]:.2f}" if ag.pos_site else "",
                                     f"{ag.battery:.1f}", ag.mode, ag.queue[0] if ag.queue else "", "|".join(ag.queue)])

    def snapshot(self):
        return {"t": self.t, "agents": self.agents, "tasks": self.tasks, "events": self.events[-14:], "keepouts": self.keepouts,
                "map_points": self.map_points, "coverage": self.cov, "cov_origin": self.cov_origin, "boundary": self.boundary,
                "zones": self.zones, "survivors": self.survivors}

    def close(self):
        for f in (self._ev_csv, self._tel_csv, self._status_csv, self._dec_f):
            f.close()
