"""Base station + AI task coordinator: registration, allocation, classification, handover, fault handling, verification."""
from __future__ import annotations

import math

import numpy as np

from p3.frames import MISSION, UGV_ODOM

STATES = ("PENDING", "ASSIGNED", "ACCEPTED", "IN_PROGRESS", "COMPLETED", "FAILED", "HOLD")


class Task:
    def __init__(self, d):
        self.task_id, self.target, self.action = d["task_id"], d["target"], d["action"]
        self.requires, self.priority = d["requires"], d["priority"]
        self.agent = None
        self.status = "PENDING"
        self.progress = 0.0
        self.target_mission = None
        self.results = None
        self.history = []


class Coordinator:
    def __init__(self, cfg, bus, frames, log):
        self.cfg, self.bus, self.frames, self.log = cfg, bus, frames, log
        self.tasks = {d["task_id"]: Task(d) for d in cfg["tasks"]}
        self.caps = {a: set(v["capabilities"]) for a, v in cfg["agents"].items()}
        self.agents = {}
        self.targets = {}
        self.stage = "INIT"
        self.events = []
        self.handover = None
        self.final = None
        self.done = False
        self.final_replies = {}

    def _event(self, t, kind, text, **kw):
        e = {"t": round(t, 2), "kind": kind, "text": text, **kw}
        self.events.append(e)
        self.log(f"[{kind}] {text}")

    def _set(self, t, task, status, why=""):
        if task.status != status:
            task.history.append({"t": round(t, 2), "from": task.status, "to": status, "why": why})
            self._event(t, "TASK", f"{task.task_id} {task.status} -> {status}" + (f" ({why})" if why else ""))
            task.status = status

    def _assign(self, t, task, agent, extra=None):
        task.agent = agent
        body = {"task_id": task.task_id, "target": task.target, "assign_to": agent, "priority": task.priority,
                "action": task.action, "frame_id": MISSION, "status": "ASSIGNED"}
        if task.target_mission is not None:
            body["target_xyz"] = task.target_mission
        body.update(extra or {})
        self.bus.send("BASE", agent, "TASK_ASSIGNMENT", body)
        self._set(t, task, "ASSIGNED", f"to {agent}")

    def available(self, agent, t):
        a = self.agents.get(agent)
        if a is None:
            return False, "not registered"
        if t - a["last_seen"] > self.cfg["comms"]["timeout_s"]:
            return False, "communication timeout"
        if a["health"] != "OK":
            return False, f"health {a['health']}"
        if a["battery_pct"] < 25:
            return False, "battery low"
        if a["localisation"] != "VALID":
            return False, "localisation invalid"
        return True, "ok"

    def capable(self, agent, task):
        return task.requires in self.caps[agent]

    def in_area(self, xyz):
        (x0, x1), (y0, y1) = self.cfg["area"]["x_m"], self.cfg["area"]["y_m"]
        return x0 <= xyz[0] <= x1 and y0 <= xyz[1] <= y1

    def classify(self, t, tgt):
        z = tgt["mission_xyz"][2]
        elevated = z >= self.cfg["classification"]["elevated_min_z_m"]
        agent = "UAV" if elevated else "UGV"
        action = self.cfg["classification"]["marker_task"].get(tgt["marker"])
        task = next((k for k in self.tasks.values() if k.action == action), None)
        tgt.update(category="AERIAL/ELEVATED" if elevated else "GROUND", agent=agent, task=task.task_id if task else None)
        self._event(t, "CLASSIFY", f"target {tgt['marker']} at mission ({tgt['mission_xyz'][0]:.1f}, {tgt['mission_xyz'][1]:.1f}, "
                    f"{z:.1f}) -> {tgt['category']} -> {agent} task {tgt['task']}", target=tgt["marker"])
        if task is not None and task.target_mission is None:
            task.target_mission = list(np.round(tgt["mission_xyz"], 2))
            task.source = tgt

    def update(self, t):
        for m in self.bus.take("BASE"):
            self._handle(t, m)
        self._watch_links(t)
        self._plan(t)

    def _handle(self, t, m):
        src, mt = m["src"], m["msg_type"]
        if mt in ("REGISTER", "STATUS"):
            new = src not in self.agents
            prev = self.agents.get(src, {})
            self.agents[src] = {**prev, **{k: m[k] for k in ("agent_id", "pose", "frame_id", "battery_pct", "health",
                                                              "localisation", "current_task", "comm") if k in m},
                                "last_seen": t, "link": "OK"}
            if new:
                self._event(t, "REGISTER", f"{src} registered: pose {m['pose']} [{m['frame_id']}], battery {m['battery_pct']}%, "
                            f"health {m['health']}, localisation {m['localisation']}")
            elif prev.get("link") == "TIMEOUT":
                self._event(t, "LINK", f"{src} link restored")
                for k in self.tasks.values():
                    if k.agent == src and k.status == "HOLD":
                        self._set(t, k, "IN_PROGRESS", "link restored, task resumed by same agent")
            if mt == "STATUS" and self.stage == "VERIFY":
                self.final_replies[src] = m
        elif mt == "TASK_ACK":
            k = self.tasks[m["task_id"]]
            if m["status"] == "ACCEPTED":
                self._set(t, k, "ACCEPTED", m.get("note", ""))
                if self.handover and self.handover["task_id"] == k.task_id:
                    self.handover["ack_t"] = t
                    self.handover["ack"] = m
                    self._event(t, "HANDOVER", f"UGV acknowledged {k.task_id}: goal in {m['goal_frame']} {m['goal_xyz']} accepted by navigation")
            else:
                self._set(t, k, "FAILED", m.get("note", "rejected"))
        elif mt == "TASK_PROGRESS":
            k = self.tasks[m["task_id"]]
            k.progress = m["progress"]
            if k.status in ("ASSIGNED", "ACCEPTED"):
                self._set(t, k, "IN_PROGRESS")
        elif mt == "TARGET_REPORT":
            key = m["marker"]
            if key in self.targets:
                return
            src_xyz = m["xyz"]
            mission_xyz = self.frames.convert(src_xyz, m["frame_id"], MISSION)
            ok = all(math.isfinite(v) for v in mission_xyz) and m["sigma_m"] <= self.cfg["handover"]["max_target_sigma_m"] \
                and m["n_obs"] >= self.cfg["handover"]["min_observations"] and self.in_area(mission_xyz)
            tgt = {"marker": key, "src_frame": m["frame_id"], "src_xyz": src_xyz, "mission_xyz": mission_xyz,
                   "sigma_m": m["sigma_m"], "n_obs": m["n_obs"], "image": m.get("image"), "t": t, "valid": ok}
            self._event(t, "FRAME", f"{key}: {m['frame_id']} {np.round(src_xyz, 2).tolist()} -> {MISSION} "
                        f"{np.round(mission_xyz, 2).tolist()} (valid={ok})")
            if ok:
                self.targets[key] = tgt
                self.classify(t, tgt)
        elif mt == "TASK_COMPLETE":
            k = self.tasks[m["task_id"]]
            if m["status"] == "COMPLETED" and m.get("results", {}).get("inspection_performed"):
                k.results = m["results"]
                k.progress = 1.0
                self._set(t, k, "COMPLETED", "valid completion message")
            else:
                self._set(t, k, "FAILED", m.get("note", "completion not verified"))

    def _watch_links(self, t):
        for a, s in self.agents.items():
            if s.get("link") == "OK" and t - s["last_seen"] > self.cfg["comms"]["timeout_s"]:
                s["link"] = "TIMEOUT"
                self._event(t, "FAULT", f"{a} communication timeout ({t - s['last_seen']:.1f}s without status)")
                for k in self.tasks.values():
                    if k.agent == a and k.status in ("ASSIGNED", "ACCEPTED", "IN_PROGRESS"):
                        self._set(t, k, "HOLD", "Condition D: agent unavailable")
                        alt = "UAV" if a == "UGV" else "UGV"
                        okc = self.capable(alt, k)
                        oka, why = self.available(alt, t)
                        self._event(t, "REASSIGN", f"{k.task_id}: alternative {alt} capable={okc} available={oka} ({why}) -> "
                                    + ("reassigned" if okc and oka else "kept on HOLD for " + a))

    def _plan(self, t):
        T = self.tasks
        if self.stage == "INIT":
            if all(a in self.agents for a in ("UAV", "UGV")):
                ok = {a: self.available(a, t) for a in ("UAV", "UGV")}
                self._event(t, "CHECK", "availability " + ", ".join(f"{a}={v[0]} ({v[1]})" for a, v in ok.items()))
                if all(v[0] for v in ok.values()):
                    self._assign(t, T["T1"], "UAV", {"survey": "lawnmower", "area": self.cfg["area"]})
                    self.stage = "SURVEY"
        elif self.stage == "SURVEY":
            if T["T1"].status == "COMPLETED":
                self._event(t, "PLAN", "survey complete; inspection list: " + ", ".join(
                    f"{v['task']}->{v['agent']}" for v in self.targets.values()))
                for k in T.values():
                    if k.task_id in ("T2", "T3", "T4", "T5") and k.target_mission is None:
                        self._set(t, k, "FAILED", "target not identified in survey")
                self.stage = "UAV_INSPECT"
                if T["T2"].status == "PENDING" and self.available("UAV", t)[0] and self.capable("UAV", T["T2"]):
                    self._assign(t, T["T2"], "UAV")
        elif self.stage == "UAV_INSPECT":
            if T["T2"].status in ("COMPLETED", "FAILED"):
                self._event(t, "PLAN", f"Condition B: T2 {T['T2'].status}; next pending task -> UGV ground inspection")
                self.bus.send("BASE", "UAV", "COMMAND", {"command": "RETURN_AND_STANDBY", "frame_id": MISSION})
                self.stage = "GROUND"
        elif self.stage == "GROUND":
            busy = any(k.agent == "UGV" and k.status in ("ASSIGNED", "ACCEPTED", "IN_PROGRESS", "HOLD") for k in T.values())
            if not busy:
                nxt = self.next_ground_task()
                if nxt is None:
                    self.stage = "VERIFY"
                    self.verify_t = t
                    T["T6"].agent = "UAV+UGV"
                    self._set(t, T["T6"], "ASSIGNED", "joint verification")
                    for a in ("UAV", "UGV"):
                        self.bus.send("BASE", a, "FINAL_STATUS_REQUEST", {"task_id": "T6", "frame_id": MISSION})
                    self._set(t, T["T6"], "IN_PROGRESS")
                else:
                    ok, why = self.available("UGV", t)
                    if ok and self.capable("UGV", nxt):
                        self.handover_task(t, nxt)
        elif self.stage == "VERIFY":
            if len(self.final_replies) == 2 or t - self.verify_t > 15:
                self.finish(t)

    def next_ground_task(self):
        pend = [k for k in self.tasks.values() if k.task_id in ("T3", "T4", "T5") and k.status == "PENDING"
                and k.target_mission is not None]
        if not pend:
            return None
        ugv = self.agents["UGV"]["pose"]
        pend.sort(key=lambda k: (k.priority, math.hypot(k.target_mission[0] - ugv[0], k.target_mission[1] - ugv[1])))
        return pend[0]

    def handover_task(self, t, k):
        src = k.source
        odom_xyz = self.frames.convert(k.target_mission, MISSION, UGV_ODOM)
        self.handover = {"task_id": k.task_id, "t": t, "target": src["marker"], "uav_frame": src["src_frame"],
                         "uav_xyz": src["src_xyz"], "mission_xyz": k.target_mission, "ugv_frame": UGV_ODOM,
                         "ugv_xyz": list(np.round(odom_xyz, 3))}
        self._event(t, "HANDOVER", f"Condition A/C: {k.task_id} target {src['marker']} UAV[{src['src_frame']}] "
                    f"{np.round(src['src_xyz'], 2).tolist()} -> mission {k.target_mission} -> UGV[{UGV_ODOM}] "
                    f"{np.round(odom_xyz, 2).tolist()}", handover=self.handover)
        self._assign(t, k, "UGV", {"target_xyz_ugv_odom": odom_xyz, "ugv_frame_id": UGV_ODOM,
                                   "observations": {"marker": src["marker"], "n_obs": src["n_obs"],
                                                    "sigma_m": src["sigma_m"], "image": src["image"]},
                                   "handover_from": "UAV", "requires_ack": True})
        self.handovers = getattr(self, "handovers", []) + [self.handover]

    def finish(self, t):
        ok = all(self.tasks[k].status == "COMPLETED" for k in ("T1", "T2", "T3", "T4", "T5"))
        T6 = self.tasks["T6"]
        T6.results = {"inspection_performed": ok, "replies": list(self.final_replies)}
        self._set(t, T6, "COMPLETED" if ok and len(self.final_replies) == 2 else "FAILED",
                  "all inspections verified" if ok else "incomplete tasks reported")
        for a in ("UAV", "UGV"):
            self.bus.send("BASE", a, "COMMAND", {"command": "END_MISSION_SAFE", "frame_id": MISSION})
        self.stage = "DONE"
        self.done = True
        self._event(t, "MISSION", "mission " + ("COMPLETED" if T6.status == "COMPLETED" else "INCOMPLETE"))
