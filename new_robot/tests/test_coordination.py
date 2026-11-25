"""Fast tests of the coordination layer (no simulator):  python -m pytest -q tests
  * frame registration round trips (UAV local / UGV map <-> site)
  * allocator feasibility rules and utility ranking
  * base station: stale-telemetry detection after the configured timeout, interruption, ground-only task stays
    PENDING (never given to the UAV), both-capable task re-assigned to the healthy UAV, recovery + RESUME
  * out-of-order telemetry never overwrites newer state
  * ground-truth firewall: autonomy / coordination modules do not import the simulator
"""
import ast
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import yaml  # noqa: E402

from coordination_sim.base_station.station import BaseStation  # noqa: E402
from coordination_sim.comms.messages import Register, Telemetry  # noqa: E402
from coordination_sim.comms.radio import RadioNetwork  # noqa: E402
from coordination_sim.frames import UavRegistration, UgvRegistration, point_in_polygon  # noqa: E402


def cfg():
    with open(os.path.join(ROOT, "config", "mission.yaml"), encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_frames_round_trip():
    u = UavRegistration((0.0, -3.0))
    assert u.to_site(*u.from_site(4.0, 7.0)) == (4.0, 7.0)
    g = UgvRegistration((3.4, -3.6), math.radians(90))
    X, Z = g.from_site(5.0, 10.0)
    x, y = g.to_site(X, Z)
    assert abs(x - 5.0) < 1e-9 and abs(y - 10.0) < 1e-9
    X, Z = g.from_site(3.4 + 1.0, -3.35 + 2.0)
    assert abs(X - 1.0) < 1e-9 and abs(Z - 2.0) < 1e-9
    assert abs(g.heading_to_site(0.0) - math.radians(90)) < 1e-9
    assert abs(g.heading_to_site(math.radians(90)) - 0.0) < 1e-9
    assert point_in_polygon(0, 0, [(-1, -1), (1, -1), (1, 1), (-1, 1)]) and not point_in_polygon(2, 0, [(-1, -1), (1, -1), (1, 1), (-1, 1)])


class Harness:
    def __init__(self, tmp_path):
        self.c = cfg()
        self.c["comms"]["latency_s"] = 0.0
        self.c["comms"]["jitter_s"] = 0.0
        self.radio = RadioNetwork(self.c["comms"], 0)
        self.bs = BaseStation(self.c, self.radio, str(tmp_path))
        self.t = 0.0
        self.seq = {"UAV-1": 0, "UGV-1": 0}
        for aid, vt, fr in (("UAV-1", "uav", "uav_local"), ("UGV-1", "ugv", "ugv_map")):
            self.radio.send(0.0, aid, "BASE", Register(agent_id=aid, vehicle_type=vt, capabilities=[vt], frame=fr))
        self.tick()

    def tel(self, aid, **kw):
        self.seq[aid] += 1
        d = dict(agent_id=aid, vehicle_type="uav" if aid.startswith("UAV") else "ugv", stamp=self.t, seq=self.seq[aid],
                 frame="uav_local" if aid.startswith("UAV") else "ugv_map", position=(0.0, 0.0, 2.0), loc_valid=True, loc_confidence=0.9,
                 battery_pct=90.0, mode="SEARCH" if aid.startswith("UAV") else "IDLE")
        d.update(kw)
        self.radio.send(self.t, aid, "BASE", Telemetry(**d))

    def tick(self, dt=0.0):
        self.t += dt
        self.radio.step(self.t)
        self.bs.step(self.t)

    def run(self, seconds, uav=True, ugv=True, ugv_kw=None, uav_kw=None):
        n = int(seconds / 0.5)
        for _ in range(n):
            if uav:
                self.tel("UAV-1", **(uav_kw or {}))
            if ugv:
                self.tel("UGV-1", **(ugv_kw or {}))
            self.tick(0.5)


def test_initial_allocation_respects_capabilities(tmp_path):
    h = Harness(tmp_path)
    h.run(2.0)
    T = h.bs.tasks
    assert T["T1"].assigned_to == "UAV-1"
    assert T["T2"].assigned_to == "UGV-1"
    assert T["T3"].assigned_to in ("UGV-1", "UAV-1")
    assert T["T4"].state == "CANDIDATE"
    assert not any(t.assigned_to and t.type == "GROUND_INSPECT" and t.assigned_to.startswith("UAV") for t in T.values())


def test_stale_telemetry_detection_and_safe_reallocation(tmp_path):
    h = Harness(tmp_path)
    h.run(2.0)
    bs = h.bs
    h.run(2.0, ugv_kw={"task_id": "T2", "task_state": "EN_ROUTE", "task_progress": 0.4, "mode": "EN_ROUTE"})
    assert bs.tasks["T2"].state == "ACTIVE"
    assert bs.tasks["T3"].assigned_to == "UGV-1"
    t_stop = h.t
    detected = None
    for _ in range(20):
        h.run(0.5, ugv=False)
        if bs.agents["UGV-1"].link == "STALE" and detected is None:
            detected = h.t
    st = h.c["comms"]["stale_timeout_s"]
    assert detected is not None and st <= detected - t_stop <= st + 1.0, "stale detected after the configured timeout"
    assert bs.tasks["T2"].state == "READY" and bs.tasks["T2"].assigned_to is None
    assert "PENDING" in bs.tasks["T2"].note or "interrupted" in bs.tasks["T2"].note
    assert bs.tasks["T3"].assigned_to == "UAV-1"
    assert not bs.agents["UGV-1"].queue
    cats = [e["category"] for e in bs.events]
    assert "FAULT_DETECTED" in cats and "TASK_INTERRUPTED" in cats and "TASK_PENDING" in cats and "TASK_REASSIGNED" in cats
    h.run(3.0, ugv_kw={"task_id": "T2", "task_state": "HELD", "task_progress": 0.4, "mode": "TASK_HELD"})
    assert bs.agents["UGV-1"].link == "HEALTHY"
    assert bs.tasks["T2"].assigned_to == "UGV-1"
    resume = [e for e in bs.events if e["category"] == "TASK_DISPATCHED" and e["task"] == "T2" and e["message"].startswith("RESUME")]
    assert resume, "held task resumed with an explicit RESUME command"


def test_old_telemetry_is_not_current(tmp_path):
    h = Harness(tmp_path)
    h.run(1.0)
    ag = h.bs.agents["UGV-1"]
    newest = ag.last_stamp
    h.radio.send(h.t, "UGV-1", "BASE", Telemetry(agent_id="UGV-1", vehicle_type="ugv", stamp=newest - 5.0, frame="ugv_map", position=(9, 9, 9)))
    h.tick(0.0)
    assert ag.last_stamp == newest and ag.pos_local != (9, 9, 9)


def test_battery_and_localisation_gates(tmp_path):
    h = Harness(tmp_path)
    h.run(1.0, ugv_kw={"battery_pct": 15.5}, uav_kw={"loc_valid": False, "loc_confidence": 0.0})
    T = h.bs.tasks
    assert T["T1"].assigned_to is None and "localisation" in T["T1"].note
    assert T["T2"].assigned_to is None and "battery" in T["T2"].note


def test_ground_truth_firewall():
    """Coordination + agent modules must not import the simulator / world / pybullet (platform/ and experiment.py are the sim side)."""
    banned = {"pybullet", "pybullet_data", "sim", "worlds", "eval", "coordination_sim.platform", "coordination_sim.experiment"}
    pkg = os.path.join(ROOT, "coordination_sim")
    for sub in ("agents", "base_station", "comms"):
        for fn in os.listdir(os.path.join(pkg, sub)):
            if not fn.endswith(".py"):
                continue
            tree = ast.parse(open(os.path.join(pkg, sub, fn), encoding="utf-8").read())
            for node in ast.walk(tree):
                names = [a.name for a in node.names] if isinstance(node, ast.Import) else [node.module or ""] if isinstance(node, ast.ImportFrom) else []
                for n in names:
                    assert n.split(".")[0] not in banned and not any(n.startswith(b) for b in banned if "." in b), f"{sub}/{fn} imports {n}"
