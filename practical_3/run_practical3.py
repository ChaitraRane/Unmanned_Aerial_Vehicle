"""Practical 3 - Collaborative task planning (one UAV + one UGV + base station) in one shared PyBullet world.

    python practical_3/run_practical3.py [--no-fault] [--max-time 600]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import pybullet as p
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path[:0] = [ROOT, HERE]

from companion.mavlink_client import CompanionMavlink  # noqa: E402
from sim.config import load_config  # noqa: E402
from sim.mavlink_fc_server import MavlinkFcServer  # noqa: E402
from sim.rgbd_camera import RGBDSensor  # noqa: E402
from sim.sim_core import UavSim  # noqa: E402
from p3.comms import Bus  # noqa: E402
from p3.coordinator import Coordinator  # noqa: E402
from p3.frames import FrameTree, MISSION, UGV_ODOM, register_ugv  # noqa: E402
from p3.uav_agent import UavAgent  # noqa: E402
from p3.ugv_agent import UgvAgent, UgvVehicle  # noqa: E402
from p3.viz import Recorder  # noqa: E402
from p3.world import InspectionWorld  # noqa: E402

SHOT_KINDS = ("REGISTER", "HANDOVER", "FAULT", "REASSIGN", "MISSION", "CLASSIFY")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=os.path.join(HERE, "config", "practical3_tasks.yaml"))
    ap.add_argument("--max-time", type=float, default=600.0)
    ap.add_argument("--no-fault", action="store_true")
    ap.add_argument("--port", type=int, default=14590)
    a = ap.parse_args()

    cfg = yaml.safe_load(open(a.config, encoding="utf-8"))
    out = os.path.join(HERE, "outputs", time.strftime("run_%Y%m%d_%H%M%S"))
    for d in ("images", "screenshots"):
        os.makedirs(os.path.join(out, d), exist_ok=True)
    logf = open(os.path.join(out, "events.log"), "w", encoding="utf-8")

    ucfg = load_config()
    off = cfg["mission_frame"]["world_from_mission_xy"]
    pad = cfg["agents"]["UAV"]["pad_mission_xyz"]
    ucfg["spawn"]["xy_enu_m"] = [pad[0] + off[0], pad[1] + off[1]]
    ucfg["spawn"]["surface_z_m"] = 0.01
    ucfg["sensors"]["camera"]["width"], ucfg["sensors"]["camera"]["height"] = 384, 288
    ucfg["mavlink"]["fc_bind"] = f"udpin:0.0.0.0:{a.port}"

    sim = UavSim(ucfg, gui=False, world=False)
    for i in range(p.getNumBodies(physicsClientId=sim.pb)):
        uid = p.getBodyUniqueId(i, physicsClientId=sim.pb)
        if b"plane" in p.getBodyInfo(uid, physicsClientId=sim.pb)[1]:
            p.removeBody(uid, physicsClientId=sim.pb)
            break
    world = InspectionWorld(sim.pb, cfg)

    def log(s):
        line = f"{sim.t:8.2f}s | {time.strftime('%H:%M:%S')} | {s}"
        print(line, flush=True)
        logf.write(line + "\n")

    server = MavlinkFcServer(sim, log_path=os.path.join(out, "fc_mavlink.log"))
    mav = CompanionMavlink(f"udp:127.0.0.1:{a.port}", log_path=os.path.join(out, "companion_mavlink.log"))
    th = threading.Thread(target=mav.connect, args=(30,), daemon=True)
    th.start()
    while th.is_alive():
        sim.step()
        time.sleep(0.0005)
    log(f"MAVLink link up: companion <-> simulated FC (sys {mav.target_system}) on UDP {a.port}")

    frames = FrameTree()
    ug = cfg["agents"]["UGV"]
    T_ugv = register_ugv(ug["dock_mission_xy"], ug["dock_heading_deg"])
    frames.add(T_ugv)
    log(f"[FRAME] registered {UGV_ODOM}: {T_ugv.describe()}")
    bus = Bus(cfg["comms"]["latency_s"], os.path.join(out, "messages.jsonl"), lambda: sim.t)
    coord = Coordinator(cfg, bus, frames, log)
    cam = RGBDSensor(sim)
    dock_w = world.w(*ug["dock_mission_xy"])
    ugv_veh = UgvVehicle(sim.pb, dock_w, math.radians(ug["dock_heading_deg"]))
    uav = UavAgent(cfg, bus, frames, mav, cam, sim.model, out, log)
    ugv = UgvAgent(cfg, bus, frames, ugv_veh, out, log)
    rec = Recorder(sim.pb, cfg, world, os.path.join(out, "practical3_simulation.mp4"), cfg["recording"]["video_fps"])
    log("operator: mission start requested (operator station)")

    fi = cfg["comms"]["fault_injection"]
    fault = {"armed": not a.no_fault, "start": None, "end": None}
    per = cfg["recording"]["frame_period_s"]
    n_frame = int(round(per / sim.dt))
    n_agent = int(round(0.1 / sim.dt))
    n_ugv = int(round(0.02 / sim.dt))
    last_hb, shots, n_ev = -9.0, set(), 0
    wall0 = time.time()
    done_t = None
    gt_track = []
    while sim.t < a.max_time:
        sim.step()
        k = sim.step_count
        frame_tick = k % n_frame == 0
        uav_frame = ugv_frame = None
        if frame_tick:
            uav_frame = cam.capture()
            ugv_frame = ugv_veh.capture()
        if k % n_ugv == 0:
            bus.pump()
            ugv.step(sim.t, n_ugv * sim.dt, ugv_frame)
        if k % n_agent == 0 or frame_tick:
            bus.pump()
            if sim.t - last_hb >= 1.0:
                last_hb = sim.t
                for ag in ("UAV", "UGV"):
                    bus.send("BASE", ag, "HEARTBEAT", {"frame_id": MISSION, "stage": coord.stage})
            uav.step(sim.t, uav_frame)
            coord.update(sim.t)
            T4 = coord.tasks["T4"]
            if fault["armed"] and fault["start"] is None and T4.status in ("ACCEPTED", "IN_PROGRESS"):
                fault["start"] = sim.t + fi["start_after_accept_s"]
                fault["end"] = fault["start"] + fi["duration_s"]
            if fault["start"] is not None:
                down = fault["start"] <= sim.t < fault["end"]
                if down != (not bus.link_up["UGV"]):
                    bus.link_up["UGV"] = not down
                    log(f"[INJECT] UGV radio link {'DOWN (fault injection, Condition D)' if down else 'UP'}")
        if frame_tick:
            img = rec.frame(sim.t, coord, uav, ugv)
            pw, _ = ugv_veh.truth()
            gt_track.append([sim.t, pw[0] - off[0], pw[1] - off[1], *ugv.pose_mission()[0][:2]])
            for e in coord.events[n_ev:]:
                key = (e["kind"], e["text"][:24])
                if e["kind"] in SHOT_KINDS and key not in shots:
                    shots.add(key)
                    cv2.imwrite(os.path.join(out, "screenshots", f"{len(shots):02d}_{e['t']:06.1f}s_{e['kind']}.png"), img)
            n_ev = len(coord.events)
            if int(sim.t / per) % 120 == 0:
                cv2.imwrite(os.path.join(out, "screenshots", f"periodic_{sim.t:06.1f}s.png"), img)
            if int(sim.t / per) % 40 == 0:
                log(f"progress t={sim.t:.0f}s wall={time.time() - wall0:.0f}s stage={coord.stage} UAV={uav.state} UGV={ugv.state}")
        if coord.done and done_t is None:
            done_t = sim.t
        if done_t is not None and sim.t - done_t > 4.0:
            break

    rec.close()
    cv2.imwrite(os.path.join(out, "screenshots", "zz_final.png"), rec.last)
    write_outputs(out, cfg, coord, uav, ugv, frames, bus, sim, server, gt_track, wall0, ugv_veh, off, T_ugv)
    mav.close()
    server.close()
    bus.close()
    sim.close()
    logf.close()
    print(f"OUTPUT {out}")


def write_outputs(out, cfg, coord, uav, ugv, frames, bus, sim, server, gt_track, wall0, ugv_veh, off, T_ugv):
    truth = {k: v["world_truth_mission"] for k, v in cfg["targets"].items()}
    pid = {"orange": "P1", "yellow": "P2", "cyan": "P3", "magenta": "P4"}
    target_eval = {}
    for name, tg in coord.targets.items():
        g = truth[pid[name]]
        target_eval[pid[name]] = {"uav_frame": tg["src_frame"], "uav_xyz": np.round(tg["src_xyz"], 3).tolist(),
                                  "mission_xyz": np.round(tg["mission_xyz"], 3).tolist(), "truth_mission_xyz": g,
                                  "horizontal_error_m": round(float(math.hypot(tg["mission_xyz"][0] - g[0], tg["mission_xyz"][1] - g[1])), 3),
                                  "category": tg["category"], "agent": tg["agent"], "task": tg["task"]}
    gt = np.array(gt_track)
    np.savetxt(os.path.join(out, "ugv_estimate_vs_truth_eval_only.csv"), gt, delimiter=",", fmt="%.3f",
               header="t,truth_x,truth_y,est_x,est_y (mission_enu)", comments="")
    drift = np.hypot(gt[:, 1] - gt[:, 3], gt[:, 2] - gt[:, 4])
    tasks = {k: {"agent": v.agent, "status": v.status, "target": v.target, "action": v.action, "target_mission": v.target_mission,
                 "results": v.results, "history": v.history} for k, v in coord.tasks.items()}
    rep = {"mission_complete": coord.tasks["T6"].status == "COMPLETED", "sim_time_s": round(sim.t, 1),
           "wall_time_s": round(time.time() - wall0, 1), "frame_id": MISSION,
           "transforms": [frames.tf[(MISSION, "uav_local_ned")].describe(), T_ugv.describe()],
           "tasks": tasks, "handovers": getattr(coord, "handovers", []), "targets_evaluation_only": target_eval,
           "ugv_wheel_slip_events": ugv.stall_events, "ugv_odometry_error_evaluation_only": {"final_m": round(float(drift[-1]), 3), "max_m": round(float(drift.max()), 3)},
           "messages_sent": bus.seq, "messages_dropped_link_down": bus.dropped,
           "mavlink_rx": dict(server.rx_counts), "mavlink_tx": dict(server.tx_counts),
           "final_agent_status": {a: {k: v for k, v in s.items() if k != "last_seen"} for a, s in coord.agents.items()},
           "events": coord.events}
    with open(os.path.join(out, "mission_report.json"), "w", encoding="utf-8") as f:
        json.dump(rep, f, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    with open(os.path.join(out, "transforms.json"), "w", encoding="utf-8") as f:
        json.dump({"transforms": rep["transforms"], "handovers": rep["handovers"], "targets": target_eval}, f, indent=1,
                  default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    lines = ["# Practical 3 - final mission summary (operator station)", "",
             f"Mission: **{'COMPLETED' if rep['mission_complete'] else 'INCOMPLETE'}** at t = {rep['sim_time_s']} s (sim)", "",
             "| Task | Target | Agent | Status |", "|---|---|---|---|"]
    for k, v in tasks.items():
        lines.append(f"| {k} | {v['target']} | {v['agent']} | {v['status']} |")
    with open(os.path.join(out, "mission_summary.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
