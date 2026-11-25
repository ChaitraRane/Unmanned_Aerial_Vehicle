"""Combined UAV + UGV + base-station experiment: one PyBullet world, one simulation clock."""
from __future__ import annotations

import copy
import csv
import json
import math
import os
import time

import coordination_sim.bootstrap  # noqa: F401  (sys.path for the two original projects)

import cv2
import numpy as np
import pybullet as p
import yaml

from common.frames import euler_from_R
from sim.config import load_config
from sim.mavlink_fc_server import MavlinkFcServer
from sim.rgbd_camera import RGBDSensor
from sim.thermal_camera import ThermalSensor

from coordination_sim.agents.uav_agent import UavAgent
from coordination_sim.agents.ugv_agent import UgvAgent
from coordination_sim.base_station.station import BaseStation
from coordination_sim.bootstrap import NEW_ROBOT_ROOT
from coordination_sim.comms.radio import RadioNetwork
from coordination_sim.frames import point_in_polygon
from coordination_sim.platform.site_world import build_site_world
from coordination_sim.platform.uav_platform import SharedUavSim
from coordination_sim.platform.ugv_platform import UgvPlatform
from coordination_sim.viz.dashboard import Dashboard

DEFAULT_MISSION = os.path.join(NEW_ROBOT_ROOT, "config", "mission.yaml")


def load_mission(path=None):
    with open(path or DEFAULT_MISSION, encoding="utf-8") as f:
        return yaml.safe_load(f)


def uav_config_for_site(mcfg, mav_port=None):
    """The UAV's own validated drone.yaml, with only the deployment-specific values changed (spawn pad on flat ground, optionally a different MAVLink UDP port so two experiments can run side by side)."""
    c = copy.deepcopy(load_config())
    if mav_port:
        c["mavlink"]["fc_bind"] = f"udpin:0.0.0.0:{int(mav_port)}"
        c["mavlink"]["companion_connection"] = f"udp:127.0.0.1:{int(mav_port)}"
    pad = mcfg["site"]["uav_pad"]
    c["spawn"]["xy_enu_m"] = list(pad["xy"])
    c["spawn"]["yaw_ned_deg"] = 90.0 - pad["yaw_enu_deg"]
    c["spawn"]["surface_z_m"] = 0.0
    return c


class OverviewCamera:
    """Real PyBullet render of the shared world (software renderer) with name tags projected onto it."""

    def __init__(self, pb, w, h):
        self.pb, self.w, self.h = pb, w, h
        self.view = p.computeViewMatrix([13.0, -15.0, 17.0], [0.5, 7.5, 0.0], [0, 0, 1])
        self.proj = p.computeProjectionMatrixFOV(52, w / h, 0.5, 120.0)
        V, P = np.array(self.view).reshape(4, 4).T, np.array(self.proj).reshape(4, 4).T
        self.M = P @ V

    def project(self, xyz):
        v = self.M @ np.array([*xyz, 1.0])
        if v[3] <= 0:
            return None
        n = v[:3] / v[3]
        return int((n[0] + 1) / 2 * self.w), int((1 - n[1]) / 2 * self.h)

    def render(self, tags):
        img = p.getCameraImage(self.w, self.h, self.view, self.proj, renderer=p.ER_TINY_RENDERER, flags=p.ER_NO_SEGMENTATION_MASK,
                               physicsClientId=self.pb)
        rgb = np.asarray(img[2], np.uint8).reshape(self.h, self.w, 4)[:, :, :3]
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        for name, xyz, col in tags:
            c = self.project(xyz)
            if c:
                cv2.putText(bgr, name, (c[0] + 6, c[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(bgr, name, (c[0] + 6, c[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
        return bgr


def _panel(img, w, h, title):
    out = cv2.resize(img, (w, h)) if img is not None else np.zeros((h, w, 3), np.uint8)
    cv2.rectangle(out, (0, 0), (w, 24), (0, 0, 0), -1)
    cv2.putText(out, title, (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def uav_map_image(agent):
    if not agent.pipe.map.geo.ready:
        return None
    g = agent.pipe.map.planning_grid(agent.ucfg["demo"]["flight_altitude_m"])
    img = np.full(g.shape + (3,), 60, np.uint8)
    img[g == 0] = (200, 200, 200)
    img[g == 100] = (40, 40, 230)
    m = agent.pipe.map.metric
    if agent.cur["pose"] is not None:
        x, y = agent.cur["pose"][:2, 3]
        cv2.circle(img, (int((y - m.origin[1]) / m.v), int((x - m.origin[0]) / m.v)), 3, (255, 80, 0), -1)
    if agent.mc is not None and agent.mc.path:
        for a, b in zip(agent.mc.path[:-1], agent.mc.path[1:]):
            cv2.line(img, (int((a[1] - m.origin[1]) / m.v), int((a[0] - m.origin[0]) / m.v)),
                     (int((b[1] - m.origin[1]) / m.v), int((b[0] - m.origin[0]) / m.v)), (0, 200, 0), 1)
    return np.transpose(img, (1, 0, 2))[::-1].copy()


def ugv_map_image(agent):
    import stage8_stabilized_navigation as S8
    if not agent.initialised:
        return None
    pr = agent.occ.probability()
    img = np.full(pr.shape + (3,), 60, np.uint8)
    img[pr < S8.FREE_PROBABILITY_THRESHOLD] = (200, 200, 200)
    img[pr > S8.OCCUPIED_PROBABILITY_THRESHOLD] = (40, 40, 230)
    for a, b in zip(agent.path[:-1], agent.path[1:]):
        ca, cb = S8.world_to_grid(*a), S8.world_to_grid(*b)
        if ca and cb:
            cv2.line(img, (ca[1], ca[0]), (cb[1], cb[0]), (0, 200, 0), 2)
    c = S8.world_to_grid(*S8.pose_position_xz(agent.pose))
    if c:
        cv2.circle(img, (c[1], c[0]), 5, (255, 80, 0), -1)
    if agent.task:
        g = S8.world_to_grid(*agent.task["goal"])
        if g:
            cv2.drawMarker(img, (g[1], g[0]), (0, 220, 255), cv2.MARKER_CROSS, 14, 2)
    return img


def recording_frame(dash_img, uav_rgb, ugv_bgr, world, uav_map, ugv_map, t):
    F = np.zeros((1080, 1920, 3), np.uint8)
    F[0:360, 0:480] = _panel(cv2.cvtColor(uav_rgb, cv2.COLOR_RGB2BGR) if uav_rgb is not None else None, 480, 360, "UAV-1 RGB camera (onboard)")
    F[0:360, 480:960] = _panel(ugv_bgr, 480, 360, "UGV-1 RGB camera (onboard)")
    F[0:360, 960:1600] = _panel(world, 640, 360, f"Shared PyBullet world   t = {t:6.1f} s")
    F[0:360, 1600:1920] = _panel(uav_map, 320, 360, "UAV live map (SLAM frame)")
    F[360:1080, 0:1280] = cv2.resize(dash_img, (1280, 720))
    F[360:1080, 1280:1920] = _panel(ugv_map, 640, 720, "UGV occupancy map + A* path (map frame)")
    return F


class ChaseCamera:
    def __init__(self, pb, w, h, back, up, ahead, fov):
        self.pb, self.w, self.h = pb, w, h
        self.back, self.up, self.ahead = back, up, ahead
        self.proj = p.computeProjectionMatrixFOV(fov, w / h, 0.05, 60.0)
        self.yaw = None
        self.eye = None

    def render(self, pos, yaw):
        if self.yaw is None:
            self.yaw = yaw
        self.yaw += 0.15 * ((yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi)
        f = np.array([math.cos(self.yaw), math.sin(self.yaw), 0.0])
        eye = np.array(pos) - self.back * f + np.array([0, 0, self.up])
        self.eye = eye if self.eye is None else self.eye + 0.35 * (eye - self.eye)
        tgt = np.array(pos) + self.ahead * f
        view = p.computeViewMatrix(self.eye.tolist(), tgt.tolist(), [0, 0, 1])
        img = p.getCameraImage(self.w, self.h, view, self.proj, renderer=p.ER_TINY_RENDERER, flags=p.ER_NO_SEGMENTATION_MASK,
                               physicsClientId=self.pb)
        rgb = np.asarray(img[2], np.uint8).reshape(self.h, self.w, 4)[:, :, :3]
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def closeup_frame(uav_img, ugv_img, uav_rgb, ugv_bgr, world, bs, uav_agent, ugv_agent, t, scenario):
    F = np.full((1080, 1920, 3), 25, np.uint8)
    F[0:720, 0:960] = _panel(uav_img, 960, 720, "UAV-1  close-up chase view")
    F[0:720, 960:1920] = _panel(ugv_img, 960, 720, "UGV-1  close-up chase view")
    F[720:1080, 0:480] = _panel(cv2.cvtColor(uav_rgb, cv2.COLOR_RGB2BGR) if uav_rgb is not None else None, 480, 360, "UAV-1 onboard camera")
    F[720:1080, 480:960] = _panel(ugv_bgr, 480, 360, "UGV-1 onboard camera")
    F[720:1080, 960:1440] = _panel(world, 480, 360, "Site overview")
    x, y = 1452, 750
    cv2.putText(F, f"Practical 4  [{scenario}]   t = {t:6.1f} s", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    y += 28
    for ag in bs.agents.values():
        tk = ag.queue[0] if ag.queue else "-"
        prog = f"{bs.tasks[tk].progress * 100:.0f}%" if tk in bs.tasks else ""
        col = (60, 200, 60) if ag.link == "HEALTHY" else (40, 40, 230)
        cv2.putText(F, f"{ag.agent_id}: {ag.mode[:14]}  task {tk} {prog}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (230, 230, 230), 1, cv2.LINE_AA)
        cv2.putText(F, f"   link {ag.link}  battery {ag.battery:.0f}%", (x, y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.46, col, 1, cv2.LINE_AA)
        y += 48
    for ev in [e for e in bs.events if e["category"] not in ("SCREENSHOT", "AGENT_MODE", "TASK_ACK", "TASK_DISPATCHED", "TASK_PENDING")][-7:]:
        col = (80, 80, 255) if "FAULT" in ev["category"] or "INTERRUPT" in ev["category"] else (0, 200, 255) if "REASSIGN" in ev["category"]             else (90, 220, 90) if "COMPLETED" in ev["category"] or "RECOVER" in ev["category"] else (200, 200, 200)
        cv2.putText(F, f"{ev['t']:6.1f} {ev['category'][:16]} {ev['agent'] or ''} {ev['task'] or ''}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)
        y += 20
    return F


class FaultInjector:
    def __init__(self, spec, name, radio, station, ugv_agent):
        self.spec, self.name, self.radio, self.bs, self.ugv = spec, name, radio, station, ugv_agent
        self.t_on = self.t_off = None
        self.active = False
        self.target_id = None

    def step(self, t, agents_cfg):
        if self.spec is None:
            return
        tgt = self.spec["target"]
        self.target_id = agents_cfg[tgt]["id"]
        if self.t_on is None:
            tr = self.spec["trigger"]
            fire = False
            if "at_t_s" in tr:
                fire = t >= tr["at_t_s"]
            else:
                done = sum(1 for e in self.ugv.events if e["event"] == "TASK_COMPLETED")
                tk = self.ugv.task
                fire = (done >= tr["after_completed_tasks"] and tk is not None and tk["state"] == "EN_ROUTE"
                        and tk["progress"] >= tr["min_active_progress"]) or t >= tr["latest_t_s"]
            if fire:
                self.t_on, self.active = t, True
                self.radio.set_link_down(self.target_id, True)
                self.bs.event(t, "FAULT_INJECTED", self.target_id, None,
                              f"[experiment harness] {self.name}: radio link of {self.target_id} forced DOWN (both directions) for {self.spec['duration_s']} s; "
                              f"vehicle and simulation keep running", fault=self.name)
        elif self.active and t >= self.t_on + self.spec["duration_s"]:
            self.active, self.t_off = False, t
            self.radio.set_link_down(self.target_id, False)
            self.bs.event(t, "FAULT_CLEARED", self.target_id, None, f"[experiment harness] {self.name}: radio link of {self.target_id} restored")


def run(scenario="normal", out=None, seconds=None, gui=False, mission_path=None, show=False, verbose=True, mav_port=None, closeup=False):
    mcfg = load_mission(mission_path)
    seed = int(mcfg["seed"])
    np.random.seed(seed)
    seconds = float(seconds or mcfg["sim"]["duration_s"])
    out = out or os.path.join(NEW_ROBOT_ROOT, "results", scenario)
    os.makedirs(out, exist_ok=True)
    shots = os.path.join(out, "screenshots")
    os.makedirs(shots, exist_ok=True)
    os.makedirs(os.path.join(out, "eval"), exist_ok=True)
    fault_spec = {"normal": None, "fault_ugv_link": mcfg["faults"]["ugv_link"], "fault_uav_link": mcfg["faults"]["uav_link"]}[scenario]

    ucfg = uav_config_for_site(mcfg, mav_port)
    pb = p.connect(p.GUI if gui else p.DIRECT)
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=pb)
        p.resetDebugVisualizerCamera(24, 35, -40, [0.5, 8.0, 0.0], physicsClientId=pb)
    hz = ucfg["sim"]["physics_hz"]
    p.setGravity(0, 0, -ucfg["vehicle"]["gravity_mps2"], physicsClientId=pb)
    p.setPhysicsEngineParameter(fixedTimeStep=1.0 / hz, numSolverIterations=100, numSubSteps=1, physicsClientId=pb)
    world = build_site_world(pb, mcfg, ucfg)
    uav = SharedUavSim(ucfg, pb, world)
    cam, th = RGBDSensor(uav), ThermalSensor(uav, world["thermal_c"])
    gpad = mcfg["site"]["ugv_pad"]
    ugvp = UgvPlatform(pb, gpad["xy"], math.radians(gpad["yaw_enu_deg"]), mcfg["agents"]["ugv"]["battery"], seed)

    radio = RadioNetwork(mcfg["comms"], seed, os.path.join(out, "radio_messages.jsonl"))
    bs = BaseStation(mcfg, radio, out)
    bs.event(0.0, "EXPERIMENT", None, None, f"scenario={scenario} seed={seed} physics={hz} Hz, one PyBullet client, UAV+UGV+base station")
    uav_agent = UavAgent(ucfg, mcfg, radio, cam.intrinsics(), th.intrinsics(), out)
    ugv_agent = UgvAgent(mcfg, mcfg["agents"]["ugv"], radio, ugvp.kin, ugvp.intrinsics, ugvp.camera_matrix, out)
    bs.event(0.0, "UAV_SLAM_BACKEND", "UAV-1", None, f"{uav_agent.backend.name} ({uav_agent.backend_note})")
    server = MavlinkFcServer(uav, bind=ucfg["mavlink"]["fc_bind"], log_path=os.path.join(out, "mavlink_fc.log"))
    uav_agent.connect_mavlink(ucfg["mavlink"]["companion_connection"], os.path.join(out, "mavlink_companion.log"))
    uav.hooks.append(lambda s: (uav_agent.feed_imu(s.imu_sample.t, s.imu_sample.gyro, s.imu_sample.accel) if s.new_imu else None,
                                uav_agent.feed_gps(s.gps_sample) if s.new_gps else None))
    injector = FaultInjector(fault_spec, scenario.replace("fault_", ""), radio, bs, ugv_agent)
    dash = Dashboard(bs.boundary)
    dash._bs = tuple(mcfg["site"]["base_station_xy"])
    ov = OverviewCamera(pb, mcfg["sim"]["overview_render"]["width"], mcfg["sim"]["overview_render"]["height"])

    cam_div = int(round(hz / cam.rate_hz))
    ugv_div = int(round(hz / 5.0))
    ugv_phase = ugv_div // 2
    nav_div = int(round(hz / ucfg["nav"]["control_hz"]))
    bs_div = int(round(hz / 10.0))
    gt_div = int(round(hz / 2.0))
    poly = bs.boundary
    gt_f = open(os.path.join(out, "eval", "gt_trace.csv"), "w", newline="")
    gt_w = csv.writer(gt_f)
    gt_w.writerow(["t", "uav_gt_x", "uav_gt_y", "uav_gt_z", "uav_est_site_x", "uav_est_site_y", "uav_inside", "ugv_gt_x", "ugv_gt_y", "ugv_gt_yaw",
                   "ugv_est_site_x", "ugv_est_site_y", "ugv_inside", "ugv_cmd_l", "ugv_cmd_r", "uav_fc_mode", "uav_armed"])
    collisions = {"ugv": [], "uav": []}
    seen_c = {"ugv": set(), "uav": set()}
    ground = world["ground_body"]
    shots_taken = {}
    next_dash = 0.0
    next_ov = 0.0
    wall0 = time.perf_counter()
    video = None
    bootstrap_t = 1.0
    last_ugv_depth = None
    last_uav_rgb = last_ugv_bgr = None
    rec = None
    crec = None
    uav_chase = ChaseCamera(pb, 640, 480, 2.6, 1.1, 1.0, 60)
    ugv_chase = ChaseCamera(pb, 640, 480, 1.9, 0.9, 1.2, 60)
    step = 0

    def shot(name, banner=None):
        if name in shots_taken:
            return
        dash.overview = ov.render(tags())
        img = dash.render(bs.snapshot(), f"Practical 4 - UAV/UGV coordination  [{scenario}]", banner)
        cv2.imwrite(os.path.join(shots, f"{name}.png"), img)
        cv2.imwrite(os.path.join(shots, f"{name}_world.png"), dash.overview)
        shots_taken[name] = round(uav.t, 2)
        bs.event(uav.t, "SCREENSHOT", None, None, f"saved screenshots/{name}.png")

    def tags():
        u = uav.vehicle.read_state().pos_enu
        g = ugvp.truth_xy_yaw()
        bx, by = mcfg["site"]["base_station_xy"]
        return [("UAV-1", (u[0], u[1], u[2] + 0.3), (255, 120, 60)), ("UGV-1", (g[0], g[1], 0.5), (60, 160, 255)),
                ("BASE STATION", (bx, by, 3.9), (255, 255, 255))]

    uav.state = uav.vehicle.read_state()
    while uav.t < 0.2 or (not uav_agent.mav_ready and uav.t < 20.0):
        uav.step_vehicle()
        p.stepSimulation(physicsClientId=pb)
        uav.after_world_step()
        if step % 50 == 0:
            time.sleep(0.002)
        step += 1
    if not uav_agent.mav_ready:
        raise RuntimeError("MAVLink companion link did not come up (is another simulator using UDP 14540?)")
    yaw_ned0 = euler_from_R(uav.estimate.R_ned_frd)[2]
    uav_agent.set_initial_heading(yaw_ned0)
    bs.event(uav.t, "MAVLINK_LINK", "UAV-1", None, f"companion <-> FC MAVLink up over UDP ({ucfg['mavlink']['companion_connection']}), FC heartbeat received")
    t0_sim = uav.t

    while uav.t < seconds:
        uav.step_vehicle()
        p.stepSimulation(physicsClientId=pb)
        uav.after_world_step()
        t = uav.t
        k = uav.step_count
        imu_s, wh_s = ugvp.sample(t)
        ugv_agent.feed_inertial(imu_s, wh_s)
        ugvp.step_battery(uav.dt)
        radio.step(t)
        if k % cam_div == 0:
            rgb, depth = cam.capture()
            last_uav_rgb = rgb
            uav_agent.on_frame(t, rgb, depth, th.capture())
        if k % ugv_div == ugv_phase and t >= bootstrap_t:
            bgr, dep = ugvp.capture()
            last_ugv_depth = dep
            last_ugv_bgr = bgr
            if not ugv_agent.initialised:
                ugv_agent.bootstrap(t, bgr, dep)
            else:
                ugv_agent.perceive(t, bgr, dep)
            ugv_agent.handle_messages(t)
            ugvp.apply(*ugv_agent.control(t, dep))
            ugv_agent.report(t, ugvp.battery.pct)
            if t >= 1.5 and not closeup:
                if t >= next_ov:
                    next_ov = t + mcfg["sim"]["overview_period_s"]
                    dash.overview = ov.render(tags())
                fb = None
                if any(a.link != "HEALTHY" for a in bs.agents.values() if a.last is not None):
                    fb = "FAULT: " + ", ".join(f"{a.agent_id} link {a.link} ({a.telemetry_age:.1f} s)" for a in bs.agents.values() if a.link != "HEALTHY")
                d_img = dash.render(bs.snapshot(), f"Practical 4 - UAV/UGV coordination  [{scenario}]", fb)
                fr = recording_frame(d_img, last_uav_rgb, last_ugv_bgr, dash.overview, uav_map_image(uav_agent), ugv_map_image(ugv_agent), t)
                if rec is None:
                    rec = cv2.VideoWriter(os.path.join(out, "simulation_recording.mp4"), cv2.VideoWriter_fourcc(*"avc1"), 10, (1920, 1080))
                    if not rec.isOpened():
                        rec = cv2.VideoWriter(os.path.join(out, "simulation_recording.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 10, (1920, 1080))
                rec.write(fr)
                if show:
                    cv2.imshow("simulation recording", cv2.resize(fr, (1280, 720)))
                    cv2.waitKey(1)
        if k % nav_div == 0:
            uav_agent.nav_step(t)
            uav_agent.report(t)
            if closeup and t >= 1.0:
                if t >= next_ov:
                    next_ov = t + mcfg["sim"]["overview_period_s"]
                    dash.overview = ov.render(tags())
                us = uav.vehicle.read_state()
                uy = math.atan2(us.R_enu_flu[1, 0], us.R_enu_flu[0, 0])
                gx, gy, gyaw = ugvp.truth_xy_yaw()
                fr = closeup_frame(uav_chase.render(us.pos_enu, uy), ugv_chase.render((gx, gy, 0.25), gyaw), last_uav_rgb, last_ugv_bgr,
                                   dash.overview, bs, uav_agent, ugv_agent, t, scenario)
                if crec is None:
                    crec = cv2.VideoWriter(os.path.join(out, "closeup_recording.mp4"), cv2.VideoWriter_fourcc(*"avc1"), 10, (1920, 1080))
                crec.write(fr)
                if show:
                    cv2.imshow("close-up recording", cv2.resize(fr, (1280, 720)))
                    cv2.waitKey(1)
        if k % bs_div == 0:
            injector.step(t, mcfg["agents"])
            bs.step(t)
        if k % gt_div == 0:
            u = uav.vehicle.read_state().pos_enu
            g = ugvp.truth_xy_yaw()
            ua, ga = bs.agents["UAV-1"], bs.agents["UGV-1"]
            ue = uav_agent.cur["pose"][:3, 3] if uav_agent.cur["pose"] is not None else None
            ues = bs.uav_reg.to_site(ue[0], ue[1]) if ue is not None else (np.nan, np.nan)
            ge = ugv_agent.pose[:3, 3] if ugv_agent.pose is not None else None
            ges = bs.ugv_reg.to_site(ge[0], ge[2]) if ge is not None else (np.nan, np.nan)
            gt_w.writerow([f"{t:.2f}", f"{u[0]:.3f}", f"{u[1]:.3f}", f"{u[2]:.3f}", f"{ues[0]:.3f}", f"{ues[1]:.3f}", int(point_in_polygon(u[0], u[1], poly)),
                           f"{g[0]:.3f}", f"{g[1]:.3f}", f"{g[2]:.3f}", f"{ges[0]:.3f}", f"{ges[1]:.3f}", int(point_in_polygon(g[0], g[1], poly)),
                           f"{ugvp.cmd[0]:.2f}", f"{ugvp.cmd[1]:.2f}", uav.fc.mode, int(uav.fc.armed)])
            for cp in ugvp.contacts():
                if cp[2] != ground and cp[2] != uav.vehicle.body and cp[2] not in seen_c["ugv"]:
                    seen_c["ugv"].add(cp[2])
                    collisions["ugv"].append({"t": round(t, 2), "body": int(cp[2]), "name": world["obstacle_ids"].get(cp[2], "scene object")})
            if uav.fc.armed and u[2] > 0.5:
                for cp in uav.vehicle.contacts_with_world():
                    if cp[2] != ground and cp[2] not in seen_c["uav"]:
                        seen_c["uav"].add(cp[2])
                        collisions["uav"].append({"t": round(t, 2), "body": int(cp[2]), "name": world["obstacle_ids"].get(cp[2], "scene object")})
            bs.log_status(t)
        if t >= next_dash:
            next_dash = t + mcfg["sim"]["dashboard_period_s"]
            if t >= next_ov:
                next_ov = t + mcfg["sim"]["overview_period_s"]
                dash.overview = ov.render(tags())
            fault_banner = None
            ugv_rec = bs.agents["UGV-1"]
            if injector.active or any(a.link != "HEALTHY" for a in bs.agents.values() if a.registered_t is not None and t > 5):
                fault_banner = "FAULT: " + ", ".join(f"{a.agent_id} link {a.link} ({a.telemetry_age:.1f} s)" for a in bs.agents.values() if a.link != "HEALTHY")
            img = dash.render(bs.snapshot(), f"Practical 4 - UAV/UGV coordination  [{scenario}]", fault_banner)
            if video is None:
                video = cv2.VideoWriter(os.path.join(out, "dashboard.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 5, (img.shape[1], img.shape[0]))
            video.write(img)
            uz = uav.vehicle.read_state().pos_enu[2]
            if "01_shared_world" not in shots_taken and uz > 1.5 and ugv_agent.mode == "EN_ROUTE":
                shot("01_shared_world")
            if "02_task_allocation" not in shots_taken and any(tk.state == "ACTIVE" for tk in bs.tasks.values() if tk.assigned_to == "UGV-1") \
                    and any(tk.state == "ACTIVE" for tk in bs.tasks.values() if tk.assigned_to == "UAV-1") and t > 25:
                shot("02_task_allocation")
            if "03_task_completed" not in shots_taken and any(tk.state == "COMPLETED" for tk in bs.tasks.values()):
                shot("03_task_completed")
            if injector.t_on is not None and "04_fault_detected" not in shots_taken and ugv_rec.link != "HEALTHY" and injector.target_id == "UGV-1" \
                    and t > injector.t_on + 4.0:
                shot("04_fault_detected", fault_banner)
            if injector.t_on is not None and injector.target_id == "UAV-1" and "04_fault_detected" not in shots_taken and bs.agents["UAV-1"].link != "HEALTHY" \
                    and t > injector.t_on + 4.0:
                shot("04_fault_detected", fault_banner)
            if any(e["category"] == "TASK_REASSIGNED" for e in bs.events) and "05_reassignment" not in shots_taken:
                shot("05_reassignment", fault_banner)
            if injector.t_off is not None and "06_recovered" not in shots_taken and t > injector.t_off + 6.0:
                shot("06_recovered")
        if t > 30 and all(tk.state in ("COMPLETED", "FAILED") for tk in bs.tasks.values()) and uav_agent.mc is not None and uav_agent.mc.done \
                and ugv_agent.mode == "IDLE":
            bs.event(t, "EXPERIMENT_END", None, None, "all tasks terminal and UAV landed")
            break
        if verbose and k % (hz * 10) == 0:
            print(f"   ... sim {t:6.1f} s  wall {time.perf_counter() - wall0:6.0f} s  UAV {uav_agent.mode_name():<12} UGV {ugv_agent.mode:<14}", flush=True)

    t_end = uav.t
    shot("07_final")
    wall = time.perf_counter() - wall0
    gt_f.close()
    if video is not None:
        video.release()
    if rec is not None:
        rec.release()
    if crec is not None:
        crec.release()
    gt = np.genfromtxt(os.path.join(out, "eval", "gt_trace.csv"), delimiter=",", names=True, dtype=None, encoding=None)
    ue = np.hypot(gt["uav_est_site_x"] - gt["uav_gt_x"], gt["uav_est_site_y"] - gt["uav_gt_y"])
    ge = np.hypot(gt["ugv_est_site_x"] - gt["ugv_gt_x"], gt["ugv_est_site_y"] - gt["ugv_gt_y"])
    summ = {
        "scenario": scenario, "seed": seed, "sim_time_s": round(t_end, 2), "wall_time_s": round(wall, 1), "physics_hz": hz,
        "uav_slam_backend": uav_agent.backend.name, "uav_slam_note": uav_agent.backend_note,
        "tasks": {k_: v.summary() for k_, v in bs.tasks.items()},
        "task_states": {k_: v.state for k_, v in bs.tasks.items()},
        "completed": [k_ for k_, v in bs.tasks.items() if v.state == "COMPLETED"],
        "agents": {a.agent_id: {"link": a.link, "mode": a.mode, "battery_pct": a.battery, "completed": a.completed, "failed": a.failed,
                                "pos_site": a.pos_site} for a in bs.agents.values()},
        "fault": None if fault_spec is None else {"name": scenario, "target": injector.target_id, "injected_t": injector.t_on, "cleared_t": injector.t_off,
                                                  "detected_t": next((e["t"] for e in bs.events if e["category"] == "FAULT_DETECTED"), None),
                                                  "stale_timeout_s": mcfg["comms"]["stale_timeout_s"]},
        "radio": radio.stats,
        "eval_ground_truth_only": {
            "uav_site_position_error_m": {"rmse": float(np.sqrt(np.nanmean(ue ** 2))), "max": float(np.nanmax(ue))},
            "ugv_site_position_error_m": {"rmse": float(np.sqrt(np.nanmean(ge ** 2))), "max": float(np.nanmax(ge))},
            "uav_always_inside_boundary": bool(np.all(gt["uav_inside"] == 1)), "ugv_always_inside_boundary": bool(np.all(gt["ugv_inside"] == 1)),
            "collisions": collisions, "uav_mission_events": uav_agent.mc.events if uav_agent.mc else [],
        },
        "uav_agent_events": uav_agent.events, "ugv_agent_events": ugv_agent.events, "ugv_stats": ugv_agent.stats,
        "ugv_viwo": ugv_agent.est.diagnostics() if ugv_agent.initialised else {},
        "screenshots": shots_taken,
        "mavlink": {"server_rx": dict(server.rx_counts), "server_tx": dict(server.tx_counts),
                    "companion_setpoints_sent": uav_agent.mav.n_setpoints_sent if uav_agent.mav else 0},
    }
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(summ, f, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    with open(os.path.join(out, "events.json"), "w") as f:
        json.dump(bs.events, f, indent=1, default=float)
    write_summary_md(out, summ, bs.events)
    bs.close()
    radio.close()
    uav_agent.close()
    server.close()
    p.disconnect(pb)
    return summ


def write_summary_md(out, s, events):
    L = [f"# Run summary - scenario `{s['scenario']}`", "",
         f"- simulated time {s['sim_time_s']} s (wall {s['wall_time_s']} s), physics {s['physics_hz']} Hz, seed {s['seed']}",
         f"- UAV SLAM backend: `{s['uav_slam_backend']}` - {s['uav_slam_note']}", "", "## Final task states", "",
         "| Task | Type | State | By | Completed at [s] | Interruptions | Result |", "|---|---|---|---|---|---|---|"]
    for k, v in s["tasks"].items():
        r = (v.get("result") or {}).get("summary", "")
        L.append(f"| {k} {v['name']} | {v['type']} | {v['state']} | {v['completed_by'] or v['assigned_to'] or '-'} | {v['completed_t'] or '-'} | {v['interruptions']} | {r} |")
    if s["fault"]:
        f = s["fault"]
        L += ["", "## Fault", "", f"- `{f['name']}` on {f['target']}: injected t={f['injected_t']}, detected t={f['detected_t']} "
              f"(stale timeout {f['stale_timeout_s']} s), cleared t={f['cleared_t']}"]
    e = s["eval_ground_truth_only"]
    L += ["", "## Offline evaluation (ground truth, never used online)", "",
          f"- UAV site-frame position error RMSE {e['uav_site_position_error_m']['rmse']:.2f} m (max {e['uav_site_position_error_m']['max']:.2f} m)",
          f"- UGV site-frame position error RMSE {e['ugv_site_position_error_m']['rmse']:.2f} m (max {e['ugv_site_position_error_m']['max']:.2f} m)",
          f"- inside boundary: UAV {e['uav_always_inside_boundary']}, UGV {e['ugv_always_inside_boundary']}",
          f"- collisions: UAV {len(e['collisions']['uav'])}, UGV {len(e['collisions']['ugv'])} {e['collisions']}",
          f"- radio: {s['radio']}", "", "## Coordination events", "", "| t [s] | Category | Agent | Task | Message |", "|---|---|---|---|---|"]
    for ev in events:
        if ev["category"] in ("SCREENSHOT",):
            continue
        L.append(f"| {ev['t']} | {ev['category']} | {ev['agent'] or ''} | {ev['task'] or ''} | {ev['message'].replace('|', '/')} |")
    with open(os.path.join(out, "summary.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")
