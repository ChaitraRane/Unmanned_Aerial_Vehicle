"""4-panel recording: overview | live mission map + task table / UAV camera | UGV camera, plus event ticker."""
from __future__ import annotations

import math

import cv2
import numpy as np
import pybullet as p

W, H = 1280, 800
STATUS_BGR = {"PENDING": (150, 150, 150), "ASSIGNED": (0, 200, 255), "ACCEPTED": (255, 200, 0), "IN_PROGRESS": (0, 165, 255),
              "COMPLETED": (60, 200, 60), "FAILED": (40, 40, 230), "HOLD": (200, 0, 200)}
MK_BGR = {"orange": (0, 115, 255), "yellow": (0, 230, 255), "cyan": (240, 230, 0), "magenta": (215, 0, 240)}
PID = {"orange": "P1", "yellow": "P2", "cyan": "P3", "magenta": "P4"}
F = cv2.FONT_HERSHEY_SIMPLEX


def txt(img, s, xy, sc=0.42, col=(235, 235, 235), th=1):
    cv2.putText(img, s, xy, F, sc, (0, 0, 0), th + 2, cv2.LINE_AA)
    cv2.putText(img, s, xy, F, sc, col, th, cv2.LINE_AA)


class Recorder:
    def __init__(self, pb, cfg, world, path, fps):
        self.pb, self.cfg, self.world = pb, cfg, world
        self.vw = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (W, H))
        eye, tgt = world.w(-9, -16, 24), world.w(17, 11, 0)
        self.view = p.computeViewMatrix(eye, tgt, (0, 0, 1), physicsClientId=pb)
        self.proj = p.computeProjectionMatrixFOV(52, 640 / 360, 0.5, 120)
        self.last = None

    def overview(self):
        img = p.getCameraImage(640, 360, self.view, self.proj, renderer=p.ER_TINY_RENDERER, flags=p.ER_NO_SEGMENTATION_MASK,
                               physicsClientId=self.pb)
        rgb = np.asarray(img[2], np.uint8).reshape(360, 640, 4)[:, :, :3]
        buf = np.asarray(img[3], np.float32).reshape(360, 640)
        rgb = rgb.copy()
        rgb[buf >= 0.9999] = (205, 220, 238)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def map_panel(self, t, coord, uav, ugv):
        img = np.full((360, 640, 3), 28, np.uint8)
        s, ox, oy = 10.0, 15, 340
        (x0, x1), (y0, y1) = self.cfg["area"]["x_m"], self.cfg["area"]["y_m"]

        def P(x, y):
            return int(ox + (x - x0) * s), int(oy - (y - y0) * s)

        cv2.rectangle(img, P(x0, y1), P(x1, y0), (52, 60, 52), -1)
        if "magenta" not in coord.targets:
            cv2.rectangle(img, P(27.2, y1), P(x1, y0), (70, 70, 90), -1)
            txt(img, "UNEXPLORED", (P(28, 14)[0] - 4, P(28, 14)[1]), 0.36, (200, 200, 255))
        for gx in range(int(x0), int(x1) + 1, 5):
            cv2.line(img, P(gx, y0), P(gx, y1), (64, 74, 64), 1)
        for gy in range(int(y0), int(y1) + 1, 5):
            cv2.line(img, P(x0, gy), P(x1, gy), (64, 74, 64), 1)
        cv2.rectangle(img, P(x0, y1), P(x1, y0), (40, 40, 230), 2)
        occ = ugv.occupied_mission()
        for x, y in occ[:: max(1, len(occ) // 4000)]:
            cv2.circle(img, P(x, y), 1, (170, 170, 170), -1)
        cv2.rectangle(img, P(-1.6, 0.0), P(1.6, -2.2), (200, 200, 200), -1)
        o = P(0, 0)
        cv2.arrowedLine(img, o, P(4, 0), (60, 60, 255), 2, tipLength=0.25)
        cv2.arrowedLine(img, o, P(0, 4), (60, 255, 60), 2, tipLength=0.25)
        txt(img, "E", (P(4.3, -0.3)[0], P(4.3, -0.3)[1]), 0.38, (90, 90, 255))
        txt(img, "N", (P(-0.4, 4.4)[0], P(-0.4, 4.4)[1]), 0.38, (90, 255, 90))
        txt(img, "BASE (0,0,0)", (P(-4.5, -3.6)[0], P(-4.5, -3.6)[1]), 0.34)
        for name, tg in coord.targets.items():
            q = P(tg["mission_xyz"][0], tg["mission_xyz"][1])
            cv2.circle(img, q, 7, MK_BGR[name], -1)
            cv2.circle(img, q, 9, (255, 255, 255), 1)
            txt(img, f"{PID[name]} {tg['category'][0]}", (q[0] + 10, q[1] + 4), 0.38, MK_BGR[name])
        if len(uav.traj) > 1:
            pts = np.array([P(a, b) for _, a, b, _ in uav.traj], np.int32)
            cv2.polylines(img, [pts], False, (255, 170, 60), 1, cv2.LINE_AA)
        if len(ugv.traj) > 1:
            pts = np.array([P(a, b) for _, a, b in ugv.traj], np.int32)
            cv2.polylines(img, [pts], False, (80, 255, 120), 2, cv2.LINE_AA)
        if ugv.state == "NAVIGATING" and ugv.path:
            pts = [ugv.frames.convert([a, b, 0], "ugv_odom", "mission_enu") for a, b in ugv.path]
            cv2.polylines(img, [np.array([P(q[0], q[1]) for q in pts], np.int32)], False, (0, 255, 255), 1, cv2.LINE_AA)
        for h in getattr(coord, "handovers", []):
            if t - h["t"] < 12:
                tg = h["mission_xyz"]
                ug = coord.agents["UGV"]["pose"]
                cv2.arrowedLine(img, P(ug[0], ug[1]), P(tg[0], tg[1]), (255, 0, 255), 2, tipLength=0.06)
                txt(img, f"HANDOVER {h['task_id']}", (P(tg[0], tg[1])[0] - 50, P(tg[0], tg[1])[1] - 14), 0.42, (255, 120, 255), 1)
        if uav.traj:
            _, a, b, c = uav.traj[-1]
            q = P(a, b)
            cv2.drawMarker(img, q, (255, 200, 80), cv2.MARKER_TILTED_CROSS, 14, 2)
            txt(img, f"UAV z={c:.1f}", (q[0] + 8, q[1] - 8), 0.36, (255, 200, 80))
        if ugv.traj:
            q = P(ugv.traj[-1][1], ugv.traj[-1][2])
            cv2.rectangle(img, (q[0] - 4, q[1] - 4), (q[0] + 4, q[1] + 4), (80, 255, 120), -1)
            txt(img, "UGV", (q[0] + 8, q[1] + 14), 0.36, (80, 255, 120))
        tx = 432
        cv2.rectangle(img, (tx - 6, 0), (640, 360), (18, 18, 18), -1)
        txt(img, "TASK STATUS", (tx, 20), 0.5, (255, 255, 255), 1)
        for i, k in enumerate(coord.tasks.values()):
            y = 44 + i * 24
            ag = k.agent or ("UAV+UGV" if k.task_id == "T6" else "-")
            txt(img, f"{k.task_id} {k.target:4s} {ag:7s}", (tx, y), 0.4)
            txt(img, k.status, (tx + 128, y), 0.4, STATUS_BGR[k.status])
        y = 44 + 6 * 24 + 6
        txt(img, f"stage: {coord.stage}", (tx, y), 0.4, (200, 200, 255))
        txt(img, f"UAV: {uav.state}", (tx, y + 20), 0.38, (255, 200, 80))
        txt(img, f"UGV: {ugv.state}", (tx, y + 38), 0.38, (80, 255, 120))
        lk = coord.agents.get("UGV", {}).get("link", "-")
        txt(img, f"UGV link: {lk}", (tx, y + 56), 0.38, (60, 60, 255) if lk == "TIMEOUT" else (200, 200, 200))
        txt(img, "frame: mission_enu", (tx, y + 80), 0.36, (180, 180, 180))
        txt(img, "origin = base station", (tx, y + 96), 0.36, (180, 180, 180))
        txt(img, "X=East Y=North Z=Up", (tx, y + 112), 0.36, (180, 180, 180))
        return img

    def cam(self, rgb, label, info):
        if rgb is None:
            im = np.zeros((360, 640, 3), np.uint8)
        else:
            im = cv2.resize(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), (480, 360))
            im = cv2.copyMakeBorder(im, 0, 0, 80, 80, cv2.BORDER_CONSTANT, value=(18, 18, 18))
        txt(im, label, (90, 22), 0.55, (255, 255, 255), 1)
        for i, s in enumerate(info):
            txt(im, s, (90, 44 + 18 * i), 0.42, (230, 230, 120))
        return im

    def frame(self, t, coord, uav, ugv):
        img = np.zeros((H, W, 3), np.uint8)
        img[0:360, 0:640] = self.overview()
        txt(img, "SHARED INSPECTION SITE (PyBullet)", (10, 22), 0.55, (255, 255, 255), 1)
        txt(img, f"t = {t:6.1f} s", (530, 22), 0.5)
        img[0:360, 640:1280] = self.map_panel(t, coord, uav, ugv)
        um, _ = uav.pose_mission() if uav.reg is not None else ((0, 0, 0), 0)
        uk = uav.task["task_id"] if uav.task else "-"
        img[360:720, 0:640] = self.cam(uav.last_rgb, "UAV RGB-D camera (onboard)",
                                       [f"task {uk}  state {uav.state}", f"pose mission_enu ({um[0]:.1f}, {um[1]:.1f}, {um[2]:.1f})"])
        gm, _ = ugv.pose_mission()
        gk = ugv.task["task_id"] if ugv.task else "-"
        img[360:720, 640:1280] = self.cam(ugv.last_rgb, "UGV RGB-D camera (onboard)",
                                          [f"task {gk}  state {ugv.state}", f"pose mission_enu ({gm[0]:.1f}, {gm[1]:.1f})",
                                           f"pose ugv_odom ({ugv.x[0]:.1f}, {ugv.x[1]:.1f}, {math.degrees(ugv.x[2]):.0f} deg)"])
        cv2.line(img, (640, 0), (640, 720), (90, 90, 90), 1)
        cv2.line(img, (0, 360), (1280, 360), (90, 90, 90), 1)
        for i, e in enumerate(coord.events[-4:]):
            col = {"HANDOVER": (255, 120, 255), "FAULT": (60, 60, 255), "REASSIGN": (60, 140, 255), "FRAME": (255, 220, 120),
                   "MISSION": (80, 255, 80)}.get(e["kind"], (220, 220, 220))
            txt(img, f"{e['t']:6.1f}s [{e['kind']}] {e['text']}"[:170], (8, 738 + 18 * i), 0.4, col)
        self.vw.write(img)
        self.last = img
        return img

    def close(self):
        self.vw.release()
