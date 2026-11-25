"""Operator dashboard (OpenCV image).  Left: site situation map drawn ONLY from base-station knowledge (telemetry estimates
transformed to the site frame, shared map summaries, coverage, tasks, keep-outs).  Right: live PyBullet render of the
shared world (real simulation image), the UAV / UGV status table, task queue and coordination event log.
"""
from __future__ import annotations

import math

import cv2
import numpy as np

W, H = 1600, 900
MAP_W = 820
STATE_COL = {"CANDIDATE": (150, 150, 150), "READY": (0, 165, 255), "ASSIGNED": (255, 200, 0), "ACTIVE": (0, 220, 255),
             "COMPLETED": (60, 200, 60), "FAILED": (40, 40, 230), "CANCELLED": (120, 120, 120)}
LINK_COL = {"HEALTHY": (60, 200, 60), "STALE": (0, 165, 255), "LOST": (40, 40, 230), "UNKNOWN": (150, 150, 150)}
AG_COL = {"uav": (40, 90, 230), "ugv": (230, 120, 30)}


def _txt(img, s, xy, scale=0.45, col=(230, 230, 230), th=1):
    cv2.putText(img, s, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, col, th, cv2.LINE_AA)


class Dashboard:
    def __init__(self, boundary):
        xs = [p[0] for p in boundary]
        ys = [p[1] for p in boundary]
        self.x0, self.x1 = min(xs) - 1.5, max(xs) + 1.5
        self.y0, self.y1 = min(ys) - 1.5, max(ys) + 1.5
        self.s = min((MAP_W - 20) / (self.x1 - self.x0), (H - 60) / (self.y1 - self.y0))
        self.overview = None

    def px(self, x, y):
        return int(10 + (x - self.x0) * self.s), int(50 + (self.y1 - y) * self.s)

    def render(self, snap, title, banner=None):
        img = np.full((H, W, 3), 28, np.uint8)
        t = snap["t"]
        _txt(img, title, (10, 22), 0.6, (255, 255, 255), 1)
        _txt(img, f"sim t = {t:6.1f} s", (MAP_W - 150, 22), 0.55, (255, 255, 255))
        if banner:
            cv2.rectangle(img, (10, 28), (MAP_W - 10, 46), (0, 0, 160), -1)
            _txt(img, banner, (16, 42), 0.45, (255, 255, 255))
        self._map(img, snap)
        self._right(img, snap)
        return img

    def _map(self, img, snap):
        cv2.rectangle(img, (10, 50), (MAP_W - 10, H - 10), (45, 45, 45), -1)
        cov, (ox, oy) = snap["coverage"], snap["cov_origin"]
        for gy in range(cov.shape[0]):
            for gx in range(cov.shape[1]):
                v = cov[gy, gx]
                if v:
                    a, b = self.px(ox + gx, oy + gy + 1), self.px(ox + gx + 1, oy + gy)
                    col = (70, 95, 70) if v & 1 else (95, 80, 60)
                    if v == 3:
                        col = (95, 110, 80)
                    cv2.rectangle(img, a, b, col, -1)
        for z in snap["zones"].values():
            cx, cy = z["center"]
            hx, hy = z["half"]
            cv2.rectangle(img, self.px(cx - hx, cy + hy), self.px(cx + hx, cy - hy), (0, 200, 230), 1)
            _txt(img, z["id"], self.px(cx - hx + 0.1, cy + hy - 0.6), 0.4, (0, 200, 230))
        pts = np.array([self.px(*p) for p in snap["boundary"]], np.int32)
        cv2.polylines(img, [pts], True, (60, 60, 230), 2)
        for aid, P in snap["map_points"].items():
            col = (90, 90, 200) if aid.startswith("UAV") else (200, 140, 60)
            for x, y in P[::2]:
                cv2.circle(img, self.px(x, y), 1, col, -1)
        for k in snap["keepouts"]:
            cv2.circle(img, self.px(*k["xy"]), int(k["radius_m"] * self.s), (0, 100, 255), 1)
            _txt(img, "KEEP-OUT", self.px(k["xy"][0] - 1.0, k["xy"][1] + k["radius_m"] + 0.2), 0.35, (0, 120, 255))
        for tk in snap["tasks"].values():
            col = STATE_COL.get(tk.state, (200, 200, 200))
            for i, w in enumerate(tk.waypoints):
                cv2.circle(img, self.px(*w), 3, col, -1)
                if i:
                    cv2.line(img, self.px(*tk.waypoints[i - 1]), self.px(*w), col, 1)
            if tk.target_xy:
                c = self.px(*tk.target_xy)
                cv2.drawMarker(img, c, col, cv2.MARKER_DIAMOND, 14, 2)
                if tk.approach_xy and tk.approach_xy != tk.target_xy:
                    cv2.line(img, self.px(*tk.approach_xy), c, col, 1)
                _txt(img, f"{tk.task_id}", (c[0] + 8, c[1] - 6), 0.42, col)
        for tid, xy in snap["survivors"].items():
            _txt(img, f"person #{tid}", self.px(xy[0] - 1.0, xy[1] - 0.9), 0.35, (180, 180, 255))
        bs = self.px(*self._bs)
        cv2.rectangle(img, (bs[0] - 7, bs[1] - 7), (bs[0] + 7, bs[1] + 7), (230, 230, 230), -1)
        cv2.line(img, (bs[0] + 4, bs[1] - 7), (bs[0] + 4, bs[1] - 22), (230, 230, 230), 2)
        _txt(img, "BASE", (bs[0] + 10, bs[1] + 4), 0.4)
        for ag in snap["agents"].values():
            if ag.pos_site is None:
                continue
            c = self.px(*ag.pos_site)
            stale = ag.link != "HEALTHY"
            col = (130, 130, 130) if stale else AG_COL[ag.vtype]
            yaw = ag.yaw_site or 0.0
            tip = (int(c[0] + 16 * math.cos(yaw)), int(c[1] - 16 * math.sin(yaw)))
            cv2.circle(img, c, 8, col, -1 if not stale else 2)
            cv2.line(img, c, tip, col, 2)
            lab = ag.agent_id + (f" LAST KNOWN ({ag.telemetry_age:.0f}s old)" if stale else "")
            _txt(img, lab, (c[0] + 10, c[1] + 16), 0.42, col)
        _txt(img, "site map (base-station estimates only)   red=boundary  yellow=zones  green=UAV coverage  brown=UGV coverage",
             (14, H - 16), 0.38, (180, 180, 180))

    def _right(self, img, snap):
        x0 = MAP_W
        if self.overview is not None:
            ov = cv2.resize(self.overview, (W - x0 - 10, int((W - x0 - 10) * 9 / 16)))
            img[10:10 + ov.shape[0], x0:x0 + ov.shape[1]] = ov
            _txt(img, "PyBullet shared world (live render)", (x0 + 6, 26), 0.45, (255, 255, 255))
            y = 10 + ov.shape[0] + 8
        else:
            y = 20
        rows = ["Position (site)", "Battery", "Task", "Status", "Progress", "Link"]
        ags = list(snap["agents"].values())
        cols = [x0, x0 + 130, x0 + 450]
        cv2.rectangle(img, (x0, y), (W - 10, y + 22 + 18 * len(rows)), (50, 50, 50), -1)
        _txt(img, "Parameter", (cols[0] + 4, y + 15), 0.45, (255, 255, 255))
        for j, ag in enumerate(ags):
            _txt(img, ag.agent_id, (cols[j + 1] + 4, y + 15), 0.45, AG_COL[ag.vtype], 1)
        for i, r in enumerate(rows):
            yy = y + 33 + 18 * i
            _txt(img, r, (cols[0] + 4, yy), 0.42)
            for j, ag in enumerate(ags):
                tk = snap["tasks"].get(ag.queue[0]) if ag.queue else None
                if r.startswith("Position"):
                    v = "-" if ag.pos_site is None else f"({ag.pos_site[0]:.1f}, {ag.pos_site[1]:.1f})" + \
                        (f" z={ag.pos_local[2]:.1f}" if ag.vtype == "uav" and ag.pos_local else "") + (f" age {ag.telemetry_age:.1f}s" if ag.link != "HEALTHY" else "")
                    col = (230, 230, 230) if ag.link == "HEALTHY" else (0, 165, 255)
                elif r == "Battery":
                    v, col = (f"{ag.battery:.0f} %" if ag.battery >= 0 else "-"), (230, 230, 230)
                elif r == "Task":
                    v, col = (f"{tk.task_id} {tk.type}" if tk else "none") + (f"  (+{len(ag.queue) - 1} queued)" if len(ag.queue) > 1 else ""), (230, 230, 230)
                elif r == "Status":
                    st = "FAULT" if ag.health == "FAULT" or ag.failsafe else ("HOLD" if "HOLD" in ag.mode else "ACTIVE")
                    if ag.link != "HEALTHY":
                        st = "UNAVAILABLE"
                    v = f"{st} [{ag.mode}]"
                    col = (60, 200, 60) if st == "ACTIVE" else (0, 165, 255) if st == "HOLD" else (40, 40, 230)
                elif r == "Progress":
                    v, col = (f"{tk.progress * 100:.0f} % ({tk.state})" if tk else "-"), (230, 230, 230)
                else:
                    v = f"{ag.link} ({ag.telemetry_age:.1f} s)" if ag.telemetry_age < 1e5 else ag.link
                    col = LINK_COL.get(ag.link, (200, 200, 200))
                _txt(img, v[:46], (cols[j + 1] + 4, yy), 0.42, col)
        y = y + 30 + 18 * len(rows)
        _txt(img, "Task queue", (x0 + 4, y + 12), 0.48, (255, 255, 255))
        y += 18
        for tk in sorted(snap["tasks"].values(), key=lambda t_: t_.task_id):
            col = STATE_COL.get(tk.state, (200, 200, 200))
            s = f"{tk.task_id} {tk.state:<9} {tk.assigned_to or tk.completed_by or '-':<6} {tk.progress * 100:3.0f}%  p={tk.priority:.2f} {tk.name}"
            _txt(img, s[:96], (x0 + 4, y + 12), 0.38, col)
            y += 15
            if tk.note and tk.state in ("READY", "CANDIDATE"):
                _txt(img, ("   " + tk.note)[:110], (x0 + 4, y + 12), 0.33, (0, 165, 255))
                y += 14
        y += 6
        _txt(img, "Coordination events", (x0 + 4, y + 12), 0.48, (255, 255, 255))
        y += 18
        for ev in snap["events"]:
            if y > H - 14:
                break
            col = (40, 40, 230) if ev["category"] in ("FAULT_DETECTED", "FAULT_INJECTED", "TASK_INTERRUPTED", "LINK_LOST") else \
                  (0, 165, 255) if ev["category"] in ("TASK_PENDING", "TASK_REASSIGNED", "FAULT_CLEARED") else \
                  (60, 200, 60) if ev["category"] in ("TASK_COMPLETED", "LINK_RECOVERED") else (210, 210, 210)
            s = f"{ev['t']:6.1f} {ev['category']:<18} {ev['agent'] or '':<6} {ev['task'] or '':<3} {ev['message']}"
            _txt(img, s[:118], (x0 + 4, y + 12), 0.33, col)
            y += 14
