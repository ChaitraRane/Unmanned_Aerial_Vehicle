"""Practical 2: load telemetry logs, plot/map positions, quantify data-quality issues."""
import csv
import json
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
LOG = os.path.join(ROOT, "logs", "demo", "windows_dynamic")
P3 = os.path.join(ROOT, "practical_3", "outputs", "run_20261104_190102")
FIG = os.path.join(HERE, "figures")
R_EARTH = 6378137.0


def load_csv(path, text_cols=()):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    out = {}
    for k in rows[0]:
        vals = [r[k] for r in rows]
        if k in text_cols:
            out[k] = np.array(vals)
        else:
            out[k] = np.array([float(v) if v not in ("", "nan", "None") else np.nan for v in vals])
    return out


def yaw_from_quat(qx, qy, qz, qw):
    return np.degrees(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2)))


def interp(t_src, v_src, t_dst):
    return np.interp(t_dst, t_src, v_src, left=np.nan, right=np.nan)


def main():
    os.makedirs(FIG, exist_ok=True)
    fc = load_csv(os.path.join(LOG, "telemetry", "fc_telemetry.csv"), text_cols=("mode",))
    gps = load_csv(os.path.join(LOG, "telemetry", "gps.csv"))
    slam = load_csv(os.path.join(LOG, "slam", "slam_pose.csv"), text_cols=("state",))
    gt = load_csv(os.path.join(LOG, "sim", "ground_truth.csv"), text_cols=("mode",))
    world = json.load(open(os.path.join(LOG, "sim", "world_truth.json")))
    summary = json.load(open(os.path.join(LOG, "demo_summary.json")))

    lat0, lon0, alt0 = np.mean(gps["lat"][:10]), np.mean(gps["lon"][:10]), np.mean(gps["alt"][:10])
    gps_e = np.radians(gps["lon"] - lon0) * R_EARTH * math.cos(math.radians(lat0))
    gps_n = np.radians(gps["lat"] - lat0) * R_EARTH
    gps_u = gps["alt"] - alt0

    pn0, pe0, pd0 = gt["pn"][0], gt["pe"][0], gt["pd"][0]
    gt_e, gt_n, gt_u = gt["pe"] - pe0, gt["pn"] - pn0, -(gt["pd"] - pd0)
    gt_yaw_enu = (90.0 - np.degrees(gt["yaw"]) + 180) % 360 - 180
    slam_yaw = yaw_from_quat(slam["qx"], slam["qy"], slam["qz"], slam["qw"])

    t_s = slam["t"]
    err_slam = np.hypot(slam["x"] - interp(gt["t"], gt_e, t_s), slam["y"] - interp(gt["t"], gt_n, t_s))
    err_gps = np.hypot(gps_e - interp(gt["t"], gt_e, gps["t_sim"]), gps_n - interp(gt["t"], gt_n, gps["t_sim"]))
    gps_minus_slam = np.hypot(interp(gps["t_sim"], gps_e, t_s) - slam["x"], interp(gps["t_sim"], gps_n, t_s) - slam["y"])

    t_fc = fc["fc_time_ms"] / 1000.0
    dt_fc = np.diff(t_fc)
    dwall = np.diff(fc["t_wall"])
    dup = int(np.sum(dt_fc == 0))
    gaps = [(float(t_fc[i]), float(dt_fc[i])) for i in np.nonzero(dt_fc > 0.5)[0]]
    rtf = (t_fc[-1] - t_fc[0]) / (fc["t_wall"][-1] - fc["t_wall"][0])
    dt_gps = np.diff(gps["t_sim"])
    batt_steps = np.unique(fc["battery_pct"][~np.isnan(fc["battery_pct"])])

    # ---------------- Figure 1: map
    fig, ax = plt.subplots(figsize=(10, 9))
    ax.plot(gt_e, gt_n, color="#555555", lw=2.2, alpha=0.55, label="Ground truth (evaluation only)")
    ax.plot(gps_e, gps_n, ".", ms=2.2, color="#1f77b4", label="GPS fix (GPS_RAW_INT, 10 Hz) -> local E/N")
    ax.plot(slam["x"], slam["y"], color="#d62728", lw=1.6, label="SLAM estimate (/uav/slam/pose, VO fallback)")
    for h in world["humans"]:
        x, y = h["xy_enu"][0] - pe0, h["xy_enu"][1] - pn0
        ax.plot(x, y, "^", color="green", ms=11, mec="k")
        ax.annotate(f"survivor {h['id']} (true)", (x, y), xytext=(6, 6), textcoords="offset points", fontsize=8)
    for s in summary["semantic_map_final"]["survivors"]:
        ax.plot(s["local_x"], s["local_y"], "x", color="purple", ms=10, mew=2)
    ax.plot([], [], "x", color="purple", ms=10, mew=2, label="Survivor detected (UAV map)")
    for hz in world["hazards"]:
        x, y = hz["xy_enu"][0] - pe0, hz["xy_enu"][1] - pn0
        ax.add_patch(plt.Circle((x, y), hz["radius_m"], color="orange", alpha=0.5))
        ax.annotate("fire hazard", (x, y), xytext=(8, -12), textcoords="offset points", fontsize=8)
    ax.plot(0, 0, "s", color="black", ms=10, label="Take-off / home (origin)")
    ax.plot(gt_e[-1], gt_n[-1], "o", mfc="none", mec="#555555", ms=14, mew=2, label="Actual landing point")
    ax.plot(slam["x"][-1], slam["y"][-1], "o", mfc="none", mec="#d62728", ms=14, mew=2, label="Landing point believed by SLAM")
    ax.annotate("", xy=(gt_e[-1], gt_n[-1]), xytext=(slam["x"][-1], slam["y"][-1]),
                arrowprops=dict(arrowstyle="<->", color="k", lw=1.2))
    ax.text((gt_e[-1] + slam["x"][-1]) / 2, (gt_n[-1] + slam["y"][-1]) / 2 - 1.6,
            f"final drift {err_slam[-1]:.1f} m", ha="center", fontsize=9, weight="bold")
    ax.annotate("N", xy=(0.95, 0.95), xytext=(0.95, 0.86), xycoords="axes fraction", ha="center", fontsize=12, weight="bold",
                arrowprops=dict(arrowstyle="-|>", lw=2))
    ax.set_xlabel("East of take-off point [m]")
    ax.set_ylabel("North of take-off point [m]")
    ax.set_title(f"UAV autonomous mission (dynamic scene) - position map\n"
                 f"GPS origin {lat0:.6f} N, {lon0:.6f} E, {alt0:.1f} m  |  {t_fc[-1]:.0f} s mission")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend(loc="lower left", fontsize=8, framealpha=0.95)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig1_uav_position_map.png"), dpi=160)
    plt.close(fig)

    # ---------------- Figure 2: time series
    fig, axs = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    axs[0].plot(gt["t"], gt_u, color="#555555", lw=2, alpha=0.5, label="ground truth")
    axs[0].plot(gps["t_sim"], gps_u, ".", ms=2, color="#1f77b4", label="GPS altitude (rel. to start)")
    axs[0].plot(t_s, slam["z"], color="#d62728", lw=1.2, label="SLAM z")
    axs[0].set_ylabel("Height [m]")
    axs[0].legend(fontsize=8, loc="upper right")
    axs[0].set_title("UAV telemetry vs mission time (FC time, fc_time_ms / 1000)")
    axs[1].plot(gt["t"], gt_yaw_enu, color="#555555", lw=2, alpha=0.5, label="ground truth")
    axs[1].plot(t_s, slam_yaw, color="#d62728", lw=1, label="SLAM yaw (from quaternion)")
    axs[1].set_ylabel("Heading, ENU [deg]\n(0 = East, 90 = North)")
    axs[1].legend(fontsize=8, loc="upper right")
    a2 = axs[2]
    a2.plot(t_fc, fc["battery_v"], color="#2ca02c", lw=1.2, label="battery_v")
    a2.set_ylabel("Battery [V]")
    a2b = a2.twinx()
    a2b.step(t_fc, fc["battery_pct"], color="#9467bd", lw=1.2, where="post", label="battery_pct")
    a2b.set_ylabel("Battery [%]")
    a2.legend(loc="upper right", fontsize=8)
    a2b.legend(loc="center right", fontsize=8)
    modes = fc["mode"]
    codes = {m: i for i, m in enumerate(["HOLD", "OFFBOARD", "LAND", "FAILSAFE_HOLD"])}
    axs[3].step(t_fc, [codes.get(m, -1) for m in modes], where="post", color="k", lw=1.2, label="mode")
    axs[3].step(t_fc, fc["armed"] * 0.8 - 1.4, where="post", color="#ff7f0e", lw=1.5, label="armed (high = armed)")
    axs[3].set_yticks(list(codes.values()))
    axs[3].set_yticklabels(list(codes.keys()))
    axs[3].set_ylabel("FC status")
    axs[3].legend(fontsize=8, loc="center right")
    axs[3].set_xlabel("Mission time [s]")
    for a in axs:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig2_uav_timeseries.png"), dpi=150)
    plt.close(fig)

    # ---------------- Figure 3: data quality
    fig, axs = plt.subplots(3, 1, figsize=(11, 10))
    axs[0].plot(t_s, err_slam, color="#d62728", label="SLAM horizontal error vs ground truth")
    axs[0].plot(gps["t_sim"], err_gps, color="#1f77b4", lw=0.8, alpha=0.8, label="GPS horizontal error vs ground truth")
    axs[0].plot(t_s, gps_minus_slam, "--", color="k", lw=1, label="|GPS - SLAM| (detectable on board)")
    axs[0].axhline(3.0, color="gray", ls=":", lw=1)
    axs[0].text(2, 3.2, "3 m consistency threshold", fontsize=8, color="gray")
    axs[0].set_ylabel("Horizontal error [m]")
    axs[0].set_xlabel("Mission time [s]")
    axs[0].set_title("Issue 1 - SLAM drift (status stays 'OK' throughout)")
    axs[0].legend(fontsize=8)
    axs[1].plot(fc["t_wall"] - fc["t_wall"][0], t_fc, color="k", lw=1.2, label="fc_time_ms / 1000")
    axs[1].plot([0, fc["t_wall"][-1] - fc["t_wall"][0]], [0, fc["t_wall"][-1] - fc["t_wall"][0]], ":", color="gray",
                label="real time (slope 1)")
    axs[1].set_xlabel("Wall-clock time since first row, t_wall [s]")
    axs[1].set_ylabel("FC / simulation time [s]")
    axs[1].set_title(f"Issue 2 - two clocks: simulation ran at {rtf:.2f}x real time; t_wall cannot be used as mission time")
    axs[1].legend(fontsize=8)
    bins = np.linspace(0, 0.2, 41)
    axs[2].hist(np.clip(dt_fc, 0, 0.2), bins=bins, color="#8c564b", alpha=0.8, label="fc_telemetry rows: delta fc_time")
    axs[2].hist(np.clip(dt_gps, 0, 0.2), bins=bins, color="#1f77b4", alpha=0.6, label="gps.csv rows: delta t_sim")
    axs[2].set_yscale("log")
    axs[2].set_xlabel("Interval between consecutive samples [s] (clipped at 0.2 s)")
    axs[2].set_ylabel("Count (log)")
    axs[2].set_title(f"Issue 3 - irregular sampling: {dup} duplicate FC timestamps (dt = 0), "
                     f"{len(gaps)} gaps > 0.5 s, max gap {max([g[1] for g in gaps], default=0):.2f} s")
    axs[2].legend(fontsize=8)
    for a in axs:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig3_data_quality.png"), dpi=150)
    plt.close(fig)

    p3 = practical3_feed()

    stats = {
        "uav_log": os.path.relpath(LOG, ROOT), "rows": {"fc_telemetry": len(t_fc), "gps": len(gps["t_sim"]),
                                                        "slam_pose": len(t_s), "ground_truth": len(gt["t"])},
        "gps_origin": [lat0, lon0, alt0], "mission_s": float(t_fc[-1]), "wall_s": float(fc["t_wall"][-1] - fc["t_wall"][0]),
        "rtf": float(rtf), "slam_states": {s: int(np.sum(slam["state"] == s)) for s in np.unique(slam["state"])},
        "slam_err_final_m": float(err_slam[-1]), "slam_err_max_m": float(np.nanmax(err_slam)),
        "slam_err_rms_m": float(np.sqrt(np.nanmean(err_slam ** 2))),
        "gps_err_rms_m": float(np.sqrt(np.nanmean(err_gps ** 2))), "gps_err_max_m": float(np.nanmax(err_gps)),
        "gps_minus_slam_final_m": float(gps_minus_slam[np.isfinite(gps_minus_slam)][-1]),
        "first_time_gps_slam_gt_3m": float(t_s[np.argmax(gps_minus_slam > 3.0)]) if np.any(gps_minus_slam > 3) else None,
        "fc_dt_median_s": float(np.median(dt_fc)), "fc_duplicate_timestamps": dup, "fc_gaps_gt_0p5s": gaps[:10],
        "fc_wall_dt_median_s": float(np.median(dwall)), "gps_dt_median_s": float(np.median(dt_gps)),
        "gps_dt_max_s": float(dt_gps.max()), "battery_v": [float(fc["battery_v"][0]), float(fc["battery_v"][-1])],
        "battery_pct_levels": batt_steps.tolist(), "landing_truth_en": [float(gt_e[-1]), float(gt_n[-1])],
        "landing_slam_en": [float(slam["x"][-1]), float(slam["y"][-1])], "landing_gps_en": [float(gps_e[-1]), float(gps_n[-1])],
        "modes": sorted(set(fc["mode"].tolist())), "nav_lost_ticks": summary["nav_stats"]["lost_ticks"],
        "link_lost_ticks": summary["nav_stats"]["link_lost_ticks"], "practical3": p3,
    }
    with open(os.path.join(HERE, "telemetry_stats.json"), "w") as f:
        json.dump(stats, f, indent=1)
    print(json.dumps(stats, indent=1))


def practical3_feed():
    msgs = [json.loads(l) for l in open(os.path.join(P3, "messages.jsonl"), encoding="utf-8")]
    st = {a: [m for m in msgs if m["msg_type"] == "STATUS" and m["src"] == a and m.get("delivery") != "DROPPED_LINK_DOWN"]
          for a in ("UAV", "UGV")}
    dropped = [m for m in msgs if m.get("delivery") == "DROPPED_LINK_DOWN"]
    ugv_t = np.array([m["stamp"] for m in st["UGV"]])
    dts = np.diff(ugv_t)
    gap_i = int(np.argmax(dts))
    ev = load_csv(os.path.join(P3, "ugv_estimate_vs_truth_eval_only.csv"))
    tk = [k for k in ev if k.startswith("est_y")][0]
    drift = np.hypot(ev["truth_x"] - ev["est_x"], ev["truth_y"] - ev[tk])
    walls = [m["wall_time"] for m in st["UGV"]]

    fig, axs = plt.subplots(1, 2, figsize=(15, 6.2), gridspec_kw={"width_ratios": [1.35, 1]})
    ax = axs[0]
    ax.add_patch(plt.Rectangle((-5, -5), 40, 30, fill=False, ec="red", lw=2, label="Safety boundary (40 x 30 m)"))
    u = np.array([m["pose"] for m in st["UAV"]])
    g = np.array([m["pose"] for m in st["UGV"]])
    sc = ax.scatter(u[:, 0], u[:, 1], c=u[:, 2], cmap="Blues", s=14, vmin=-1, vmax=6, label="UAV STATUS pose (colour = height)")
    ax.plot(g[:, 0], g[:, 1], "-o", ms=3, color="#2ca02c", lw=1, label="UGV STATUS pose (estimated)")
    ax.plot(ev["truth_x"], ev["truth_y"], color="#555555", lw=1.5, alpha=0.5, label="UGV ground truth (evaluation only)")
    a, b = st["UGV"][gap_i]["pose"], st["UGV"][gap_i + 1]["pose"]
    ax.plot([a[0], b[0]], [a[1], b[1]], color="red", lw=3)
    ax.annotate(f"no UGV status for {dts[gap_i]:.1f} s\n(radio link down)", (a[0], a[1]), xytext=(-150, 25),
                textcoords="offset points", fontsize=8, color="red", arrowprops=dict(arrowstyle="->", color="red"))
    tg = {"P1": (22.0, 16.0), "P2": (14.0, 5.0), "P3": (9.0, 13.0), "P4": (32.0, 4.0)}
    for k, (x, y) in tg.items():
        ax.plot(x, y, "D", color="orange", mec="k", ms=8)
        ax.annotate(k, (x, y), xytext=(6, 4), textcoords="offset points", fontsize=9, weight="bold")
    ax.plot(0, 0, "s", color="k", ms=9, label="Base station (origin)")
    plt.colorbar(sc, ax=ax, label="UAV height Up [m]", fraction=0.035)
    ax.set_xlabel("East [m] (mission_enu)")
    ax.set_ylabel("North [m] (mission_enu)")
    ax.set_title("Practical 3 base-station feed - UAV and UGV positions")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7.5, loc="upper left")
    ax = axs[1]
    for agent, col in (("UAV", "#1f77b4"), ("UGV", "#2ca02c")):
        ts = np.array([m["stamp"] for m in st[agent]])
        ax.plot(ts[1:], np.diff(ts), ".", color=col, label=f"{agent} STATUS interval")
    ax.axhline(3.0, color="red", ls=":", lw=1)
    ax.text(3, 3.15, "3 s link timeout", color="red", fontsize=8)
    ax2 = ax.twinx()
    ax2.plot(ev["t"], drift, color="#555555", lw=1.2, label="UGV odometry error vs truth")
    ax2.set_ylabel("UGV position error [m]")
    ax.set_xlabel("Mission time, stamp [s]")
    ax.set_ylabel("Interval between STATUS messages [s]")
    ax.set_title("Message gaps and UGV dead-reckoning drift")
    ax.legend(loc="upper left", fontsize=8)
    ax2.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig4_practical3_feed.png"), dpi=150)
    plt.close(fig)
    same_wall = sum(1 for i in range(1, len(walls)) if walls[i] == walls[i - 1])
    return {"status_msgs": {a: len(v) for a, v in st.items()}, "dropped": len(dropped),
            "ugv_gap_s": float(dts[gap_i]), "ugv_gap_from_s": float(ugv_t[gap_i]), "ugv_drift_max_m": float(drift.max()),
            "ugv_drift_final_m": float(drift[-1]), "ugv_consecutive_equal_wall_time": same_wall,
            "uav_battery_levels": sorted({m["battery_pct"] for m in st["UAV"]})}


if __name__ == "__main__":
    main()
