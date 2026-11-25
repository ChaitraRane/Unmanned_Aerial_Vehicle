"""Block diagrams for Practical 1 (UAV and UGV)."""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figures")
COL = {"power": "#f6c9a8", "compute": "#b9d3f2", "sensor": "#c9e8c3", "prop": "#e5c4ec", "comm": "#f4e7a1", "ext": "#dddddd"}


def box(ax, x, y, w, h, title, lines, kind):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.12",
                                fc=COL[kind], ec="#333333", lw=1.3))
    ax.text(x + w / 2, y + h - 0.22, title, ha="center", va="top", fontsize=9.5, weight="bold")
    ax.text(x + w / 2, y + h - 0.55, "\n".join(lines), ha="center", va="top", fontsize=7.6, linespacing=1.25)
    return {"l": (x, y + h / 2), "r": (x + w, y + h / 2), "t": (x + w / 2, y + h), "b": (x + w / 2, y), "c": (x + w / 2, y + h / 2)}


def arrow(ax, a, b, label="", color="#222222", both=False, off=(0, 0.12), style="-|>"):
    ax.annotate("", xy=b, xytext=a, arrowprops=dict(arrowstyle="<|-|>" if both else style, lw=1.4, color=color,
                                                    shrinkA=2, shrinkB=2))
    if label:
        ax.text((a[0] + b[0]) / 2 + off[0], (a[1] + b[1]) / 2 + off[1], label, ha="center", va="bottom", fontsize=7,
                color=color, bbox=dict(fc="white", ec="none", pad=0.6, alpha=0.9))


def legend(ax, x, y):
    items = [("power", "Power"), ("compute", "Compute"), ("sensor", "Sensors"), ("prop", "Mobility / propulsion"),
             ("comm", "Communication"), ("ext", "External")]
    for i, (k, s) in enumerate(items):
        ax.add_patch(FancyBboxPatch((x + i * 2.55, y), 0.35, 0.25, boxstyle="round,pad=0.01", fc=COL[k], ec="#333"))
        ax.text(x + i * 2.55 + 0.45, y + 0.12, s, va="center", fontsize=8)
    ax.plot([], [], color="#c0392b", lw=1.4, label="power")
    ax.text(x + 15.4, y + 0.12, "red = power   black = data/control   blue = mechanical", va="center", fontsize=7.5)


def uav():
    fig, ax = plt.subplots(figsize=(15, 9.2))
    ax.set_xlim(0, 20); ax.set_ylim(0, 12.2); ax.axis("off")
    ax.text(10, 11.9, "UAV - Disaster-response quadrotor (2.2 kg, X-frame)", ha="center", fontsize=14, weight="bold")
    cam = box(ax, 5.2, 9.0, 3.8, 2.3, "Intel RealSense D435", ["stereo depth + RGB", "640x480, 58 deg VFOV", "0.2-12 m used, 10 deg down"], "sensor")
    thr = box(ax, 9.6, 9.0, 3.6, 2.3, "FLIR Lepton 3.5", ["thermal 160x120", "57 x 44 deg", "(synthetic in sim)"], "sensor")
    gps = box(ax, 13.9, 9.0, 3.6, 2.3, "Matek GNSS + compass", ["u-blox M10 class", "10 Hz, ~0.6 m CEP", "mast-mounted, aft"], "sensor")
    pi = box(ax, 5.2, 5.2, 4.6, 2.8, "Raspberry Pi 5 (8 GB)", ["companion computer, ROS 2 Humble", "SLAM, YOLOv8n, mapping", "A* + VFH+, mission logic"], "compute")
    fc = box(ax, 12.0, 5.2, 4.8, 2.8, "Pixhawk 6C Mini (PX4)", ["STM32H743 flight controller", "IMU ICM-42688-P + BMI088", "barometer, 250 Hz control loop"], "compute")
    wifi = box(ax, 0.4, 5.7, 3.4, 1.8, "Wi-Fi 802.11ac", ["Pi 5 on-board radio", "ROS 2 / JSON to base"], "comm")
    base = box(ax, 0.4, 9.2, 3.4, 1.9, "Base station", ["operator + AI", "task coordinator"], "ext")
    rc = box(ax, 17.7, 5.6, 2.1, 2.0, "RC receiver", ["manual override", "(recommended)"], "comm")
    bat = box(ax, 0.4, 1.0, 3.8, 2.6, "6S LiPo battery", ["8400 mAh, 22.2 V nom.", "19.2-25.2 V, 0.65 kg", "failsafe 20.4 V / 15 %"], "power")
    pm = box(ax, 5.0, 0.6, 3.0, 1.5, "Power module", ["V/I sense -> FC"], "power")
    bec = box(ax, 5.0, 2.6, 3.0, 1.5, "5 V / 5 A BEC", ["Pi 5 supply"], "power")
    esc = box(ax, 10.2, 0.8, 3.4, 2.2, "4-in-1 ESC", ["DShot / PWM input", "22.2 V to motors"], "prop")
    mot = box(ax, 15.0, 0.6, 4.6, 3.2, "4 x S3110 800 KV motors", ["11 in propellers (r = 0.14 m)", "~12 N max per motor", "hover ~6000 rpm, T/W ~ 2.2"], "prop")
    arrow(ax, cam["b"], (6.4, 8.0), "USB 3.0")
    arrow(ax, thr["b"], (8.8, 8.0), "USB (PureThermal)", off=(0.6, 0.12))
    arrow(ax, gps["b"], fc["t"], "UART + I2C")
    arrow(ax, pi["r"], fc["l"], "MAVLink 2, UART 921600", both=True)
    arrow(ax, wifi["r"], pi["l"], "", both=True)
    arrow(ax, wifi["t"], base["b"], "ROS 2 / task msgs", both=True, off=(0.9, 0))
    arrow(ax, rc["l"], fc["r"], "SBUS", off=(0, 0.12))
    arrow(ax, (4.2, 1.35), pm["l"], "22.2 V", color="#c0392b")
    arrow(ax, (4.2, 3.0), bec["l"], "22.2 V", color="#c0392b")
    arrow(ax, (8.0, 1.2), (10.2, 1.6), "22.2 V", color="#c0392b", off=(0, -0.4))
    arrow(ax, (8.0, 1.7), (12.6, 5.2), "5 V + V/I", color="#c0392b", off=(-0.5, 0.2))
    arrow(ax, bec["t"], (6.5, 5.2), "5 V", color="#c0392b", off=(0.35, -0.2))
    arrow(ax, (14.0, 5.2), (11.9, 3.0), "DShot", off=(0.8, 0))
    arrow(ax, esc["r"], mot["l"], "3-phase", color="#2557a7")
    legend(ax, 0.4, 0.05)
    fig.savefig(os.path.join(OUT, "uav_block_diagram.png"), dpi=170, bbox_inches="tight")
    plt.close(fig)


def ugv():
    fig, ax = plt.subplots(figsize=(15, 9.2))
    ax.set_xlim(0, 20); ax.set_ylim(0, 12.2); ax.axis("off")
    ax.text(10, 11.9, "UGV - LiDAR-free skid-steer ground robot (0.50 x 0.34 m, 4 kg chassis)", ha="center", fontsize=14, weight="bold")
    cam = box(ax, 5.0, 9.0, 4.2, 2.3, "Intel RealSense D435", ["stereo depth + RGB", "640x480, 60 deg FOV", "0.25 m fwd, 0.18 m up"], "sensor")
    imu = box(ax, 9.9, 9.0, 3.8, 2.3, "MPU-6050 IMU", ["3-axis gyro + accel", "200 Hz, +-2000 deg/s", "bias random walk modelled"], "sensor")
    enc = box(ax, 14.4, 9.0, 4.4, 2.3, "Quadrature wheel encoders", ["Hall A/B, 4 wheels", "2048 ticks/rev (sim)", "50 Hz"], "sensor")
    jet = box(ax, 5.0, 5.2, 5.0, 2.8, "NVIDIA Jetson Nano 4 GB", ["JetPack 4.6.1, ROS 2", "VO + VIWO, mapping, YOLO11n", "A* + VFH+, path following"], "compute")
    mcu = box(ax, 12.2, 5.2, 4.4, 2.8, "Motor MCU (ESP32)", ["PWM generation", "encoder counting", "watchdog -> stop"], "compute")
    wifi = box(ax, 0.4, 5.7, 3.4, 1.8, "Wi-Fi 802.11ac", ["Intel 8265 M.2", "ROS 2 / JSON to base"], "comm")
    base = box(ax, 0.4, 9.2, 3.4, 1.9, "Base station", ["operator + AI", "task coordinator"], "ext")
    estop = box(ax, 17.4, 5.6, 2.4, 2.0, "E-stop", ["cuts motor", "power"], "ext")
    bat = box(ax, 0.4, 1.0, 3.8, 2.6, "3S Li-ion battery", ["11.1 V, 5000 mAh", "(assumed rating)", "~55 Wh"], "power")
    buck = box(ax, 5.0, 1.4, 3.6, 1.8, "5 V / 4 A buck", ["Jetson + MCU supply"], "power")
    drv = box(ax, 10.0, 1.0, 3.6, 2.4, "Dual H-bridge driver", ["Cytron MDD10A class", "left / right channels"], "prop")
    mot = box(ax, 15.0, 0.6, 4.6, 3.2, "4 x 12 V DC gear motors", ["skid-steer, r = 0.08 m wheels", "track 0.38 m, base 0.34 m", "~0.7 m/s max (mission)"], "prop")
    arrow(ax, cam["b"], (6.5, 8.0), "USB 3.0")
    arrow(ax, imu["b"], (9.4, 8.0), "I2C (GPIO hdr)", off=(0.5, 0.12))
    arrow(ax, enc["b"], mcu["t"], "A/B pulses")
    arrow(ax, jet["r"], mcu["l"], "USB-serial: v, w / ticks", both=True)
    arrow(ax, wifi["r"], jet["l"], "", both=True)
    arrow(ax, wifi["t"], base["b"], "ROS 2 / task msgs", both=True, off=(0.9, 0))
    arrow(ax, estop["l"], mcu["r"], "")
    arrow(ax, bat["r"], (5.0, 2.3), "11.1 V", color="#c0392b")
    arrow(ax, (4.2, 1.4), (10.0, 1.4), "", color="#c0392b")
    ax.text(7.0, 0.95, "11.1 V", color="#c0392b", fontsize=7, ha="center")
    arrow(ax, buck["t"], (7.5, 5.2), "5 V", color="#c0392b", off=(0.35, -0.6))
    arrow(ax, (13.3, 5.2), (12.4, 3.4), "PWM + DIR", off=(0.8, 0))
    arrow(ax, drv["r"], mot["l"], "motor V", color="#2557a7")
    legend(ax, 0.4, 0.05)
    fig.savefig(os.path.join(OUT, "ugv_block_diagram.png"), dpi=170, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    uav()
    ugv()
