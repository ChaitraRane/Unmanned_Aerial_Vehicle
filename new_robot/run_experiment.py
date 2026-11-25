"""Practical 4 - combined UAV / UGV / base-station coordination experiment."""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from coordination_sim.experiment import run  # noqa: E402

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="normal", choices=["normal", "fault_ugv_link", "fault_uav_link"])
    ap.add_argument("--seconds", type=float, default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--mav-port", type=int, default=None, help="MAVLink UDP port (default from drone.yaml: 14540)")
    ap.add_argument("--closeup", action="store_true", help="record closeup_recording.mp4 (chase cameras of both vehicles, real time)")
    a = ap.parse_args()
    s = run(a.scenario, a.out, a.seconds, a.gui, show=a.show, mav_port=a.mav_port, closeup=a.closeup)
    print("\nFINAL TASK STATES:", s["task_states"])
    print("summary:", os.path.join(a.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "results", a.scenario), "summary.md"))
