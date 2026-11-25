"""Import bootstrap: makes the two ORIGINAL projects importable without copying or modifying them."""
from __future__ import annotations

import os
import sys
import types

NEW_ROBOT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORKSPACE = os.path.dirname(NEW_ROBOT_ROOT)
UAV_ROOT = os.environ.get("UAV_ROOT", WORKSPACE)
UGV_SIM_DIR = os.environ.get("UGV_SIM_DIR", os.path.join(WORKSPACE, "robot_vision", "robot_vision", "simulation"))
UGV_URDF = os.path.join(UGV_SIM_DIR, "simple_robot.urdf")


class _Open3dStub(types.ModuleType):
    def __getattr__(self, name):
        raise RuntimeError("open3d is not installed; the coordination experiment does not use the UGV point-cloud viewer")


def setup_paths():
    for pth in (UAV_ROOT, UGV_SIM_DIR, NEW_ROBOT_ROOT):
        if pth not in sys.path:
            sys.path.insert(0, pth)
    try:
        import open3d  # noqa: F401
    except Exception:
        sys.modules["open3d"] = _Open3dStub("open3d")
    for p_ in (UAV_ROOT, UGV_SIM_DIR):
        if not os.path.isdir(p_):
            raise FileNotFoundError(f"required project folder not found: {p_}")


setup_paths()
