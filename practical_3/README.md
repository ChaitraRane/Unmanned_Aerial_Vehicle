# Practical 3 – Collaborative Task Planning

Run (Windows, from the repository root):

```powershell
. .\scripts\env_micromamba.ps1 uavros
& $env:UAV_PYTHON practical_3\run_practical3.py
```

| Path | Content |
|---|---|
| `run_practical3.py` | Launcher: shared world, UAV (MAVLink FC), UGV, base station, recorder |
| `config/practical3_tasks.yaml` | Targets, task allocation, priorities, handover and comms settings |
| `p3/` | World, frames, message bus, coordinator, UAV agent, UGV agent, visualisation |
| `PRACTICAL_3_REPORT.md` | Report |
| `PRACTICAL_3_PSEUDOCODE.md` | Structured pseudocode |
| `outputs/run_20261104_190102/` | Submitted run: video, screenshots, logs, mission report |
