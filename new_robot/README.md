# new_robot — combined UAV + UGV + base-station coordination experiment (Practical 4)

One simulated **UAV**, one simulated **UGV** and a **base station with an AI-assisted task coordinator**, all in **one
PyBullet world on one simulation clock**. The two existing projects are reused in place, not copied or modified:

| Project | Location | Reused here |
|---|---|---|
| UAV (disaster UAV, MAVLink FC) | `..` (`D:\disaster_uav_pybullet_mavlink_fc`) | `UavSim` physics / motors / battery / IMU / GPS / simulated FC, `MavlinkFcServer` + `CompanionMavlink` (real MAVLink 2 over UDP), RGB-D + thermal sensors, `SlamFrontEnd` + `orb_pnp_vo` (ORB-SLAM3 on Linux), `PerceptionMappingPipeline` (YOLOv8n person detector + thermal fusion + geo-semantic map), `MissionController` (A* + VFH+ + fail-safes), URDF `urdf/disaster_uav.urdf`, world textures/primitives/human model |
| UGV / AGV (robot_vision) | `..\robot_vision\robot_vision\simulation` | `simple_robot.urdf` (4-wheel differential drive), `stage2_pointcloud` wheel motors + RGB-D camera, `viwo_sensors.SimulatedSensorSuite` (IMU + encoders), `VisualOdometry`, `ViwoEstimator` (error-state EKF), `GroundPlaneEstimator`, `OccupancyMap`, `GlobalPlanner` (A*), `LocalPlanner` (gap selection / emergency stop), `arc_wheel_speeds` |

New code (this folder) is only the coordination layer: site world, shared stepping, message format, simulated radio,
base station, allocator, the two agent wrappers, dashboard, fault injector, logs, tests.

## Install

Everything runs in the UAV project's existing micromamba environment **`uavros`** (it has pybullet, pymavlink,
onnxruntime, opencv, numpy, scipy, PyYAML). ROS 2 is **not** needed for this experiment. The `uav` env lacks onnxruntime
(required by the UAV person detector), so use `uavros`. On a fresh machine: `pip install -r ..\requirements.txt`.
`open3d` (used only by the UGV's point-cloud viewer) is **not** required: a stub is registered when it is missing.

## Run

PowerShell (from this folder):

```powershell
.\run.ps1 -Tests                              # fast coordination tests (6 tests, < 2 s, no simulator)
.\run.ps1                                     # normal mission
.\run.ps1 -Scenario fault_ugv_link            # PRIMARY fault: UGV radio/telemetry outage
.\run.ps1 -Scenario fault_uav_link            # optional secondary fault: UAV <-> base-station link outage
.\run.ps1 -Gui -Show                          # PyBullet GUI + live dashboard window (slower)
```

Equivalent direct command (any shell): `python run_experiment.py --scenario normal|fault_ugv_link|fault_uav_link
[--seconds N] [--mav-port 14550] [--gui] [--show]`. Use a different `--mav-port` per process to run two scenarios at once.
The seed (`config/mission.yaml: seed`) makes the radio, sensors and world deterministic; small run-to-run differences can
remain because the MAVLink UDP link and threads are real.

Runtime on the development laptop (Windows, software rendering): ~5–6 s wall per simulated second, i.e. ~20–30 min per run.

## Outputs — `results/<scenario>/`

| File | Content |
|---|---|
| `summary.md`, `summary.json` | final task states, fault timeline, offline evaluation (localisation error vs ground truth, boundary, collisions), MAVLink counters |
| `events.csv`, `events.json` | every coordination event: registration, initial status, allocation (with utility terms of every candidate), dispatch, progress, completion, next-task decision, fault injected / detected, interruption, pending, re-assignment, recovery |
| `telemetry_rx.csv` | every telemetry message received by the base station (stamp, age, site position, battery, mode, task, progress) |
| `agent_status.csv` | base-station view of each agent at 2 Hz (link state, telemetry age, position, battery, task queue) |
| `allocation_decisions.jsonl` | full allocator decision records (all candidates, terms, rejection reasons) |
| `radio_messages.jsonl` | every radio message with send / delivery time and fate (`QUEUED`, `DROPPED_LINK_DOWN`) |
| `mavlink_fc.log`, `mavlink_companion.log` | MAVLink traffic between the UAV companion agent and the simulated FC |
| `screenshots/*.png` | automatic screenshots: dashboard + `_world` PyBullet render (shared world, task allocation, completion, fault, re-assignment, recovery, final) |
| `simulation_recording.mp4` | full recording (1920×1080, 10 fps, 5 frames per simulated second): UAV camera, UGV camera, shared world, UAV map, UGV occupancy map + A\* path, operator dashboard |
| `closeup_recording.mp4` | (`--closeup`) real-time chase-camera close-ups of the UAV and the UGV, small onboard cameras, site overview, live status |
| `dashboard.mp4` | operator dashboard only (1 frame per simulated second) |
| `evidence_ugv/`, `evidence_uav/` | inspection evidence images taken by the vehicles' own cameras |
| `eval/gt_trace.csv` | ground truth, written for OFFLINE scoring only |

## Layout

```
config/mission.yaml                 site layout, tasks, comms timeouts, allocator weights, fault definitions
run_experiment.py, run.ps1          launchers
coordination_sim/
  bootstrap.py                      sys.path to the two original projects (+ open3d stub)
  frames.py                         site <-> UAV local / UGV map registration (surveyed launch poses)
  comms/messages.py, radio.py       common message format; simulated radio (latency, jitter, drop, forced outage)
  base_station/station.py           registry, stale/lost detection, task state machine, observation -> tasks, dispatch, logs
  base_station/allocator.py         explainable rule-based utility allocator
  agents/uav_agent.py               UAV companion: SLAM + perception + MissionController + MAVLink client + coordination I/O
  agents/ugv_agent.py               UGV onboard: VO + VIWO + OccupancyMap + A* + LocalPlanner + task executor + comm watchdog
  platform/                         SIMULATION SIDE ONLY: site world, shared UAV sim, UGV body/sensors
  viz/dashboard.py                  operator dashboard
  experiment.py                     shared stepping loop, fault injector, screenshots, summary
tests/test_coordination.py          allocator / stale detection / reassignment / firewall tests
PRACTICAL_4_REPORT.md               practical report
PSEUDOCODE.md                       pseudocode of the system
```

## Ground-truth firewall

`agents/`, `base_station/`, `comms/` never import `pybullet`, `sim`, `worlds` or `coordination_sim.platform`
(checked by `tests/test_coordination.py::test_ground_truth_firewall`). Vehicles report positions in their own estimation
frames; the base station converts them with the surveyed launch-pad registration. Ground truth is written only to
`eval/gt_trace.csv` and used only after the run.
