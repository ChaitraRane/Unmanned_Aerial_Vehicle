# Practical 3 – Collaborative Task Planning (UAV + UGV + Base Station)

## 1. Aim

To plan and demonstrate a collaborative inspection mission in which one UAV and one UGV, coordinated by a base station with an AI task coordinator, share an outdoor inspection site. The mission demonstrates:

- capability-based task allocation;
- an explicit task sequence with status messages;
- a UAV-to-UGV handover with acknowledgement;
- a shared ENU reference frame that keeps coordinates consistent between platforms.

The whole plan is executed in a running PyBullet simulation.

## 2. System Used

| Element | Implementation (reused) |
|---|---|
| UAV | `urdf/disaster_uav.urdf`; physics, motors, battery and simulated flight controller from `sim/` (`UavSim`). It is commanded **only over MAVLink 2 / UDP** (`companion/mavlink_client.py` → `sim/mavlink_fc_server.py`): ARM, OFFBOARD and LAND via `COMMAND_LONG`, and velocity setpoints via `SET_POSITION_TARGET_LOCAL_NED`. The FC setpoint watchdog and failsafes are unchanged. |
| UAV perception | Onboard RGB-D camera model (`sim/rgbd_camera.py`). Markers are found by colour segmentation and localised in 3-D using depth, the URDF camera extrinsics and the FC attitude and position. |
| UGV | `robot_vision/.../simulation/simple_robot.urdf`: four-wheel skid-steer with an RGB-D camera, wheel encoders and a gyro. |
| UGV autonomy | Wheel–gyro odometry with zero-velocity gyro-bias calibration, a log-odds occupancy grid built from depth, the **A\*** global planner reused from `planning/astar.py`, pure-pursuit path following, and a front-obstacle guard with stop, re-plan and reverse. |
| Base station | Building with an antenna mast at the mission origin. It provides a message bus (100 ms latency, per-agent link state and timestamped JSON log) and the AI coordinator (`p3/coordinator.py`). |
| Environment | 40 m × 30 m site with a red safety boundary. It contains: an elevated lattice tower (P1), a ground equipment cabinet (P2), an environmental-monitoring station (P3), and an unexplored region behind a brick wall reachable only through a 2.0 m passage (P4). Barrels and crates act as obstacles. |

## 3. Task-Allocation Table

| Task | Inspection point | Agent | Required capability | Action and completion evidence |
|---|---|---|---|---|
| T1 | Entire area | UAV | aerial_survey | Lawn-mower survey at 5 m. Completion requires target reports with ≥ 5 observations and σ < 1.5 m. |
| T2 | P1 elevated structure | UAV | elevated_inspection | Hold at a 5 m standoff facing P1 for 6 s. Completion requires the marker in ≥ 5 frames, plus images. |
| T3 | P2 ground equipment | UGV | ground_inspection | Close-up at 1.6 m standoff. Completion requires images, the marker in ≥ 3 frames, and the target re-measured by the UGV. |
| T4 | P3 environmental monitoring | UGV | env_sensor | Navigate and sample the environmental sensor (**simulated**, see §8) for 4 s. |
| T5 | P4 unexplored region | UGV | ground_inspection | Pass through the narrow passage, record images, do a 360° local-mapping scan and count the newly mapped cells. |
| T6 | Final verification | UAV + UGV | joint | Final status from both agents, verification of T1–T5, report generation, and a safe end state. |

**Allocation rule.** The UAV takes wide-area and elevated work: it has a wide field of view and altitude, and it can reach the top of the 5 m structure. The UGV takes close-range ground work: it can hold a camera 1.6 m from equipment, carry ground sensors and map tight spaces. The allocation is editable in `config/practical3_tasks.yaml`.

## 4. Workflow and Handover Conditions

| Stage | Event (as logged) |
|---|---|
| 1. Initialisation | Both agents send `REGISTER` with: agent ID, pose and `frame_id`, battery, health, localisation state, current task and comm status. The coordinator checks availability (registered, link alive, health OK, battery ≥ 25 %, localisation VALID) before assigning anything. |
| 2. Survey (T1) | `TASK_ASSIGNMENT T1 → UAV`, then `TASK_ACK`, then `TASK_PROGRESS` at 1 Hz. The UAV sends a `TARGET_REPORT` (in `uav_local_ned`) each time a region of interest is confirmed. |
| 3. Classification | Each report is transformed into `mission_enu` and validated (finite, σ ≤ 1.5 m, ≥ 3 observations, inside the boundary). It is then classified: **z ≥ 2 m → AERIAL/ELEVATED → UAV**, otherwise **GROUND → UGV**. Agent capability and availability are checked before assignment. |
| 4. UAV inspection (T2) | Assigned after T1 completes. The base station marks T2 `COMPLETED` only when a `TASK_COMPLETE` arrives with `inspection_performed = true`. |
| 5. Handover | The coordinator converts the UAV target from `uav_local_ned` to `mission_enu` to `ugv_odom`. It then sends a `TASK_ASSIGNMENT` containing both coordinate sets, the priority, the UAV observations and the UAV image. The UGV re-checks the transform (< 5 cm), plans an A\* path, and only then returns `TASK_ACK ACCEPTED` and starts moving. |
| 6. UGV inspection (T3–T5) | After each `TASK_COMPLETE`, the coordinator recommends the next pending ground task, ordered by priority and then distance. |
| 7. Verification (T6) | The coordinator sends `FINAL_STATUS_REQUEST` to both agents and verifies every task. It then writes `mission_report.json` and `mission_summary.md`, and sends `END_MISSION_SAFE`. The UAV is landed and disarmed by the FC; the UGV is parked. |

| Condition | Coordinator behaviour |
|---|---|
| A – UAV identifies a ground target | Validate, transform, classify as GROUND, and hand over to the UGV with ACK. |
| B – UAV completes an aerial inspection | Mark the task COMPLETED, recommend the next pending task (UGV ground work), and send the UAV `RETURN_AND_STANDBY`. |
| C – UGV completes a ground inspection | Record the results, update status, and assign the next eligible ground task. |
| D – Robot unavailable or comm timeout (> 3 s) | Put the active task on **HOLD**. Reassign only if the other agent has the required capability and is available; otherwise keep the task on HOLD and resume it when the link returns. The UGV's own fail-safe (no base heartbeat for 3 s → stop) acts independently. |

Task states used: `PENDING, ASSIGNED, ACCEPTED, IN_PROGRESS, COMPLETED, FAILED, HOLD`. Structured pseudocode is in `PRACTICAL_3_PSEUDOCODE.md`.

## 5. Shared Reference Frame

- **mission_enu**: origin at the base station (0, 0, 0); X = East, Y = North, Z = Up. All task messages carry `frame_id` and a timestamp (`stamp` in simulation time, plus `wall_time`).
- **uav_local_ned**: the FC's EKF local frame, received over MAVLink `LOCAL_POSITION_NED`. It is registered once, on the surveyed take-off pad, from the FC position reported there:
  `T_mission←uav = (R_ENU←NED, t = pad_mission − R·p0_ned)`. At run time this gave `t = [15.01, 10.01, −0.14]` and yaw 90° (axis swap N/E, D → −Up).
- **ugv_odom**: origin at the dock, x along the docked heading. It is registered from the surveyed dock pose: `T_mission←ugv = (Rz(30°), t = [3.0, −2.5, 0])`.
- The two SLAM/odometry frames are **not** assumed to be aligned. All exchanges go through `mission_enu` using these verified initial transforms. Simulator ground truth never enters either agent; it is used only for the offline evaluation in `mission_report.json → targets_evaluation_only`.

**Handover example (T3, from `transforms.json`):**

| | Frame | x | y | z |
|---|---|---|---|---|
| UAV detection (source) | uav_local_ned | −5.04 | −1.37 | −1.23 |
| Shared | mission_enu | 13.64 | 4.97 | 1.09 |
| UGV navigation goal target | ugv_odom | 12.95 | 1.15 | 1.09 |
| UGV's own camera measurement after arrival | ugv_odom → mission_enu | 13.47 | 4.76 | – |

The UAV and UGV estimates of the same target, from independent sensors and independent frames, agree to within **0.29 m** (T3), **0.21 m** (T4) and **0.78 m** (T5). This cross-platform residual is computed at run time and shows that the frames are consistent.

## 6. Simulation Procedure

```powershell
. .\scripts\env_micromamba.ps1 uavros
& $env:UAV_PYTHON practical_3\run_practical3.py
```

Optional flags: `--no-fault` (skip the Condition D injection) and `--max-time 600`.

The run is a single process. The PyBullet world, simulated FC and MAVLink server run on UDP 14590. The companion MAVLink client, UAV agent, UGV agent, base station and recorder run alongside them. A fault injection takes the UGV radio link down for 7 s, 6 s after T4 is accepted.

Each run writes to `practical_3/outputs/run_<timestamp>/`:

| File | Content |
|---|---|
| `practical3_simulation.mp4` | 4-panel recording: site overview, live mission map with task table / UAV camera, UGV camera, plus an event ticker. 8 fps, 2× real time. |
| `events.log` | Timestamped coordinator and agent event log. |
| `messages.jsonl` | Every bus message: REGISTER, STATUS, TASK_ASSIGNMENT, TASK_ACK, TASK_PROGRESS, TARGET_REPORT, TASK_COMPLETE, HEARTBEAT, COMMAND, including dropped messages. |
| `transforms.json` | Frame registrations, each handover with its source and transformed coordinates, and target accuracy. |
| `mission_report.json`, `mission_summary.md` | Final record and operator summary. |
| `fc_mavlink.log`, `companion_mavlink.log` | MAVLink traffic on both sides. |
| `screenshots/`, `images/` | Event screenshots, UAV region-of-interest detections, and UAV/UGV inspection images. |

## 7. Observations and Results

Final run `outputs/run_20261104_190102`: **mission COMPLETED, all six tasks COMPLETED**. The mission took 206.7 s of simulated time (293.7 s wall time).

**Event timeline (from `events.log`):**

| t [s] | Event |
|---|---|
| 0.2 | UGV and UAV register in `mission_enu`. Availability check passes for both. T1 is assigned to the UAV. |
| 2.5 | FC armed and switched to OFFBOARD over MAVLink. Take-off to 5 m, then a 3-lane survey. |
| 11.6 / 18.6 / 29.4 / 34.6 | ROIs confirmed: P2 yellow, P4 magenta, P1 orange, P3 cyan. Each is transformed, validated and classified: P1 → AERIAL/ELEVATED → UAV T2; P2, P3, P4 → GROUND → UGV T3, T4, T5. |
| 61.2 | T1 COMPLETED. T2 assigned to the UAV. |
| 75.0 | T2 COMPLETED: marker seen in 20 frames at 3.5 m range, with 2 images. Condition B triggers the UAV's return to the pad (landed and disarmed by the FC at 98 s). |
| 75.1–75.3 | **Handover T3**: UAV → base → UGV. The UGV acknowledges only after its A\* path is accepted. |
| 96.5 | T3 COMPLETED. Condition C recommends T4. Handover T4 is acknowledged. |
| 102.8 | Fault injection: the UGV radio link goes down. |
| 105.2 | **Condition D**: UGV communication timeout, so T4 goes to HOLD. Reassignment check: UAV `capable=False` (no env_sensor), so T4 stays on HOLD for the UGV. The UGV's onboard fail-safe stops it (COMM_LOSS_HOLD). |
| 109.8 | Link restored. T4 HOLD → IN_PROGRESS. The UGV resumes navigation. |
| 120.5 | T4 COMPLETED with simulated environmental readings. Handover T5 is acknowledged. The UGV crosses the 2 m passage into the unexplored region. |
| 202.4 | T5 COMPLETED: marker seen in 8 frames, 360° scan, 1132 newly mapped cells. |
| 202.7 | T6: final status from both agents, every task verified, mission COMPLETED. UAV landed and disarmed; UGV PARKED. |

**Target localisation by the UAV, in mission_enu (ground truth used for evaluation only):**

| Target | UAV estimate | Ground truth | Horizontal error |
|---|---|---|---|
| P1 tower (marker panels) | (22.98, 15.96, 4.96) | (22.0, 16.0, 5.0) | 0.98 m (panel face, not the tower centre) |
| P2 equipment | (13.64, 4.97, 1.09) | (14.0, 5.0, 0.9) | 0.36 m |
| P3 env. station | (9.15, 12.93, 0.98) | (9.0, 13.0, 1.0) | 0.16 m |
| P4 unexplored region | (31.95, 3.96, 0.08) | (32.0, 4.0, 0.3) | 0.06 m |

**Cross-platform consistency** (the UGV re-measures each handed-over target with its own camera in `ugv_odom`, then converts it to `mission_enu`): residuals of 0.27 m (T3), 0.21 m (T4) and 0.78 m (T5, after the longest drive). UGV odometry error against ground truth reached at most **0.90 m** over the whole mission (`ugv_estimate_vs_truth_eval_only.csv`).

**Simulated environmental sensor at P3**: 28.3 °C, 43.0 % RH, 565 ppm CO₂ (mean of 8 samples).

**Traffic**: 993 base-station messages, of which 17 were dropped while the UGV link was down. On the MAVLink side the FC received 1065 `SET_POSITION_TARGET_LOCAL_NED`, 13 `COMMAND_LONG` and 285 `HEARTBEAT` messages.

**Earlier attempt (kept for transparency, `outputs/attempt1_run_20261104_185108_T5_failed`)**: with a 1.4 m passage, the UGV clipped the wall edge and its wheels spun in place. The encoders kept counting distance, so odometry drifted by up to 13 m. The UGV then never saw the P4 marker. The coordinator correctly refused to mark T5 complete (`FAILED: target marker not observed`) and reported the mission as **INCOMPLETE** instead of claiming success. The fix for the final run was a wheel-slip detector (encoders moving while consecutive depth frames are static → stop integrating translation and back off), a 2.0 m passage and tighter path following.

## 8. Limitations

- **The Practical 4 combined project was not found** outside the excluded `new_robot` folder. This scenario was therefore built as a new module that reuses the UAV stack in this repository and the UGV URDF from `robot_vision`. No existing UAV, UGV or `new_robot` file was modified.
- **UAV localisation** uses the FC EKF estimate received over MAVLink. In this simulator that estimate is truth plus configured noise, standing in for a Pixhawk EKF. The repository's RGB-D SLAM, voxel mapping, A\* and VFH+ nodes (the ROS 2 chain) were not launched for this scenario, to keep one process. The survey altitude (5 m) is above every obstacle except the tower, and the route keeps more than 3 m from the tower.
- **UGV localisation** is wheel–gyro odometry with zero-velocity gyro-bias calibration. The visual part of the `robot_vision` VIWO estimator and its 12 k-line Stage-8 navigation stack are monolithic and were not extracted. The UGV instead uses the repository A\* planner on its own depth occupancy grid. Unknown cells are treated as traversable (optimistic ground planning), with a stop-and-replan guard.
- **Environmental sensor (T4) is simulated.** Neither robot has environmental hardware, so temperature, humidity and CO₂ come from a labelled synthetic field in the experiment layer (`env_sensor_sim`).
- **Inspection targets carry coloured markers** so that a lightweight detector can find them; no learned detector is used for inspection points.
- **Cross-platform registration** uses verified initial transforms (surveyed pad and dock). Map-to-map registration between the UAV and UGV maps is not implemented. The UGV's re-measurement of each target serves as the run-time consistency check.
- The camera feeds and overview are rendered with PyBullet's TinyRenderer, which runs at roughly half real time. The video is encoded at 2× speed.
