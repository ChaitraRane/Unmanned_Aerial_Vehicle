# Practical 4 — AI Model and Swarm Simulation for UAVs/UGVs

**System:** one UAV + one UGV + one AI-assisted base station, simulated together in PyBullet.

## 1. Aim and simulation setup

**Aim.** Build a simple, explainable AI coordination model for two heterogeneous robots. The agents report position,
battery, health, localisation, task status and sensor observations to a base station, which assigns, tracks and re-plans
inspection tasks. A stale-telemetry fault is injected to show safe stopping and task reallocation.

**Setup.**

| Item | Value |
|---|---|
| Simulator | PyBullet, one client, one 500 Hz timeline shared by both vehicles |
| Site | 24 m × 31 m outdoor yard with a painted operating boundary, 4 inspection zones, base station with antenna mast, 1.3 m container passage, pump house with valve skid, warehouse, standing casualty, perimeter gate, overheating cable drum |
| UAV | existing `disaster_uav.urdf` quadrotor with the original physics, motors, battery and simulated Pixhawk-style FC, commanded over **MAVLink 2 / UDP**; RGB-D + thermal camera; SLAM `orb_pnp_vo` (ORB-SLAM3 is not built on Windows); YOLOv8n person detector; geo-semantic map; `MissionController` (A\* + VFH+ + fail-safes) |
| UGV | existing `simple_robot.urdf` differential drive; RGB-D camera, IMU, wheel encoders; custom VO → VIWO error-state EKF → occupancy map → A\* → local gap planner → wheel motors |
| Base station | registry, link monitor, rule-based utility allocator, task state machine, operator dashboard |
| Seed | 4 (radio, sensors and world are deterministic) |

Launch commands (from `new_robot/`):

```
.\run.ps1 -Scenario normal
.\run.ps1 -Scenario fault_ugv_link
.\run.ps1 -Tests
```

## 2. UAV–UGV–base-station architecture

```
 UAV companion (existing autonomy)                 BASE STATION                    UGV onboard (existing autonomy)
 SLAM -> perception/map -> MissionController   <-- radio -->  coordinator  <-- radio -->  VO -> VIWO -> map -> A* -> local planner
          |  MAVLink 2 (UDP)                    telemetry / observations / task commands          |  wheel speeds
 simulated FC -> motors -> PyBullet   <=========== one shared PyBullet world, one clock ===========>  wheel motors -> PyBullet
```

- **High level vs low level.** The base station sends only goals, priorities, RESUME and CANCEL. All motion is produced by
  the vehicles' own controllers: the UAV through `MissionController` and the FC over MAVLink, the UGV through its planner
  and its differential-drive law.
- **Message format.** One schema for both vehicles (`TELEMETRY`, `OBSERVATION`, `TASK_COMMAND`, `TASK_ACK`, `HEARTBEAT`,
  `REGISTER`). Every message carries a simulation-time stamp.
- **Frames.** Each vehicle reports in its own estimation frame. The UAV SLAM frame is ENU with its origin at the
  take-off pad. The UGV Stage-8 map frame is X right, Y down, Z forward from its first camera pose. The base station
  converts both to the site frame using the surveyed launch poses.
- **Ground-truth firewall.** Autonomy and coordination code never imports the simulator. Ground truth is written only
  to `eval/gt_trace.csv` and used after the run.

## 3. Agent inputs and AI coordination rule

**Agent inputs (telemetry, 2 Hz):** estimated position and heading, localisation validity and confidence, battery
(the UAV value comes from MAVLink `SYS_STATUS`), health, mode, link state, current task, task state and progress,
active fail-safe.
**Observations:** confirmed persons and thermal hazards (UAV), inspection results with evidence images (UGV), and
down-sampled occupancy maps from both vehicles.

**Model type:** rule-based / heuristic utility. Nothing is trained and no neural network makes the decisions.

1. **Hard feasibility rules:** the capability matches the task (`GROUND_INSPECT` → UGV only, `AERIAL_SURVEY` → UAV only,
   `VISUAL_CHECK` → either); the link is HEALTHY; no fail-safe or fault is active; localisation is valid with
   confidence ≥ 0.4; battery − estimated energy ≥ reserve; the agent's queue has free capacity.
2. **Utility:**

```
U = 1.0·priority − 0.5·ETA/60 − 0.8·energy/(battery − reserve) + 0.4·loc_conf
    + 0.3·(completed+1)/(completed+failed+2) + 0.35·preference + 0.10·commitment
```

3. **Greedy assignment.** The best feasible (task, agent) pair is taken first. Queued tasks that have not been dispatched
   yet are re-auctioned, so a newly reported high-priority casualty moves ahead of them. Every decision is logged with
   every candidate's terms or rejection reason (`allocation_decisions.jsonl`).
4. **Next-task recommendation.** After each completion the remaining feasible tasks are ranked for that agent and the
   recommendation is logged (`NEXT_TASK_DECISION`).
5. **Shared situational awareness:**
   - UAV detections create new tasks, with duplicates within 3 m suppressed.
   - UAV thermal hazards become UGV keep-out zones.
   - Tasks flagged *requires aerial clearance* are released only once the UAV has covered their zone.

## 4. Simulation workflow and observed task progress

Measured run: `fault_ugv_link`, seed 4. It covers the full nominal workflow plus the fault. It lasted 172 s of simulated
time and stopped automatically once every task was terminal and the UAV had landed.

| t [s] | Event |
|---|---|
| 0.2 | MAVLink companion ↔ FC link up (UDP) |
| 0.4 / 1.8 | UAV and UGV register; initial status: UAV battery 95 %, UGV 100 % |
| 1.8 | **Initial allocation:** T1 aerial survey → UAV (U = 0.87); T2 valve-skid inspection → UGV (U = 0.88, the UAV is infeasible for GROUND_INSPECT); T3 gate check → UGV (UGV 0.99 vs UAV 0.52) |
| 2.4–6.0 | UAV: ARM → OFFBOARD → TAKEOFF → SEARCH over MAVLink. UGV drives through the 1.3 m container passage |
| 22.5 | **Stage 1 → 2:** the UAV's detector confirms a person at site (1.7, 14.8) (ground truth (1.8, 15.0)). The base station creates T5 "casualty check" (GROUND_INSPECT, priority 0.95); re-auction places it ahead of T3 in the UGV queue |
| 27.5 | UAV thermal hazard at (−2.0, 20.8) → UGV keep-out zone (r = 2.5 m) |
| 33.1 | UAV aerial coverage of zone Z-D reaches 43 % → T4 released → UAV (UGV infeasible: queue full) |
| 35.4 | **First complete cycle:** T2 COMPLETED by the UGV (object observed at 2.24 m, expected 2.38 m, evidence image saved) → NEXT_TASK_DECISION: T5 → dispatched |
| 66.1 | T1 aerial survey COMPLETED → next for the UAV: T4 |
| 88.1 / 114.1 | T4 and T3 COMPLETED by the UAV (stand-off waypoints inserted into its mission before the return leg) |
| 172.0 | UAV has landed; all 5 tasks COMPLETED |

**Stage 4 (shared situational awareness).** Both vehicles' occupancy maps, the UAV coverage, the keep-outs and the
detections are shown in one site frame on the dashboard. A person reported again within 3 m does not create a duplicate
task.

## 5. Fault injection and safe response

**Fault.** At t = 42.1 s, while the UGV was executing T5 (17 % progress, T3 queued behind it), the UGV radio link was
forced down in both directions for 22 s. The rest of the simulation kept running.

| t [s] | Response |
|---|---|
| 42.1 | FAULT_INJECTED. The last UGV telemetry was stamped 41.9 s |
| 44.0 | **FAULT_DETECTED:** telemetry 2.10 s old > 2.0 s timeout → UGV-1 UNAVAILABLE. Its last known position is shown as "LAST KNOWN (age)" and is not treated as current |
| 44.0 | T5 (ground-only) INTERRUPTED → **PENDING**: "UAV-1: GROUND_INSPECT requires ugv; UGV-1: link STALE". It was **not** given to the UAV |
| 44.0 | T3 (both-capable) un-queued from the UGV → **REASSIGNED to UAV-1** (the only feasible agent, U = −0.09); a waypoint is inserted into the flying UAV's mission |
| 44.7 | **UGV onboard watchdog:** no base-station heartbeat for 2.6 s → COMM_HOLD, wheels stopped, task HELD. The UGV did not backtrack (20.2 s in hold) |
| 50.0 | LINK_LOST (> 8 s without telemetry) |
| 64.1 | Link restored. UGV mode COMM_HOLD → TASK_HELD (it waits for authorisation) |
| 66.0 | LINK_RECOVERED after 3 consecutive fresh messages → T5 re-allocated to the UGV and sent as **RESUME** |
| 82.8 | T5 COMPLETED by the UGV; T3 COMPLETED by the UAV at 114.1 s |

The UAV's own fail-safes (LOST_HOLD, LINK_LOST, FC FAILSAFE_HOLD, LANDING) were left unchanged and were not triggered.
No ground-truth position was used online.

## 6. Results and limitations

| Metric | Result |
|---|---|
| Tasks completed | 5 / 5: UAV 3 (T1, T3, T4), UGV 2 (T2, T5) |
| Fault detection latency | 1.9 s after injection (2.0 s timeout from the last message) |
| Ground-only task during the fault | kept PENDING, never given to the UAV |
| Both-capable task during the fault | re-assigned to the UAV and completed |
| UGV during the outage | stopped (COMM_HOLD 20.2 s), resumed only on RESUME |
| UAV site-position error (offline vs ground truth) | RMSE 0.35 m, max 0.62 m (orb_pnp_vo) |
| UGV site-position error | RMSE 0.20 m, max 0.34 m (VIWO; 854/854 VO updates succeeded) |
| Boundary / collisions | both vehicles always inside the boundary; 0 collisions |
| MAVLink | 1701 SET_POSITION_TARGET_LOCAL_NED, 10 COMMAND_LONG, 345 SYS_STATUS (battery) messages |
| Radio | 1507 messages sent, 1412 delivered, 92 dropped during the outage |
| Unit tests | 6/6 pass (allocator rules, stale detection, reallocation, recovery, old-telemetry rejection, ground-truth firewall) |

**Observed weakness.** The UGV's casualty inspection (T5) ended "object NOT confirmed". It measured 8.93 m in the image
centre against an expected 2.02 m: the narrow person did not fill the central depth patch. The task counted as completed
(the vehicle reached the point and recorded evidence), but its result is honestly flagged as unconfirmed. A target
detector on the UGV camera would fix this.

**Not run for this submission:** the separate `normal` scenario and the optional `fault_uav_link` scenario were not run
in the time available, although both launch commands exist. The nominal workflow is fully contained in the run above.

**Limitations**

- **Simplified communication.** The radio is logical: latency, jitter, drops and forced outage only. There is no RF
  propagation, range or bandwidth model and no RSSI.
- **Limited map sharing.** The two maps are registered only through the surveyed start poses. They are not merged by
  matching their content, so estimator drift appears directly as an offset in the site frame. The base station shows
  both maps but does not fuse them.
- **SLAM backend.** On Windows the UAV uses the `orb_pnp_vo` fallback, which has no loop closure. ORB-SLAM3 is only
  validated on Ubuntu.
- **UGV navigation.** The UGV's route-instruction navigation controller (topological missions, lane tracking) is not
  used. A small goal follower drives the reused A\* and local gap planner.
- **Simulated sensing and energy.**
  - Rendering is synthetic and the thermal image is synthetic.
  - The UGV battery is a simple energy model.
  - The UAV's inspection evidence is a camera frame at the stand-off point; the camera is not deliberately aimed at the
    target.
- **Not repeatable bit-for-bit.** The MAVLink UDP link and threads are real, so the seed fixes everything except tiny
  timing differences.
- **No physical validation.** Everything was tested in simulation only.

## Evidence

- Video: `results/fault_ugv_link/simulation_recording.mp4` (UAV camera, UGV camera, shared world, both maps, dashboard)
- Close-up video: `results/fault_ugv_link_closeup/closeup_recording.mp4` (real-time chase views of both vehicles; repeat run, identical task timeline)
- Screenshots: `results/fault_ugv_link/screenshots/`
- Logs: `events.csv`, `telemetry_rx.csv`, `agent_status.csv`, `allocation_decisions.jsonl`, `radio_messages.jsonl`, `mavlink_*.log`
- Summary: `results/fault_ugv_link/summary.md`, `summary.json`
- Pseudocode: `PSEUDOCODE.md`
