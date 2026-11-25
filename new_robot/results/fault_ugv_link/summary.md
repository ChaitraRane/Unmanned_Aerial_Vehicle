# Run summary - scenario `fault_ugv_link`

- simulated time 172.0 s (wall 953.6 s), physics 500 Hz, seed 4
- UAV SLAM backend: `orb_pnp_vo` - ORB-SLAM3 runner not found under C:\Users\Chaitra Rane/slam_deps/ORB_SLAM3 (build it with slam/build_orbslam3.sh on Linux); using the orb_pnp_vo fallback (RGB-D ORB + PnP odometry, no loop closure)

## Final task states

| Task | Type | State | By | Completed at [s] | Interruptions | Result |
|---|---|---|---|---|---|---|
| T1 Aerial survey of site | AERIAL_SURVEY | COMPLETED | UAV-1 | 66.1 | 0 |  |
| T2 Close inspection: pump-house valve skid | GROUND_INSPECT | COMPLETED | UGV-1 | 35.4 | 0 | UGV ground view: object observed at 2.24 m (expected 2.38 m), evidence T2_t035.png |
| T3 Visual check: north-east perimeter gate | VISUAL_CHECK | COMPLETED | UAV-1 | 114.1 | 0 | UAV aerial visual check at 3.2 m from the target, altitude 2.0 m, image T3_check_t113.png |
| T4 Thermal/visual check: north-west bay cable drum | VISUAL_CHECK | COMPLETED | UAV-1 | 88.1 | 0 | UAV aerial visual check at 3.1 m from the target, altitude 2.0 m, image T4_check_t087.png |
| T5 Casualty check: person #1 reported by UAV | GROUND_INSPECT | COMPLETED | UGV-1 | 82.8 | 1 | UGV ground view: object NOT confirmed at 8.93 m (expected 2.02 m), evidence T5_t082.png |

## Fault

- `fault_ugv_link` on UGV-1: injected t=42.1, detected t=44.0 (stale timeout 2.0 s), cleared t=64.1

## Offline evaluation (ground truth, never used online)

- UAV site-frame position error RMSE 0.35 m (max 0.62 m)
- UGV site-frame position error RMSE 0.20 m (max 0.34 m)
- inside boundary: UAV True, UGV True
- collisions: UAV 0, UGV 0 {'ugv': [], 'uav': []}
- radio: {'sent': 1507, 'delivered': 1412, 'dropped_random': 0, 'dropped_link_down': 92}

## Coordination events

| t [s] | Category | Agent | Task | Message |
|---|---|---|---|---|
| 0.0 | TASK_CREATED |  | T1 | Aerial survey of site (AERIAL_SURVEY, priority 0.9, capable: uav) |
| 0.0 | TASK_CREATED |  | T2 | Close inspection: pump-house valve skid (GROUND_INSPECT, priority 0.8, capable: ugv) |
| 0.0 | TASK_CREATED |  | T3 | Visual check: north-east perimeter gate (VISUAL_CHECK, priority 0.55, capable: ugv/uav) |
| 0.0 | TASK_CREATED |  | T4 | Thermal/visual check: north-west bay cable drum (VISUAL_CHECK, priority 0.4, capable: uav/ugv) - waiting for aerial clearance |
| 0.0 | EXPERIMENT |  |  | scenario=fault_ugv_link seed=4 physics=500 Hz, one PyBullet client, UAV+UGV+base station |
| 0.0 | UAV_SLAM_BACKEND | UAV-1 |  | orb_pnp_vo (ORB-SLAM3 runner not found under C:\Users\Chaitra Rane/slam_deps/ORB_SLAM3 (build it with slam/build_orbslam3.sh on Linux); using the orb_pnp_vo fallback (RGB-D ORB + PnP odometry, no loop closure)) |
| 0.2 | MAVLINK_LINK | UAV-1 |  | companion <-> FC MAVLink up over UDP (udp:127.0.0.1:14550), FC heartbeat received |
| 0.4 | AGENT_REGISTERED | UAV-1 |  | uav registered, frame uav_local, capabilities ['uav'] |
| 0.4 | INITIAL_STATUS | UAV-1 |  | pos site=(0.0,-3.0) battery=95% loc_valid=False mode=IDLE_ON_PAD |
| 0.4 | LINK_HEALTHY | UAV-1 |  | telemetry fresh |
| 0.4 | TASK_PENDING |  | T1 | Aerial survey of site: no feasible agent -> stays PENDING: UAV-1: localisation not valid (conf 0.00); UGV-1: capability: AERIAL_SURVEY requires uav |
| 0.4 | TASK_PENDING |  | T2 | Close inspection: pump-house valve skid: no feasible agent -> stays PENDING: UAV-1: capability: GROUND_INSPECT requires ugv; UGV-1: no telemetry yet |
| 0.4 | TASK_PENDING |  | T3 | Visual check: north-east perimeter gate: no feasible agent -> stays PENDING: UAV-1: localisation not valid (conf 0.00); UGV-1: no telemetry yet |
| 1.2 | AGENT_REGISTERED | UGV-1 |  | ugv registered, frame ugv_map, capabilities ['ugv'] |
| 1.8 | INITIAL_STATUS | UGV-1 |  | pos site=(3.4,-3.4) battery=100% loc_valid=True mode=IDLE |
| 1.8 | LINK_HEALTHY | UGV-1 |  | telemetry fresh |
| 1.8 | TASK_ALLOCATED | UGV-1 | T2 | Close inspection: pump-house valve skid -> UGV-1 (utility 0.88, ETA 53 s, energy 1.1%) / UAV-1 infeasible (capability: GROUND_INSPECT requires ugv); UGV-1=0.88 |
| 1.8 | TASK_ALLOCATED | UGV-1 | T3 | Visual check: north-east perimeter gate -> UGV-1 (utility 0.99, ETA 51 s, energy 1.0%) / UAV-1=0.52; UGV-1=0.99 |
| 1.8 | TASK_ALLOCATED | UAV-1 | T1 | Aerial survey of site -> UAV-1 (utility 0.87, ETA 57 s, energy 5.1%) / UAV-1=0.87; UGV-1 infeasible (capability: AERIAL_SURVEY requires uav) |
| 1.8 | TASK_DISPATCHED | UAV-1 | T1 | ASSIGN sent; goal in UAV-1 frame: (-2.6,6.5), (-1.4,12.8), (-0.2,19.0) ... |
| 1.8 | TASK_DISPATCHED | UGV-1 | T2 | ASSIGN sent; goal in UGV-1 frame: (0.1,10.1), (0.6,11.9) |
| 2.0 | TASK_ACK | UGV-1 | T2 | accepted |
| 2.0 | TASK_ACK | UAV-1 | T1 | accepted (new sortie #1) |
| 2.4 | AGENT_MODE | UAV-1 | T1 | IDLE_ON_PAD -> ARM |
| 2.4 | TASK_ACTIVE | UAV-1 | T1 | UAV-1 reports ACCEPTED |
| 2.4 | AGENT_MODE | UGV-1 | T2 | IDLE -> EN_ROUTE |
| 2.4 | TASK_ACTIVE | UGV-1 | T2 | UGV-1 reports EN_ROUTE |
| 2.9 | AGENT_MODE | UAV-1 | T1 | ARM -> OFFBOARD |
| 3.4 | AGENT_MODE | UAV-1 | T1 | OFFBOARD -> TAKEOFF |
| 6.0 | AGENT_MODE | UAV-1 | T1 | TAKEOFF -> SEARCH |
| 22.5 | AGENT_MODE | UAV-1 | T1 | SEARCH -> INSPECT |
| 22.5 | OBSERVATION | UAV-1 |  | UAV confirmed person (track #1) at site (1.7,14.8), lat 30.000133394330017, lon 76.00001726173063, sigma 1.01 m |
| 22.5 | TASK_CREATED | UAV-1 | T5 | Casualty check: person #1 reported by UAV -> GROUND_INSPECT priority 0.95, approach (1.9,13.3) |
| 22.5 | TASK_ALLOCATED | UGV-1 | T5 | Casualty check: person #1 reported by UAV -> UGV-1 (utility 0.98, ETA 59 s, energy 1.2%) / UAV-1 infeasible (capability: GROUND_INSPECT requires ugv); UGV-1=0.98 |
| 25.0 | AGENT_MODE | UAV-1 | T1 | INSPECT -> SEARCH |
| 27.5 | OBSERVATION | UAV-1 |  | UAV thermal hazard (fire) at site (-2.0,20.8) -> UGV keep-out r=2.5 m |
| 30.6 | AGENT_MODE | UGV-1 | T2 | EN_ROUTE -> INSPECT_ALIGN |
| 31.2 | AGENT_MODE | UGV-1 | T2 | INSPECT_ALIGN -> INSPECTING |
| 33.1 | TASK_RELEASED |  | T4 | UAV aerial coverage of Z-D = 43% -> task released for allocation |
| 33.1 | TASK_ALLOCATED | UAV-1 | T4 | Thermal/visual check: north-west bay cable drum -> UAV-1 (utility 0.70, ETA 59 s, energy 5.3%) / UAV-1=0.70; UGV-1 infeasible (queue full) |
| 33.1 | TASK_DISPATCHED | UAV-1 | T4 | ASSIGN sent; goal in UAV-1 frame: (-4.6,22.2), (-2.0,24.5) |
| 33.3 | TASK_ACK | UAV-1 | T4 | accepted (1 waypoint(s) inserted before the return leg) |
| 33.6 | TASK_ACTIVE | UAV-1 | T4 | UAV-1 reports ACCEPTED |
| 35.2 | INSPECTION_RESULT | UGV-1 | T2 | UGV ground view: object observed at 2.24 m (expected 2.38 m), evidence T2_t035.png |
| 35.4 | AGENT_MODE | UGV-1 | T2 | INSPECTING -> IDLE |
| 35.4 | TASK_COMPLETED | UGV-1 | T2 | Close inspection: pump-house valve skid completed in 35.4 s since creation |
| 35.4 | NEXT_TASK_DECISION | UGV-1 | T5 | recommended next for UGV-1: T5; candidates [T3:already queued, T5:already queued] |
| 35.4 | TASK_DISPATCHED | UGV-1 | T5 | ASSIGN sent; goal in UGV-1 frame: (-1.5,16.6), (-1.7,18.2) |
| 35.6 | TASK_ACK | UGV-1 | T5 | accepted |
| 36.0 | AGENT_MODE | UGV-1 | T5 | IDLE -> EN_ROUTE |
| 36.0 | TASK_ACTIVE | UGV-1 | T5 | UGV-1 reports EN_ROUTE |
| 42.1 | FAULT_INJECTED | UGV-1 |  | [experiment harness] ugv_link: radio link of UGV-1 forced DOWN (both directions) for 22.0 s; vehicle and simulation keep running |
| 44.0 | FAULT_DETECTED | UGV-1 | T5 | telemetry stale: last message 2.10 s old > timeout 2.0 s -> UGV-1 marked UNAVAILABLE; last known position (3.2,8.4) is NOT treated as current |
| 44.0 | TASK_INTERRUPTED | UGV-1 | T5 | Casualty check: person #1 reported by UAV: ACTIVE -> PENDING (progress kept 17%); stop issuing work to UGV-1 |
| 44.0 | TASK_UNQUEUED | UGV-1 | T3 | Visual check: north-east perimeter gate: ASSIGNED -> PENDING (progress kept 0%); stop issuing work to UGV-1 |
| 44.0 | TASK_REASSIGNED | UAV-1 | T3 | Visual check: north-east perimeter gate -> UAV-1 (utility -0.09, ETA 102 s, energy 9.2%) / UAV-1=-0.09; UGV-1 infeasible (link STALE (no fresh telemetry)) |
| 44.0 | TASK_PENDING |  | T5 | Casualty check: person #1 reported by UAV: no feasible agent -> stays PENDING: UAV-1: capability: GROUND_INSPECT requires ugv; UGV-1: link STALE (no fresh telemetry) |
| 44.0 | TASK_DISPATCHED | UAV-1 | T3 | ASSIGN sent; goal in UAV-1 frame: (6.3,19.0), (8.2,21.2) |
| 44.2 | TASK_ACK | UAV-1 | T3 | accepted (1 waypoint(s) inserted before the return leg) |
| 44.6 | TASK_ACTIVE | UAV-1 | T3 | UAV-1 reports ACCEPTED |
| 50.0 | LINK_LOST | UGV-1 |  | no telemetry for 8.1 s (> 8.0 s) |
| 50.0 | TASK_PENDING |  | T5 | Casualty check: person #1 reported by UAV: no feasible agent -> stays PENDING: UAV-1: capability: GROUND_INSPECT requires ugv; UGV-1: link LOST (no fresh telemetry) |
| 64.1 | FAULT_CLEARED | UGV-1 |  | [experiment harness] ugv_link: radio link of UGV-1 restored |
| 64.8 | AGENT_MODE | UGV-1 | T5 | EN_ROUTE -> TASK_HELD |
| 66.0 | LINK_RECOVERED | UGV-1 |  | 3 consecutive fresh telemetry messages; vehicle reports mode TASK_HELD, task T5 (HELD) -> agent AVAILABLE again |
| 66.0 | TASK_REASSIGNED | UGV-1 | T5 | Casualty check: person #1 reported by UAV -> UGV-1 (utility 1.31, ETA 26 s, energy 0.5%) / UAV-1 infeasible (capability: GROUND_INSPECT requires ugv); UGV-1=1.31 |
| 66.0 | TASK_DISPATCHED | UGV-1 | T5 | RESUME sent; goal in UGV-1 frame: (-1.5,16.6), (-1.7,18.2) |
| 66.1 | TASK_COMPLETED | UAV-1 | T1 | Aerial survey of site completed in 66.1 s since creation |
| 66.1 | NEXT_TASK_DECISION | UAV-1 | T4 | recommended next for UAV-1: T4; candidates [none] |
| 66.2 | TASK_ACK | UGV-1 | T5 | accepted (resumed from HELD) |
| 66.6 | AGENT_MODE | UGV-1 | T5 | TASK_HELD -> EN_ROUTE |
| 66.6 | TASK_ACTIVE | UGV-1 | T5 | UGV-1 reports EN_ROUTE |
| 78.6 | AGENT_MODE | UGV-1 | T5 | EN_ROUTE -> INSPECTING |
| 82.4 | INSPECTION_RESULT | UGV-1 | T5 | UGV ground view: object NOT confirmed at 8.93 m (expected 2.02 m), evidence T5_t082.png |
| 82.8 | AGENT_MODE | UGV-1 | T5 | INSPECTING -> IDLE |
| 82.8 | TASK_COMPLETED | UGV-1 | T5 | Casualty check: person #1 reported by UAV completed in 60.3 s since creation |
| 82.8 | NEXT_TASK_DECISION | UGV-1 |  | recommended next for UGV-1: none (idle / hold); candidates [none] |
| 87.7 | INSPECTION_RESULT | UAV-1 | T4 | UAV aerial visual check at 3.1 m from the target, altitude 2.0 m, image T4_check_t087.png |
| 88.1 | TASK_COMPLETED | UAV-1 | T4 | Thermal/visual check: north-west bay cable drum completed in 88.1 s since creation |
| 88.1 | NEXT_TASK_DECISION | UAV-1 | T3 | recommended next for UAV-1: T3; candidates [none] |
| 113.9 | INSPECTION_RESULT | UAV-1 | T3 | UAV aerial visual check at 3.2 m from the target, altitude 2.0 m, image T3_check_t113.png |
| 114.1 | AGENT_MODE | UAV-1 | T3 | SEARCH -> RETURN |
| 114.1 | TASK_COMPLETED | UAV-1 | T3 | Visual check: north-east perimeter gate completed in 114.1 s since creation |
| 114.1 | NEXT_TASK_DECISION | UAV-1 |  | recommended next for UAV-1: none (idle / hold); candidates [none] |
| 157.6 | AGENT_MODE | UAV-1 | T3 | RETURN -> HOLD_END |
| 163.6 | AGENT_MODE | UAV-1 | T3 | HOLD_END -> LANDING |
| 172.0 | EXPERIMENT_END |  |  | all tasks terminal and UAV landed |
