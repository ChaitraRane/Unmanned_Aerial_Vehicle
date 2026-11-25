# Practical 3 – Collaborative Task Planning: Structured Pseudocode

The pseudocode mirrors the implementation in `practical_3/p3/` (file references in brackets).

## 1. Data structures

```
FRAME mission_enu      : origin = base station (0,0,0), X = East, Y = North, Z = Up
FRAME uav_local_ned    : UAV flight-controller EKF frame (North, East, Down)
FRAME ugv_odom         : UGV odometry frame, origin = dock, x = docked heading

TASK   { task_id, target, action, requires, priority, agent, status, progress, target_mission, results, history }
STATUS ∈ { PENDING, ASSIGNED, ACCEPTED, IN_PROGRESS, COMPLETED, FAILED, HOLD }

MESSAGE header { seq, msg_type, src, dst, stamp (sim time), wall_time, frame_id }
  REGISTER / STATUS   { agent_id, pose, frame_id, battery_pct, health, localisation, current_task, comm }
  TASK_ASSIGNMENT     { task_id, target, assign_to, priority, action, target_xyz, frame_id, status = ASSIGNED,
                        [handover: target_xyz_ugv_odom, ugv_frame_id, observations, requires_ack] }
  TASK_ACK            { task_id, agent_id, status = ACCEPTED | REJECTED, goal_frame, goal_xyz }
  TASK_PROGRESS       { task_id, agent_id, status = IN_PROGRESS, progress, pose, frame_id }
  TARGET_REPORT       { marker, xyz, frame_id = uav_local_ned, sigma_m, n_obs, image }
  TASK_COMPLETE       { task_id, agent_id, status, results { images, sensor_data, inspection_performed } }
```

## 2. Shared reference frame  [frames.py]

```
PROCEDURE REGISTER_UAV(pad_mission_xyz, p0_ned):
    R ← R_ENU←NED                                  // swap N/E, negate D
    t ← pad_mission_xyz − R · p0_ned               // p0_ned = FC position reported on the surveyed pad
    RETURN T(mission_enu ← uav_local_ned) = (R, t)

PROCEDURE REGISTER_UGV(dock_xy, dock_heading):
    RETURN T(mission_enu ← ugv_odom) = (Rz(dock_heading), [dock_xy, 0])

PROCEDURE CONVERT(p, src, dst):
    RETURN T(dst ← mission_enu) · T(mission_enu ← src) · p
```

## 3. Base station / AI coordinator  [coordinator.py]

```
LOOP every 0.1 s:
    FOR msg IN inbox(BASE): HANDLE(msg)
    WATCH_LINKS()
    PLAN()

PROCEDURE AVAILABLE(agent):
    RETURN registered ∧ (now − last_seen ≤ timeout) ∧ health = OK ∧ battery ≥ 25 % ∧ localisation = VALID

PROCEDURE PLAN():
    CASE stage OF
    INIT:        IF UAV and UGV registered ∧ AVAILABLE(both):
                     ASSIGN(T1, UAV); stage ← SURVEY
    SURVEY:      IF T1 = COMPLETED:
                     mark T2..T5 without an identified target as FAILED
                     IF AVAILABLE(UAV) ∧ CAPABLE(UAV, T2): ASSIGN(T2, UAV)
                     stage ← UAV_INSPECT
    UAV_INSPECT: IF T2 ∈ {COMPLETED, FAILED}:                        // Condition B
                     send UAV COMMAND RETURN_AND_STANDBY; stage ← GROUND
    GROUND:      IF UGV has no active task:
                     next ← pending ground task with lowest (priority, distance to UGV)
                     IF next = NONE: start T6; send FINAL_STATUS_REQUEST to both; stage ← VERIFY
                     ELSE IF AVAILABLE(UGV) ∧ CAPABLE(UGV, next): HANDOVER(next)   // Condition A / C
    VERIFY:      IF both final replies received (or 15 s elapsed): FINISH()

PROCEDURE HANDLE(TARGET_REPORT m):                                // from UAV during T1
    p_mission ← CONVERT(m.xyz, m.frame_id, mission_enu)
    valid ← finite(p_mission) ∧ m.sigma ≤ 1.5 m ∧ m.n_obs ≥ 3 ∧ inside operational boundary
    LOG (m.frame_id, m.xyz) → (mission_enu, p_mission)
    IF valid: CLASSIFY(target)

PROCEDURE CLASSIFY(target):
    IF target.z ≥ 2.0 m THEN category ← AERIAL/ELEVATED; agent ← UAV
    ELSE                      category ← GROUND;          agent ← UGV
    task ← task whose action matches the target's marker class
    task.target_mission ← p_mission

PROCEDURE HANDOVER(task):                                         // UAV → base → UGV
    p_odom ← CONVERT(task.target_mission, mission_enu, ugv_odom)
    LOG uav_local_ned xyz → mission_enu xyz → ugv_odom xyz
    SEND TASK_ASSIGNMENT(task, target_xyz = task.target_mission, target_xyz_ugv_odom = p_odom,
                         priority, observations + UAV image, requires_ack = TRUE) TO UGV
    task.status ← ASSIGNED                                        // ACCEPTED only after the UGV's TASK_ACK

PROCEDURE HANDLE(TASK_ACK m):
    IF m.status = ACCEPTED THEN task.status ← ACCEPTED ELSE task.status ← FAILED

PROCEDURE HANDLE(TASK_PROGRESS m):
    task.progress ← m.progress; IF task.status ∈ {ASSIGNED, ACCEPTED}: task.status ← IN_PROGRESS

PROCEDURE HANDLE(TASK_COMPLETE m):
    IF m.status = COMPLETED ∧ m.results.inspection_performed: task.status ← COMPLETED
    ELSE task.status ← FAILED                                    // a waypoint alone never completes a task

PROCEDURE WATCH_LINKS():                                          // Condition D
    FOR agent WITH now − last_seen > 3 s:
        FOR task OF agent IN {ASSIGNED, ACCEPTED, IN_PROGRESS}:
            task.status ← HOLD
            alt ← other agent
            IF CAPABLE(alt, task) ∧ AVAILABLE(alt): ASSIGN(task, alt)
            ELSE keep HOLD for the original agent
    ON next STATUS from that agent: link restored → HOLD tasks → IN_PROGRESS

PROCEDURE FINISH():
    ok ← T1..T5 all COMPLETED
    T6 ← COMPLETED if ok and both agents replied, else FAILED (incomplete tasks listed)
    SEND END_MISSION_SAFE to UAV and UGV; write mission report
```

## 4. UAV agent  [uav_agent.py]  (commands only through MAVLink)

```
ON first telemetry: REGISTER_UAV(pad, LOCAL_POSITION_NED); SEND REGISTER
EVERY 1 s: SEND STATUS (+ TASK_PROGRESS while a task is active)

ON TASK_ASSIGNMENT T1:
    SEND TASK_ACK(ACCEPTED)
    waypoints ← lawnmower lanes over the inspection area at 5 m
    stream zero-velocity setpoints → MAV_CMD_COMPONENT_ARM_DISARM → MAV_CMD_DO_SET_MODE(OFFBOARD)
    climb, then FOR wp IN waypoints: SET_POSITION_TARGET_LOCAL_NED (velocity, yaw toward travel)
    EVERY camera frame:
        FOR colour marker class:
            mask ← HSV segmentation; d ← median depth over the mask
            p_cam ← back-project(u, v, d, K)
            p_ned ← p_local + R(roll, pitch, yaw) · (T_body_cam · p_cam)
            track.add(p_ned)
            IF n_obs ≥ 5 ∧ σ < 1.5 m ∧ not reported: SEND TARGET_REPORT(p_ned, uav_local_ned, σ, n_obs, image)
    SEND TASK_COMPLETE(T1, ROIs, observations)

ON TASK_ASSIGNMENT T2:
    fly to standoff 5 m south of P1, yaw toward P1, hold 6 s, record images
    performed ← marker visible in ≥ 5 frames
    SEND TASK_COMPLETE(T2, images, marker statistics, refined target position)

ON COMMAND RETURN_AND_STANDBY: fly to pad → MAV_CMD_NAV_LAND → FC auto-disarm → STANDBY_LANDED
```

## 5. UGV agent  [ugv_agent.py]

```
EVERY 20 ms: odometry
    IF stationary: gyro_bias ← low-pass(gyro)                    // zero-velocity bias calibration
    ELSE yaw += (gyro − gyro_bias)·dt; x,y += r·(ωL+ωR)/2 · [cos yaw, sin yaw]·dt
EVERY camera frame: depth → points → log-odds occupancy grid (ugv_odom); front-obstacle count
EVERY 1 s: SEND STATUS (+ TASK_PROGRESS)
IF no base heartbeat for 3 s: COMM_LOSS_HOLD (stop); resume the same task when the link returns

ON TASK_ASSIGNMENT:
    check ← CONVERT(target_xyz, mission_enu, ugv_odom)
    valid ← finite ∧ |check − target_xyz_ugv_odom| < 5 cm ∧ frame_id = ugv_odom
    goal ← standoff point 1.6 m before the target
    path ← A*(occupancy grid, pose, goal)                        // planning/astar.py
    IF valid ∧ path found: SEND TASK_ACK(ACCEPTED); state ← NAVIGATING
    ELSE SEND TASK_ACK(REJECTED)                                 // the UGV does not move

NAVIGATING:  re-plan A* at 1 Hz; pure-pursuit on the path; IF obstacle < 0.75 m ahead: stop + re-plan,
             reverse-and-turn if blocked > 4 s; on reaching the goal → INSPECTING
INSPECTING:  turn to face the target, collect for 4 s:
    T3 close_up_inspection     → images, marker detections, target re-measured in ugv_odom → mission_enu
    T4 environmental_sampling  → SIMULATED environmental sensor (temperature, humidity, CO2) averaged
    T5 unexplored_region       → images + 360° scan, count newly mapped cells
    performed ← marker observed in ≥ 3 frames
    SEND TASK_COMPLETE(results incl. cross-platform residual |p_UGV − p_UAV| in mission_enu)

ON COMMAND END_MISSION_SAFE: motors stopped → PARKED
```

## 6. Mission sequence

```
Stage 1  operator start → REGISTER(UAV), REGISTER(UGV) → availability check
Stage 2  T1 → UAV aerial survey → TARGET_REPORTs → transforms to mission_enu
Stage 3  coordinator classifies each target: elevated → UAV, ground → UGV
Stage 4  T2 → UAV elevated inspection → valid TASK_COMPLETE → COMPLETED
Stage 5  handover: UAV target → mission_enu → ugv_odom → TASK_ASSIGNMENT → UGV TASK_ACK
Stage 6  T3, T4, T5 → UGV navigate (A*) → inspect → TASK_COMPLETE → next task recommended
         (fault: UGV link timeout during T4 → HOLD → reassignment rejected (UAV lacks env_sensor) → resume)
Stage 7  T6 → final status from both → verification → report → UAV landed/disarmed, UGV parked
```
