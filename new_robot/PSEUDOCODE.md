# Pseudocode — UAV–UGV Coordination with an AI-Assisted Base Station

## 1. Shared simulation loop

```
INITIALISE one PyBullet world (500 Hz) with site, base station, UAV, UGV
START MAVLink link: UAV companion  <->  simulated flight controller (UDP)
REGISTER both agents with the base station

FOR every physics step t:
    UAV.flight_controller_and_motors(t)
    UGV.apply_wheel_speeds()
    world.step()
    UGV.estimator.propagate(IMU, wheel encoders)
    radio.deliver_messages(t)

    EVERY 0.2 s:  UAV.on_camera_frame(RGB-D, thermal)          # SLAM + perception + map
    EVERY 0.2 s:  UGV.on_camera_frame(RGB-D)                   # VO + VIWO + occupancy map
                  UGV.wheel_speeds <- UGV.control(t)
                  UGV.send_telemetry_and_observations()
    EVERY 0.1 s:  UAV.mission_step(t) -> MAVLink velocity setpoint
                  UAV.send_telemetry_and_observations()
    EVERY 0.1 s:  FAULT_INJECTOR.step(t)
                  BASE_STATION.step(t)
    RECORD dashboard, camera feeds and maps
UNTIL all tasks terminal AND UAV landed
```

## 2. Common message format

```
TELEMETRY   = {agent_id, vehicle_type, stamp, frame, position, heading,
               loc_valid, loc_confidence, battery, health, mode,
               comm_ok, task_id, task_state, task_progress, failsafe}
OBSERVATION = {agent_id, stamp, type ∈ {SURVIVOR, HAZARD, INSPECTION_RESULT, MAP_SUMMARY}, frame, position, data}
TASK_COMMAND = {agent_id, command ∈ {ASSIGN, RESUME, CANCEL, KEEPOUT}, task_id, type, priority, goal (vehicle frame)}
HEARTBEAT   = {stamp}                       # base station -> vehicles, 2 Hz
```

## 3. Frame registration

```
site_from_uav(x, y)   = (x + pad_uav.x, y + pad_uav.y)              # SLAM map is ENU, origin at take-off pad
site_from_ugv(X, Z)   = cam0 + X * right(yaw0) + Z * forward(yaw0)   # Stage-8 map = first camera frame
ugv_from_site(p)      = ((p - cam0)·right, (p - cam0)·forward)
```

## 4. Base station

```
PROCEDURE BASE_STATION.step(t):
    FOR each message m received:
        IF m is TELEMETRY:
            IF m.stamp <= agent.last_stamp: DISCARD            # never let old data overwrite new
            agent.state <- m ; agent.pos_site <- transform(m.position)
            update coverage grid from agent pose
            update task progress / completion from m
        IF m is OBSERVATION:
            SURVIVOR -> IF no task within 3 m: CREATE GROUND_INSPECT task (priority 0.95)
            HAZARD   -> add keep-out zone, send KEEPOUT to UGV
            INSPECTION_RESULT -> attach evidence to task

    SEND HEARTBEAT to every agent (2 Hz)
    MONITOR_LINKS(t)
    RELEASE tasks whose zone is aerially covered >= 35 %
    ALLOCATE()            # every 2 s or when something changed
    DISPATCH()
```

```
PROCEDURE MONITOR_LINKS(t):
    FOR each agent a:
        age <- t - a.last_stamp
        IF age > lost_timeout:      a.link <- LOST
        ELSE IF age > stale_timeout: a.link <- STALE
        ELSE IF a.link was STALE/LOST AND fresh_messages < 3: keep state
        ELSE a.link <- HEALTHY
        IF a.link changed HEALTHY -> STALE:
            LOG FAULT_DETECTED
            FOR each task k held by a:
                k.state <- PENDING (progress kept), k.owner <- none
            mark a UNAVAILABLE (no new tasks)
        IF a.link changed STALE/LOST -> HEALTHY:
            LOG LINK_RECOVERED, re-run allocation
```

```
PROCEDURE DISPATCH():
    FOR each HEALTHY agent a:
        FOR next undispatched task k in a.queue:
            IF a is UGV AND head task still running: STOP
            goal <- transform k.goal into a's frame
            SEND (RESUME if a reports k as HELD else ASSIGN)
```

## 5. AI-assisted allocator (rule-based utility)

```
FUNCTION FEASIBLE(task, agent):
    RETURN agent.type ∈ task.capabilities
       AND agent.link = HEALTHY
       AND no fail-safe active AND health ≠ FAULT
       AND agent.loc_valid AND agent.loc_conf >= 0.4
       AND agent.battery - ENERGY(task, agent) >= agent.reserve
       AND |agent.queue| < agent.capacity

FUNCTION UTILITY(task, agent):
    eta     <- path_factor * distance(queue_end(agent), task.goal) / cruise + queued_time + dwell
    energy  <- eta * energy_rate(agent)
    RETURN  1.0 * task.priority
          - 0.5 * eta / 60
          - 0.8 * energy / (battery - reserve)
          + 0.4 * loc_conf
          + 0.3 * (completed + 1) / (completed + failed + 2)
          + 0.35 * preference(task, agent)          # +1 preferred, -0.5 other
          + 0.10 IF agent already holds task         # commitment

PROCEDURE ALLOCATE():
    pool <- READY tasks ∪ queued-but-undispatched tasks          # re-auction
    WHILE pool not empty:
        (k, a) <- argmax UTILITY(k, a) over FEASIBLE pairs
        IF none: BREAK
        a.queue.append(k) ; pool.remove(k) ; LOG decision with all candidate terms
    FOR each k left in pool:
        k.state <- PENDING ; LOG reason per agent           # e.g. "UAV: GROUND_INSPECT requires ugv"
```

```
PROCEDURE ON_TASK_COMPLETED(k, a):
    k.state <- COMPLETED ; a.completed += 1
    ranked <- sort feasible remaining tasks by UTILITY(·, a)
    LOG NEXT_TASK_DECISION(a, head of queue OR ranked[0])
```

## 6. UAV agent (existing autonomy + coordination)

```
ON camera frame:
    pose <- SLAM(RGB-D, IMU)                         # ORB-SLAM3 / orb_pnp_vo
    IF tracking OK: map, survivors, hazards <- PERCEPTION_MAPPING(RGB-D, thermal, pose)

ON TASK_COMMAND(ASSIGN k):
    wps <- k.waypoints (survey) OR [k.approach] (visual check)
    IF no mission OR mission DONE (landed): start new sortie(wps + RETURN)
    ELSE IF mission LANDING/HOLD_END: reject
    ELSE insert wps before RETURN leg
    SEND TASK_ACK

EVERY 0.1 s:
    armed, mode, battery, link_age <- MAVLink telemetry
    step <- MissionController(pose, map, SLAM state, armed, mode, link)     # A* + VFH+ + fail-safes
    SEND MAVLink ARM / OFFBOARD / LAND actions
    SEND MAVLink SET_POSITION_TARGET_LOCAL_NED(velocity)
    task progress <- reached waypoints / task waypoints
```

## 7. UGV agent (existing autonomy + coordination)

```
ON camera frame:
    vo_pose <- VisualOdometry(RGB-D)
    IF vo_pose valid: VIWO.update_visual(vo_pose)
    pose <- VIWO.pose()
    ground <- GroundPlaneEstimator(depth)
    IF VIWO.may_write_map(): OccupancyMap.integrate(depth, pose, ground)

PROCEDURE CONTROL(t):
    IF no HEARTBEAT for 2.5 s:  task.state <- HELD ; RETURN STOP      # COMM_HOLD
    IF VIWO confidence CRITICAL: RETURN STOP                          # LOC_LOST_HOLD
    IF task HELD:                RETURN STOP until RESUME received
    IF no task:                  RETURN STOP

    IF EN_ROUTE:
        IF |goal - pose| < 0.45 m: state <- INSPECTING
        EVERY 2 s: path <- A*(OccupancyMap, pose, goal, keep-outs)
        IF no path: rotate toward goal to map it; FAIL after 45 s
        carrot <- point 0.65 m ahead on path
        analysis <- LocalPlanner.analyse(depth, carrot bearing)
        IF analysis.emergency: STOP / back off
        gap <- LocalPlanner.select_gap(analysis)
        RETURN arc_wheel_speeds(gap.angle, gap.depth, speed)

    IF INSPECTING:
        rotate to face target ; dwell 4 s
        range <- median depth in image centre
        save evidence image ; SEND INSPECTION_RESULT ; state <- COMPLETED
```

## 8. Fault injection and expected safe response

```
FAULT ugv_link:
    WHEN UGV has completed >= 1 task AND active task progress >= 25 %:
        radio.link(UGV) <- DOWN (both directions) for 22 s

EXPECTED:
    base station : telemetry age > 2 s -> FAULT_DETECTED, UGV UNAVAILABLE
                   ground-only task   -> PENDING (never given to UAV)
                   both-capable task  -> REASSIGNED to UAV
    UGV          : no heartbeat 2.5 s -> COMM_HOLD, wheels stopped, task HELD
    recovery     : 3 fresh messages -> LINK_RECOVERED -> RESUME held task -> COMPLETED
```
