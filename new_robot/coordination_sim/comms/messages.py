"""Common coordination message format (vehicle <-> base station).  JSON-serialisable dataclasses."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import List, Optional


@dataclass
class Telemetry:
    kind: str = "TELEMETRY"
    agent_id: str = ""
    vehicle_type: str = ""
    stamp: float = 0.0
    seq: int = 0
    frame: str = ""
    position: tuple = (0.0, 0.0, 0.0)
    heading_rad: float = 0.0
    loc_valid: bool = False
    loc_confidence: float = 0.0
    loc_detail: str = ""
    battery_pct: float = -1.0
    health: str = "OK"
    mode: str = ""
    comm_ok: bool = True
    task_id: Optional[str] = None
    task_state: str = "IDLE"
    task_progress: float = 0.0
    failsafe: str = ""
    extra: dict = field(default_factory=dict)


@dataclass
class Observation:
    kind: str = "OBSERVATION"
    agent_id: str = ""
    stamp: float = 0.0
    seq: int = 0
    obs_type: str = ""
    frame: str = ""
    position: Optional[tuple] = None
    data: dict = field(default_factory=dict)


@dataclass
class TaskCommand:
    kind: str = "TASK_COMMAND"
    agent_id: str = ""
    stamp: float = 0.0
    seq: int = 0
    command: str = ""
    task_id: str = ""
    task_type: str = ""
    priority: float = 0.0
    goal_frame: str = ""
    goal: Optional[List[tuple]] = None
    params: dict = field(default_factory=dict)


@dataclass
class TaskAck:
    kind: str = "TASK_ACK"
    agent_id: str = ""
    stamp: float = 0.0
    seq: int = 0
    task_id: str = ""
    accepted: bool = True
    reason: str = ""


@dataclass
class Heartbeat:
    kind: str = "HEARTBEAT"
    agent_id: str = "BASE"
    stamp: float = 0.0
    seq: int = 0


@dataclass
class Register:
    kind: str = "REGISTER"
    agent_id: str = ""
    vehicle_type: str = ""
    stamp: float = 0.0
    seq: int = 0
    capabilities: List[str] = field(default_factory=list)
    frame: str = ""
    sensors: List[str] = field(default_factory=list)
    software: dict = field(default_factory=dict)


def to_dict(msg) -> dict:
    return asdict(msg)
