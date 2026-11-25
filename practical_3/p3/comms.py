"""Base-station message bus (timestamped JSON, latency, per-agent link state)."""
from __future__ import annotations

import json
import time

import numpy as np


def _clean(o):
    if isinstance(o, dict):
        return {k: _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, np.ndarray):
        return [_clean(v) for v in o.tolist()]
    if isinstance(o, (np.floating, float)):
        return round(float(o), 3)
    if isinstance(o, np.integer):
        return int(o)
    return o


class Bus:
    def __init__(self, latency_s, log_path, clock):
        self.latency = latency_s
        self.clock = clock
        self.queue = []
        self.inbox = {"BASE": [], "UAV": [], "UGV": []}
        self.link_up = {"UAV": True, "UGV": True}
        self.seq = 0
        self.dropped = 0
        self.log = open(log_path, "w", encoding="utf-8")

    def send(self, src, dst, msg_type, body):
        self.seq += 1
        msg = {"seq": self.seq, "msg_type": msg_type, "src": src, "dst": dst,
               "stamp": round(self.clock(), 3), "wall_time": time.strftime("%Y-%m-%dT%H:%M:%S"), **_clean(body)}
        agent = src if src != "BASE" else dst
        if not self.link_up.get(agent, True):
            self.dropped += 1
            msg["delivery"] = "DROPPED_LINK_DOWN"
        else:
            self.queue.append((self.clock() + self.latency, msg))
        self.log.write(json.dumps(msg) + "\n")
        return msg

    def pump(self):
        now = self.clock()
        keep = []
        for due, msg in self.queue:
            if due <= now:
                self.inbox[msg["dst"]].append(msg)
            else:
                keep.append((due, msg))
        self.queue = keep

    def take(self, who):
        msgs, self.inbox[who] = self.inbox[who], []
        return msgs

    def close(self):
        self.log.close()
