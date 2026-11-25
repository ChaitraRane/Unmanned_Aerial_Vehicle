"""Simulated radio network between the base station and the vehicles (logical links, no RF propagation model)."""
from __future__ import annotations

import heapq
import json
import random

from coordination_sim.comms.messages import to_dict


class RadioNetwork:
    def __init__(self, cfg, seed, log_path=None):
        self.lat = float(cfg["latency_s"])
        self.jit = float(cfg["jitter_s"])
        self.drop = float(cfg["drop_prob"])
        self.rng = random.Random(seed)
        self.queue = []
        self.n = 0
        self.down = {}
        self.inbox = {}
        self.stats = {"sent": 0, "delivered": 0, "dropped_random": 0, "dropped_link_down": 0}
        self.log = open(log_path, "w", buffering=1) if log_path else None

    def set_link_down(self, agent_id, down: bool):
        self.down[agent_id] = bool(down)

    def link_down(self, agent_id):
        return self.down.get(agent_id, False)

    def send(self, t, src, dst, msg):
        """src/dst: 'BASE' or an agent id."""
        self.stats["sent"] += 1
        vehicle = dst if src == "BASE" else src
        rec = {"t_send": round(t, 3), "src": src, "dst": dst, "kind": msg.kind}
        if self.link_down(vehicle):
            self.stats["dropped_link_down"] += 1
            rec["fate"] = "DROPPED_LINK_DOWN"
        elif self.rng.random() < self.drop:
            self.stats["dropped_random"] += 1
            rec["fate"] = "DROPPED_RANDOM"
        else:
            d = max(0.0, self.lat + self.rng.uniform(-self.jit, self.jit))
            self.n += 1
            heapq.heappush(self.queue, (t + d, self.n, dst, msg))
            rec["fate"] = "QUEUED"
            rec["t_deliver"] = round(t + d, 3)
        if self.log and msg.kind not in ("HEARTBEAT",):
            body = to_dict(msg)
            if body.get("obs_type") == "MAP_SUMMARY":
                body = {k: v for k, v in body.items() if k != "data"}
            rec["msg"] = body
            self.log.write(json.dumps(rec, default=float) + "\n")
        return rec["fate"]

    def step(self, t):
        while self.queue and self.queue[0][0] <= t:
            td, _, dst, msg = heapq.heappop(self.queue)
            vehicle = msg.agent_id if dst == "BASE" else dst
            if self.link_down(vehicle):
                self.stats["dropped_link_down"] += 1
                continue
            self.inbox.setdefault(dst, []).append(msg)
            self.stats["delivered"] += 1

    def receive(self, dst):
        msgs = self.inbox.get(dst, [])
        self.inbox[dst] = []
        return msgs

    def close(self):
        if self.log:
            self.log.close()
