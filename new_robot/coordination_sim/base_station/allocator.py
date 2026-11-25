"""AI-assisted task allocation: a transparent, RULE-BASED / HEURISTIC utility model (not trained, no neural network)."""
from __future__ import annotations

import math


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class Allocator:
    def __init__(self, cfg_alloc, cfg_agents):
        self.w = cfg_alloc
        self.acfg = cfg_agents

    def evaluate(self, task, agent, queued_s=0.0, incumbent=None):
        """Returns dict(feasible, utility, terms, reason, eta_s, energy_pct)."""
        a = self.acfg[agent.vtype]
        out = {"agent": agent.agent_id, "feasible": False, "utility": None, "terms": {}, "reason": ""}
        if agent.vtype not in task.capabilities:
            out["reason"] = f"capability: {task.type} requires {'/'.join(task.capabilities)}"
            return out
        if agent.link != "HEALTHY":
            out["reason"] = "no telemetry yet" if agent.last is None else f"link {agent.link} (no fresh telemetry)"
            return out
        if agent.health == "FAULT" or agent.failsafe:
            out["reason"] = f"vehicle fail-safe/fault active ({agent.failsafe or agent.health})"
            return out
        if not agent.loc_valid or agent.loc_conf < self.w["min_localisation_conf"]:
            out["reason"] = f"localisation not valid (conf {agent.loc_conf:.2f})"
            return out
        if not agent.available_for(task):
            out["reason"] = f"not available ({agent.mode})"
            return out
        if len(agent.queue) >= a["queue_capacity"]:
            out["reason"] = "queue full"
            return out
        if agent.pos_site is None:
            out["reason"] = "no position estimate"
            return out
        start = agent.queue_end_xy() or agent.pos_site
        goal = task.first_goal_xy()
        dist = _dist(start, goal) + task.extra_path_m()
        eta = a["path_factor"] * dist / a["cruise_mps"] + queued_s + task.dwell_s()
        energy = eta * a["energy_pct_per_s"]
        margin = agent.battery - a["battery_reserve_pct"]
        if agent.battery < 0:
            out["reason"] = "battery unknown"
            return out
        if energy > margin:
            out["reason"] = f"battery: needs {energy:.1f}% but only {margin:.1f}% above reserve"
            return out
        w = self.w
        pref = 0.0 if not task.preferred else (1.0 if task.preferred == agent.vtype else -0.5)
        rel = (agent.completed + 1.0) / (agent.completed + agent.failed + 2.0)
        terms = {
            "priority": w["w_priority"] * task.priority,
            "eta": -w["w_eta"] * eta / w["eta_ref_s"],
            "energy": -w["w_energy"] * energy / max(margin, 1e-3),
            "localisation": w["w_localisation"] * agent.loc_conf,
            "reliability": w["w_reliability"] * rel,
            "preference": w["w_preference"] * pref,
        }
        if incumbent == agent.agent_id:
            terms["commitment"] = 0.10
        out.update(feasible=True, utility=round(sum(terms.values()), 4), terms={k: round(v, 4) for k, v in terms.items()},
                   eta_s=round(eta, 1), energy_pct=round(energy, 2))
        return out

    def allocate(self, tasks, agents, queued_s_fn, incumbents=None):
        """tasks: unassigned READY tasks; agents: dict id -> AgentRecord. Returns (assignments, decisions)."""
        assignments, decisions = [], []
        pending = sorted(tasks, key=lambda t_: -t_.priority)
        evals_by_task = {}
        while pending:
            best = None
            for task in pending:
                evs = [self.evaluate(task, ag, queued_s_fn(ag), (incumbents or {}).get(task.task_id)) for ag in agents.values()]
                evals_by_task[task.task_id] = evs
                for ev in evs:
                    if ev["feasible"] and (best is None or (ev["utility"], task.priority) > (best[1]["utility"], best[0].priority)):
                        best = (task, ev)
            if best is None:
                break
            task, ev = best
            agents[ev["agent"]].reserve(task)
            assignments.append((task, ev["agent"], ev))
            decisions.append({"task": task.task_id, "chosen": ev["agent"], "utility": ev["utility"],
                              "candidates": evals_by_task[task.task_id]})
            pending.remove(task)
        for task in pending:
            decisions.append({"task": task.task_id, "chosen": None, "candidates": evals_by_task.get(task.task_id, []),
                              "reason": "; ".join(f"{e['agent']}: {e['reason']}" for e in evals_by_task.get(task.task_id, []))})
        return assignments, decisions
