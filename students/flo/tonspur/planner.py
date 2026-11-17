from __future__ import annotations

import itertools
import math
import time

from geometry import Cylinder

EPS = 1e-6
MIN_SPEED = 0.001
_ids = itertools.count(1)


class JobError(Exception):
    pass


def make_job(data: dict, cfg: dict) -> dict:
    paths = data.get("paths")
    if not isinstance(paths, list) or not paths:
        raise JobError("job has no paths")
    if len(paths) > 5000:
        raise JobError("job has more than 5000 paths")
    clean, total = [], 0
    for p in paths:
        pts = p.get("pts") or []
        if not pts:
            continue
        pts = [[float(u), float(v)] for u, v in pts]
        total += len(pts)
        q = {"pts": pts, "dark": min(1.0, max(0.0, float(p.get("dark", 0.5)))),
             "dwell": min(cfg["dot_dwell_max_s"] * 8, max(0.0, float(p.get("dwell", 0.0))))}
        if p.get("speed") is not None:
            q["speed"] = float(p["speed"])
        clean.append(q)
    if not clean:
        raise JobError("job has no points")
    if total > 100000:
        raise JobError("job has more than 100000 points")
    return {"id": next(_ids), "name": str(data.get("name", "Muster"))[:80],
            "mode": str(data.get("mode", ""))[:20], "mic": data.get("mic"),
            "trigger": float(data.get("trigger") or time.time()),
            "paths": clean, "state": "wartet"}


def path_speed(cfg: dict, p: dict, vmax: float) -> float:
    if "speed" in p:
        v = p["speed"]
    else:
        light, dark = cfg["draw_speed_light_mps"], cfg["draw_speed_dark_mps"]
        v = light - p["dark"] * (light - dark)
    return min(max(v, MIN_SPEED), vmax)


def densify(pts: list, max_step_deg: float) -> list:
    out = [pts[0]]
    for (a0, h0), (a1, h1) in zip(pts, pts[1:]):
        n = max(1, math.ceil(abs(a1 - a0) / max_step_deg - EPS))
        for k in range(1, n + 1):
            t = k / n
            out.append([a0 + (a1 - a0) * t, h0 + (h1 - h0) * t])
    return out


def plan_job(cyl: Cylinder, cfg: dict, job: dict, a0: float, vmax: float) -> list:
    a1 = a0 + cyl.job_arc
    span = cyl.h_max - cyl.h_min
    planned = []
    for p in job["paths"]:
        pts = []
        for u, v in p["pts"]:
            a = cyl.clamp_a(a0 + min(max(u, 0.0), 1.0) * cyl.job_arc, a0, a1)
            h = cyl.clamp_h(cyl.h_min + min(max(v, 0.0), 1.0) * span)
            pts.append([a, h])
        pts = densify(pts, cfg["max_step_deg"])
        pts = [[cyl.clamp_a(a, a0, a1), cyl.clamp_h(h)] for a, h in pts]
        planned.append({"pts": pts, "speed": path_speed(cfg, p, vmax), "dwell": p["dwell"]})
    return planned


class Lap:
    def __init__(self, cyl: Cylinder):
        self.cyl = cyl
        self.lap = 1
        self.cursor = 0.0

    def place(self, lap=None, cursor=None):
        lap = self.lap if lap is None else lap
        cursor = self.cursor if cursor is None else cursor
        if cursor + self.cyl.job_arc > 360.0 + EPS:
            return lap + 1, 0.0, True
        return lap, cursor, False

    def commit(self, lap: int, a0: float) -> None:
        self.lap = lap
        self.cursor = min(360.0, a0 + self.cyl.job_arc)


class JobQueue:
    def __init__(self, cfg: dict):
        self.max = cfg["queue_max"]
        self.policy = cfg["queue_full_policy"]
        self.items: list = []
        self.dropped: list = []

    def add(self, job: dict):
        gone = None
        if len(self.items) >= self.max:
            if self.policy == "reject":
                raise JobError(f"Warteschlange voll ({self.max}), Auftrag abgelehnt")
            if self.policy == "drop_newest":
                self.dropped.append(job)
                return job
            gone = self.items.pop(0)
            self.dropped.append(gone)
        self.items.append(job)
        self.items.sort(key=lambda j: (j["trigger"], j["id"]))
        return gone

    def pop(self):
        return self.items.pop(0) if self.items else None

    def clear(self) -> list:
        gone, self.items = self.items, []
        return gone
