from __future__ import annotations

import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
CONFIG_FILE = HERE / "config.json"
POLICIES = ("drop_oldest", "drop_newest", "reject")
LAMP_CONTROLS = ("dmx", "always_on")


class ConfigError(Exception):
    pass


def load_config(path=None) -> dict:
    cfg = json.loads(Path(path or CONFIG_FILE).read_text())
    if cfg["direction"] not in (1, -1):
        raise ConfigError("direction must be 1 or -1")
    if not cfg["height_min_m"] < cfg["height_max_m"]:
        raise ConfigError("height_min_m must be below height_max_m")
    if not 0 < cfg["job_arc_deg"] <= 360:
        raise ConfigError("job_arc_deg must be in (0, 360]")
    if not 0 < cfg["retract_radius_m"] <= cfg["lift_radius_m"] < cfg["cloth_radius_m"] - cfg["lamp_distance_m"]:
        raise ConfigError("need 0 < retract_radius_m <= lift_radius_m < drawing radius")
    if cfg["queue_full_policy"] not in POLICIES:
        raise ConfigError(f"queue_full_policy must be one of {POLICIES}")
    cfg.setdefault("dot_extra_s", 0.0)
    if cfg["dot_extra_s"] < 0:
        raise ConfigError("dot_extra_s must be >= 0")
    cfg.setdefault("lamp_control", "always_on")
    if cfg["lamp_control"] not in LAMP_CONTROLS:
        raise ConfigError(f"lamp_control must be one of {LAMP_CONTROLS}")
    if cfg["lamp_control"] == "dmx":
        if not 1 <= cfg["dmx_channel"] <= 512:
            raise ConfigError("dmx_channel must be 1…512")
        if not (0 <= cfg["dmx_on_value"] <= 255 and 0 <= cfg["dmx_off_value"] <= 255):
            raise ConfigError("dmx_on_value and dmx_off_value must be 0…255")
        if cfg["lamp_switch_timeout_s"] <= 0 or cfg["lamp_settle_s"] < 0:
            raise ConfigError("lamp_switch_timeout_s must be > 0 and lamp_settle_s >= 0")
    if cfg["queue_max"] < 1:
        raise ConfigError("queue_max must be at least 1")
    return cfg


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matrix_to_rotvec(m) -> list:
    tr = m[0][0] + m[1][1] + m[2][2]
    ang = math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0)))
    if ang < 1e-9:
        return [0.0, 0.0, 0.0]
    if math.pi - ang > 1e-6:
        s = 2.0 * math.sin(ang)
        ax = [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s]
    else:
        d = [max(0.0, (m[i][i] + 1.0) / 2.0) for i in range(3)]
        i = max(range(3), key=lambda k: d[k])
        ax = [0.0, 0.0, 0.0]
        ax[i] = math.sqrt(d[i])
        for j in range(3):
            if j != i:
                ax[j] = (m[i][j] + m[j][i]) / (4.0 * ax[i])
    n = math.sqrt(sum(v * v for v in ax))
    return [ang * v / n for v in ax]


def rotvec_to_matrix(rv) -> list:
    ang = math.sqrt(sum(v * v for v in rv))
    if ang < 1e-12:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    x, y, z = (v / ang for v in rv)
    c, s, t = math.cos(ang), math.sin(ang), 1.0 - math.cos(ang)
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c]]


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class Cylinder:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.R = cfg["cloth_radius_m"]
        self.r_draw = self.R - cfg["lamp_distance_m"]
        self.r_ret = cfg["retract_radius_m"]
        self.r_lift = cfg["lift_radius_m"]
        self.h_min = cfg["height_min_m"]
        self.h_max = cfg["height_max_m"]
        self.start = cfg["start_angle_deg"]
        self.dir = cfg["direction"]
        self.job_arc = cfg["job_arc_deg"]
        self.switched = cfg["lamp_control"] == "dmx"
        self.tilt = bool(cfg["tilt_down_on_travel"]) and not self.switched
        self.h_travel_max = cfg["travel_height_max_m"]
        self.roll = float(cfg.get("lamp_roll_deg") or 0.0)

    def phi(self, a_deg: float) -> float:
        return math.radians(self.start + self.dir * a_deg)

    def clamp_a(self, a: float, lo: float = 0.0, hi: float = 360.0) -> float:
        return min(max(a, max(lo, 0.0)), min(hi, 360.0))

    def clamp_h(self, h: float) -> float:
        return min(max(h, self.h_min), self.h_max)

    def arc_m(self, deg: float) -> float:
        return self.R * math.radians(deg)

    def travel_h(self, h: float) -> float:
        return min(h, self.h_travel_max) if self.tilt else h

    def orientation(self, phi: float, facing: str, roll: float | None = None) -> list:
        c, s = math.cos(phi), math.sin(phi)
        if facing == "out":
            x, y, z = (0.0, 0.0, -1.0), (-s, c, 0.0), (c, s, 0.0)
        else:
            x, y, z = (-c, -s, 0.0), (-s, c, 0.0), (0.0, 0.0, -1.0)
        r = math.radians(self.roll if roll is None else roll)
        cr, sr = math.cos(r), math.sin(r)
        x, y = ([x[i] * cr + y[i] * sr for i in range(3)], [-x[i] * sr + y[i] * cr for i in range(3)])
        cols = [x, y, z]
        return [[cols[j][i] for j in range(3)] for i in range(3)]

    def pose(self, a_deg: float, h: float, radius: float, facing: str) -> list:
        p = self.phi(a_deg)
        rv = matrix_to_rotvec(self.orientation(p, facing))
        return [radius * math.cos(p), radius * math.sin(p), h] + rv

    def draw_pose(self, a: float, h: float) -> list:
        return self.pose(self.clamp_a(a), self.clamp_h(h), self.r_draw, "out")

    def lifted_pose(self, a: float, h: float) -> list:
        return self.pose(self.clamp_a(a), self.clamp_h(h), self.r_lift, "out")

    def park_pose(self, a: float, h: float) -> list:
        if not self.tilt:
            return self.lifted_pose(a, h)
        return self.pose(self.clamp_a(a), self.travel_h(self.clamp_h(h)), self.r_ret, "down")

    def describe_pose(self, pose, base_joint: float | None = None) -> dict:
        x, y, z = pose[:3]
        phi = math.atan2(y, x)
        if base_joint is not None:
            phi = base_joint + wrap_pi(phi - base_joint)
        a = self.dir * (math.degrees(phi) - self.start)
        tz = [row[2] for row in rotvec_to_matrix(pose[3:6])]
        r = math.hypot(x, y)
        radial = (tz[0] * x + tz[1] * y) / r if r > 1e-6 else 0.0
        if radial > 0.95:
            facing = "tuch"
        elif tz[2] < -0.95:
            facing = "unten"
        else:
            facing = "kippt"
        return {"a": a, "h": z, "r": r, "facing": facing,
                "gap": self.R - r if facing == "tuch" else None}
