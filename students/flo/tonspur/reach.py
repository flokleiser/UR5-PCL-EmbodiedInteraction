from __future__ import annotations

import hashlib
import json
import math
import random

from geometry import HERE, rotvec_to_matrix

D = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]
A = [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0]
AL = [math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0]
SEEDS = [[0, -41, 103, -61, 75, 0], [0, -60, 120, -60, 90, 0], [0, -90, 130, -40, 90, 0],
         [0, -58, 98, 50, 90, 0], [0, -78, 86, 81, 90, 0], [0, -90, -90, -90, 90, 0],
         [0, -60, 100, -130, -90, 0], [0, -100, -100, -70, -90, 0]]
ROLLS = (0.0, 90.0, 180.0, 270.0)
STEP_H = 0.025
TILT_STEP = 10.0
RADIAL_STEP = 0.02
MAX_STEP_JUMP = 0.35
ELBOW_MIN_DEG = 15.0
WRIST_MIN_DEG = 15.0
AXIS_MIN_R = 0.10
JOINT_RADIUS = 0.06
LAMP_RADIUS = 0.03
LAMP_LENGTH = 0.20
JOINT_NAMES = ("Gelenk 3", "Gelenk 4", "Gelenk 5", "Gelenk 6", "Flansch")
CACHE = HERE / "reach_cache.json"


def _mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _dh(t, d, a, al):
    ct, st, ca, sa = math.cos(t), math.sin(t), math.cos(al), math.sin(al)
    return [[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st], [0, sa, ca, d], [0, 0, 0, 1]]


def _rot(axis, ang):
    x, y, z = axis
    c, s, t = math.cos(ang), math.sin(ang), 1 - math.cos(ang)
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c]]


def _mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


class Tool:
    def __init__(self, tcp):
        self.p = list(tcp[:3])
        self.R = rotvec_to_matrix(tcp[3:6])

    def frames(self, q):
        T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        out = []
        for i in range(6):
            T = _mul(T, _dh(q[i], D[i], A[i], AL[i]))
            out.append(T)
        return out

    def pose(self, q, F=None):
        T = (F or self.frames(q))[-1]
        Rf = [row[:3] for row in T[:3]]
        p = [T[i][3] + sum(Rf[i][k] * self.p[k] for k in range(3)) for i in range(3)]
        R = [[sum(Rf[i][k] * self.R[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        return p, R


def target(cyl, phi, beta, roll):
    Rd = cyl.orientation(phi, "out", roll)
    if beta:
        Rd = _mm(_rot((-math.sin(phi), math.cos(phi), 0.0), math.radians(beta)), Rd)
    return Rd


def _err(q, tool, cyl, spec):
    r, h, beta, roll = spec
    p, R = tool.pose(q)
    phi = math.atan2(p[1], p[0])
    Rd = target(cyl, phi, beta, roll)
    e = [p[0] - r * math.cos(phi), p[1] - r * math.sin(phi), p[2] - h]
    for i in (0, 2):
        e += [0.3 * (R[k][i] - Rd[k][i]) for k in range(3)]
    return e


def solve(q, tool, cyl, spec, iters=120):
    q = list(q)
    for _ in range(iters):
        e = _err(q, tool, cyl, spec)
        m = len(e)
        if math.sqrt(sum(v * v for v in e)) < 1e-7:
            return q
        cols = []
        for j in range(6):
            dq = list(q)
            dq[j] += 1e-6
            ej = _err(dq, tool, cyl, spec)
            cols.append([(ej[i] - e[i]) / 1e-6 for i in range(m)])
        J = [[cols[j][i] for j in range(6)] for i in range(m)]
        Mx = [[sum(J[i][k] * J[j][k] for k in range(6)) + (1e-5 if i == j else 0) for j in range(m)] for i in range(m)]
        aug = [Mx[i] + [-e[i]] for i in range(m)]
        for c in range(m):
            p = max(range(c, m), key=lambda k: abs(aug[k][c]))
            aug[c], aug[p] = aug[p], aug[c]
            for k in range(m):
                if k != c:
                    f = aug[k][c] / aug[c][c]
                    aug[k] = [aug[k][n] - f * aug[c][n] for n in range(m + 1)]
        y = [aug[i][m] / aug[i][i] for i in range(m)]
        dq = [sum(J[k][j] * y[k] for k in range(m)) for j in range(6)]
        s = max(1.0, max(abs(v) for v in dq) / 0.3)
        q = [q[j] + dq[j] / s for j in range(6)]
    return None


def problem(q, tool):
    if abs(math.sin(q[2])) < math.sin(math.radians(ELBOW_MIN_DEG)):
        return "Arm fast gestreckt (Gelenk 3)"
    if abs(math.sin(q[4])) < math.sin(math.radians(WRIST_MIN_DEG)):
        return "Handgelenk blockiert (Gelenk 5)"
    F = tool.frames(q)
    if math.hypot(F[3][0][3], F[3][1][3]) < AXIS_MIN_R:
        return "Handgelenk über der Basisachse"
    for n, T in zip(JOINT_NAMES, F[1:]):
        if T[2][3] - JOINT_RADIUS < 0:
            return f"{n} unter der Basis"
    p, R = tool.pose(q, F)
    if min(p[2] - f * LAMP_LENGTH * R[2][2] for f in (0.0, 0.5, 1.0)) - LAMP_RADIUS < 0:
        return "Lampe unter der Basis"
    return None


def below_base(q, tool):
    F = tool.frames(q)
    for n, T in zip(JOINT_NAMES, F[1:]):
        if T[2][3] < 0:
            return n
    p, R = tool.pose(q, F)
    if min(p[2] - f * LAMP_LENGTH * R[2][2] for f in (0.0, 0.5, 1.0)) < 0:
        return "Lampe"
    return None


def _jump(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def _step(q, tool, cyl, spec, what):
    nq = solve(q, tool, cyl, spec)
    if nq is None:
        return None, f"{what}: nicht erreichbar"
    if _jump(nq, q) > MAX_STEP_JUMP:
        return None, f"{what}: Gelenk springt (Singularität)"
    msg = problem(nq, tool)
    if msg:
        return None, f"{what}: {msg}"
    return nq, None


def chain(tool, cyl, roll, h, q_park):
    q = q_park
    hp = cyl.travel_h(h)
    if cyl.tilt:
        n = int(round(90.0 / TILT_STEP))
        for k in range(1, n + 1):
            t = k / n
            spec = (cyl.r_ret + (cyl.r_lift - cyl.r_ret) * t, hp + (h - hp) * t, 90.0 * (1 - t), roll)
            q, msg = _step(q, tool, cyl, spec, "Kippen")
            if q is None:
                return None, None, msg
    q_lift = q
    n = max(1, math.ceil((cyl.r_draw - cyl.r_lift) / RADIAL_STEP))
    for k in range(1, n + 1):
        r = cyl.r_lift + (cyl.r_draw - cyl.r_lift) * k / n
        q, msg = _step(q, tool, cyl, (r, h, 0.0, roll), "Anfahren")
        if q is None:
            return None, None, msg
    return q_lift, q, None


def park_spec(cyl, h, roll):
    if cyl.tilt:
        return (cyl.r_ret, cyl.travel_h(h), 90.0, roll)
    return (cyl.r_lift, h, 0.0, roll)


def _seeds():
    rnd = random.Random(7)
    out = [[math.radians(v) for v in s] for s in SEEDS]
    out += [[rnd.uniform(-math.pi, math.pi) for _ in range(6)] for _ in range(24)]
    return out


def scan_branch(cyl, roll, hs, q0):
    tool = Tool(cyl.cfg["lamp_tcp"])
    rows, prev = [], None
    for h in hs:
        spec = park_spec(cyl, h, roll)
        if prev is None:
            qp = solve(q0, tool, cyl, spec, 300)
            msg = "nicht erreichbar" if qp is None else problem(qp, tool)
        else:
            qp, msg = _step(prev[0], tool, cyl, spec, "Fahrstellung")
        if qp is None or msg:
            rows.append((h, None, f"Fahrstellung: {msg}" if prev is None else msg))
            break
        ql, qd, msg = chain(tool, cyl, roll, h, qp)
        if msg is None and prev is not None:
            if _jump(ql, prev[1]) > MAX_STEP_JUMP or _jump(qd, prev[2]) > MAX_STEP_JUMP:
                msg = "Stellung springt zwischen Höhen"
        if msg:
            rows.append((h, None, msg))
            break
        rows.append((h, (qp, ql, qd), None))
        prev = (qp, ql, qd)
    return rows


def heights(cyl):
    n = max(1, math.ceil((cyl.h_max - cyl.h_min) / STEP_H - 1e-9))
    return [cyl.h_min + (cyl.h_max - cyl.h_min) * k / n for k in range(n + 1)]


def best_branch(cyl, roll, hs=None):
    hs = hs or heights(cyl)
    best = None
    for s in _seeds():
        rows = scan_branch(cyl, roll, hs, s)
        good = sum(1 for r in rows if r[1])
        if best is None or good > best[0]:
            best = (good, rows)
        if good == len(hs):
            break
    return best[1]


def _key(cyl, roll):
    keys = ("cloth_radius_m", "lamp_distance_m", "lamp_tcp", "height_min_m", "height_max_m", "lift_radius_m",
            "retract_radius_m", "travel_height_max_m", "tilt_down_on_travel", "lamp_control")
    blob = json.dumps([roll, ELBOW_MIN_DEG, WRIST_MIN_DEG, MAX_STEP_JUMP] + [cyl.cfg.get(k) for k in keys])
    return hashlib.sha1(blob.encode()).hexdigest()


def check_roll(cyl, roll):
    key = _key(cyl, roll)
    try:
        cache = json.loads(CACHE.read_text())
    except (OSError, ValueError):
        cache = {}
    if key in cache:
        c = cache[key]
        return c["problems"], c["table"]
    hs = heights(cyl)
    rows = best_branch(cyl, roll, hs)
    problems = []
    if len(rows) < len(hs) or not rows[-1][1]:
        h, _, why = rows[-1]
        problems.append(f"Höhe {h:.3f} m: {why}")
    tool = Tool(cyl.cfg["lamp_tcp"])
    table = []
    for h, ok, _ in rows:
        if ok:
            p, _ = tool.pose(ok[0])
            table.append([cyl.travel_h(h), list(ok[0]), math.atan2(p[1], p[0])])
    cache[key] = {"problems": problems, "table": table}
    try:
        CACHE.write_text(json.dumps(cache))
    except OSError:
        pass
    return problems, table


def choose_roll(cyl):
    fixed = cyl.cfg.get("lamp_roll_deg")
    rolls = [float(fixed)] if fixed is not None else list(ROLLS)
    best = None
    for roll in rolls:
        problems, table = check_roll(cyl, roll)
        if not problems:
            return roll, [], table
        if best is None or len(table) > len(best[2]):
            best = (roll, problems, table)
    return best


class ParkJoints:
    def __init__(self, cyl, table):
        self.cyl = cyl
        self.tool = Tool(cyl.cfg["lamp_tcp"])
        self.table = sorted(table, key=lambda r: r[0])

    def joints(self, h):
        ht = self.cyl.travel_h(self.cyl.clamp_h(h))
        row = min(self.table, key=lambda r: abs(r[0] - ht))
        q = solve(row[1], self.tool, self.cyl, park_spec(self.cyl, ht, self.cyl.roll))
        if q is None or _jump(q, row[1]) > 1.0:
            q = list(row[1])
        p, _ = self.tool.pose(q)
        return q, math.atan2(p[1], p[0])
