from __future__ import annotations

import math

from geometry import Cylinder
from lamp import LampError
from reach import ParkJoints

TAU = 2 * math.pi
JOINT_MARGIN = 0.05


class Aborted(Exception):
    pass


class Plotter:
    def __init__(self, robot, cyl: Cylinder, cfg: dict, abort, report, lamp=None):
        self.robot = robot
        self.lamp = lamp if cyl.switched else None
        self.cyl = cyl
        self.cfg = cfg
        self.abort = abort
        self.report = report
        lim = robot.limits
        self.v_travel = min(cfg["travel_speed_mps"] or lim.tcp_speed_max, lim.tcp_speed_max)
        self.w_rewind = min(cfg["rewind_speed_radps"] or lim.joint_speed_max, lim.joint_speed_max)
        self.dwell = cfg["point_dwell_s"]
        self.state = None
        self.rewound = False
        self.a = 0.0
        self.h = cyl.h_min
        self.q0_ref = None

    def _check(self):
        if self.abort.is_set():
            raise Aborted()

    def _wait(self, s: float):
        if s > 0 and self.abort.wait(s):
            raise Aborted()

    def _move(self, pose, speed, what: str):
        self._check()
        ok = self.robot.move_to(pose, speed=speed)
        self._check()
        if ok is False:
            raise RuntimeError(f"Bewegung nicht ausgeführt: {what}")

    def _move_joints(self, q, what: str):
        self._check()
        ok = self.robot.move_joints(q, speed=self.w_rewind)
        self._check()
        if ok is False:
            raise RuntimeError(f"Gelenkbewegung nicht ausgeführt: {what}")

    def _lamp_on(self):
        if self.lamp is None:
            return
        self._check()
        self.lamp.set(True)
        self._check()
        self._wait(self.cfg["lamp_settle_s"])

    def _lamp_off(self):
        if self.lamp is not None:
            self.lamp.set(False)

    def _lamp_off_quiet(self):
        try:
            self._lamp_off()
        except LampError:
            pass

    def wait_lamp(self):
        if self.lamp is None or not self.lamp.required:
            return
        while not self.lamp.connected():
            self.report(step="wartet auf DMX: auf der Seite «DMX verbinden» drücken")
            self._wait(0.3)

    def base_for(self, a: float) -> float:
        return self.q0_ref + self.cyl.dir * math.radians(a)

    def joints_for_park(self, a: float, h: float, qnear=None):
        q, phi_q = self.park.joints(h)
        b = self.cyl.phi(a) + (q[0] - phi_q)
        q[0] = b + TAU * round((self.base_for(a) - b) / TAU)
        return q

    def setup(self, table) -> dict:
        cyl = self.cyl
        self.park = ParkJoints(cyl, table)
        phi0 = cyl.phi(0.0)
        q, phi_q = self.park.joints(cyl.h_min)
        q0 = phi0 + (q[0] - phi_q)
        q0 += TAU * round((phi0 - q0) / TAU)
        end = q0 + cyl.dir * TAU
        lim = TAU - JOINT_MARGIN
        if not (-lim <= q0 <= lim and -lim <= end <= lim):
            raise RuntimeError(
                f"Basisgelenk liefe von {math.degrees(q0):.0f}° bis {math.degrees(end):.0f}°, "
                f"erlaubt sind ±{math.degrees(lim):.0f}°: start_angle_deg in config.json anpassen")
        self.q0_ref = q0
        return {"base_start_deg": math.degrees(q0), "base_end_deg": math.degrees(end)}

    def validate(self, planned: list) -> None:
        cyl, lim = self.cyl, self.robot.limits
        for i, p in enumerate(planned):
            (a0, h0), (a1, h1) = p["pts"][0], p["pts"][-1]
            poses = [cyl.park_pose(a0, h0), cyl.lifted_pose(a0, h0), cyl.lifted_pose(a1, h1), cyl.park_pose(a1, h1)]
            poses += [cyl.draw_pose(a, h) for a, h in p["pts"]]
            lim.check_tcp_speed(p["speed"], lim.tcp_accel_max)
            for a, h in p["pts"]:
                if not (0.0 <= a <= 360.0 and cyl.h_min <= h <= cyl.h_max):
                    raise RuntimeError(f"Pfad {i + 1}: Punkt {a:.1f}°, {h:.3f} m ausserhalb des Bereichs")
            for pose in poses:
                lim.check_pose_target(pose)

    def recover(self, home: bool = False) -> None:
        """Lamp off, pull back, straighten up, park beside the cloth.
        home=True (Zurücksetzen / «Arm in Fahrstellung»): park at home_height_m,
        then turn back to the cloth start (0°), the way it came, base joint only."""
        cyl = self.cyl
        self._lamp_off()
        self.report(step="Lampe sichern: zurückziehen und nach unten drehen")
        pose = list(self.robot.pose())
        d = cyl.describe_pose(pose, self.robot.joints()[0])
        phi = math.atan2(pose[1], pose[0])
        if d["facing"] != "unten" and d["r"] > cyl.r_lift + 0.005:
            back = [cyl.r_lift * math.cos(phi), cyl.r_lift * math.sin(phi), pose[2]] + pose[3:6]
            self._move(back, self.v_travel, "zurückziehen")
        q = list(self.robot.joints())
        self.report(step="Arm aufrichten (Ellbogen nach oben)")
        self._move_joints([q[0], -math.pi / 2, math.pi / 2, -math.pi / 2, math.pi / 2, q[5]], "aufrichten")
        # home: always end at the same start pose (cloth start 0°, home_height_m, lamp facing the cloth)
        h_home = self.cfg.get("home_height_m")
        h = cyl.travel_h(cyl.clamp_h(h_home if home and h_home is not None else pose[2]))
        a = cyl.clamp_a(d["a"])
        self.report(step="in die geprüfte Fahrstellung")
        self._move_joints(self.joints_for_park(a, h), "Fahrstellung")
        self.state, self.a, self.h = "park", a, h
        if home and abs(a) > 0.5:
            self.rewind()

    def rewind(self) -> None:
        if self.state == "surface":
            self._lamp_off()
            self.report(step="Rücklauf zum Startwinkel, Lampe aus, nur das Basisgelenk dreht", rewinding=True)
            q = list(self.robot.joints())
            q[0] -= self.cyl.dir * math.radians(self.a)
            self._move_joints(q, "Rücklauf")
            self.a = 0.0
            self.report(rewinding=False)
            return
        self.report(step="Rücklauf zum Startwinkel (Lampe zeigt nach unten)", rewinding=True)
        self._move_joints(self.joints_for_park(0.0, self.h, self.robot.joints()), "Rücklauf")
        self.a = 0.0
        self.report(rewinding=False)

    def travel(self, a: float, h: float) -> None:
        cyl = self.cyl
        ht = cyl.travel_h(cyl.clamp_h(h))
        if abs(a - self.a) < 1e-6 and abs(ht - self.h) < 1e-6:
            return
        n = max(1, math.ceil(abs(a - self.a) / self.cfg["travel_step_deg"]))
        a0, h0 = self.a, self.h
        for k in range(1, n + 1):
            t = k / n
            self._move(cyl.park_pose(a0 + (a - a0) * t, h0 + (ht - h0) * t), self.v_travel, "fahren")
        self.a, self.h = a, ht

    def glide(self, a: float, h: float) -> None:
        cyl = self.cyl
        h = cyl.clamp_h(h)
        if abs(a - self.a) < 1e-6 and abs(h - self.h) < 1e-6:
            return
        n = max(1, math.ceil(abs(a - self.a) / self.cfg["travel_step_deg"]))
        a0, h0 = self.a, self.h
        for k in range(1, n + 1):
            t = k / n
            self._move(cyl.draw_pose(a0 + (a - a0) * t, h0 + (h - h0) * t), self.v_travel, "fahren")
        self.a, self.h = a, h

    def draw_path_switched(self, p: dict, i: int, n: int) -> None:
        cyl = self.cyl
        a, h = p["pts"][0]
        if self.state == "park":
            self.report(step=f"Pfad {i + 1}/{n}: zum Tuch fahren")
            self._move(cyl.draw_pose(self.a, self.h), self.v_travel, "anfahren")
            self.state = "surface"
        self.report(step=f"Pfad {i + 1}/{n}: fahren, Lampe aus")
        self.glide(a, h)
        self.report(step=f"Pfad {i + 1}/{n}: Lampe an, belichten mit {p['speed'] * 1000:.0f} mm/s")
        try:
            self._lamp_on()
            self._wait(p["dwell"] + self.dwell)
            for a, h in p["pts"][1:]:
                self._move(cyl.draw_pose(a, h), p["speed"], "zeichnen")
                self._wait(self.dwell)
        except BaseException:
            self._lamp_off_quiet()
            raise
        self._lamp_off()
        self.a, self.h = a, cyl.clamp_h(h)

    def sweep(self) -> None:
        cyl = self.cyl
        self._lamp_off()
        if self.state not in ("park", "surface"):
            self.recover()
        h = cyl.clamp_h((cyl.h_min + cyl.h_max) / 2)
        if self.a > 0.5:
            self.rewind()
        if cyl.switched:
            if self.state == "park":
                self.report(step="Tuchabstand: zum Tuch fahren, Lampe aus")
                self._move(cyl.draw_pose(self.a, self.h), self.v_travel, "anfahren")
                self.state = "surface"
            self.glide(0.0, h)
            r = cyl.r_draw
            self.report(step=f"Tuchabstand: eine Runde in Zeichenabstand (Lampe {r:.3f} m von der Achse, Tuch {cyl.R:.3f} m), Lampe aus")
            self.glide(360.0, h)
        else:
            self.travel(0.0, h)
            self.report(step="Tuchabstand: eine Runde in Fahrstellung (Lampe ist immer an, deshalb nicht am Tuch)")
            self.travel(360.0, h)
        self.rewind()
        self.report(step="bereit")

    def height_test(self) -> None:
        cyl = self.cyl
        if not cyl.switched:
            raise RuntimeError("Höhentest braucht lamp_control \"dmx\": mit Dauerlicht würde er belichten")
        self._lamp_off()
        if self.state not in ("park", "surface"):
            self.recover()
        if self.a > 0.5:
            self.rewind()
        if self.state == "park":
            self.report(step="Höhentest: zum Tuch fahren, Lampe aus")
            self._move(cyl.draw_pose(self.a, self.h), self.v_travel, "anfahren")
            self.state = "surface"
        for name, h in (("höchster", cyl.h_max), ("tiefster", cyl.h_min)):
            self.report(step=f"Höhentest: {name} Zeichenpunkt, {h:.2f} m über der Basis, Lampe aus")
            self.glide(0.0, h)
            self._wait(3.0)
        self.glide(0.0, (cyl.h_min + cyl.h_max) / 2)
        self.report(step="bereit")

    def short_hop(self, a0: float, h0: float, a1: float, h1: float) -> bool:
        hop = self.cfg.get("short_hop_m") or 0.0
        if not self.cyl.tilt or hop <= 0:
            return False
        d = math.hypot(self.cyl.arc_m(a1 - a0), h1 - h0)
        return d <= hop and abs(a1 - a0) <= self.cfg["travel_step_deg"]

    def draw_path(self, p: dict, i: int, n: int, nxt=None) -> None:
        cyl = self.cyl
        a, h = p["pts"][0]
        if self.state == "lifted":
            self.report(step=f"Pfad {i + 1}/{n}: kurzer Sprung, Lampe bleibt aufs Tuch gerichtet")
            self._move(cyl.lifted_pose(a, h), self.v_travel, "kurzer Sprung")
        else:
            self.report(step=f"Pfad {i + 1}/{n}: fahren")
            self.travel(a, h)
            self.report(step=f"Pfad {i + 1}/{n}: aufrichten und anfahren")
            if cyl.tilt:
                self._move(cyl.lifted_pose(a, h), self.v_travel, "aufrichten")
        self._move(cyl.draw_pose(a, h), self.v_travel, "anfahren")
        self.state = "draw"
        self.report(step=f"Pfad {i + 1}/{n}: belichten mit {p['speed'] * 1000:.0f} mm/s")
        self._wait(p["dwell"] + self.dwell)
        for a, h in p["pts"][1:]:
            self._move(cyl.draw_pose(a, h), p["speed"], "zeichnen")
            self._wait(self.dwell)
        self.report(step=f"Pfad {i + 1}/{n}: zurückziehen")
        self._move(cyl.lifted_pose(a, h), self.v_travel, "zurückziehen")
        if nxt is not None and self.short_hop(a, h, *nxt["pts"][0]):
            self.state, self.a, self.h = "lifted", a, h
            return
        if cyl.tilt:
            self._move(cyl.park_pose(a, h), self.v_travel, "nach unten drehen")
        self.state, self.a, self.h = "park", a, cyl.travel_h(cyl.clamp_h(h))

    def run_job(self, planned: list, a0: float, rewind: bool, on_path) -> None:
        self.rewound = False
        if self.state not in ("park", "surface"):
            self.recover()
        self.wait_lamp()
        self.report(step="Auftrag prüfen")
        self.validate(planned)
        if rewind or self.a > a0 + self.cyl.job_arc + 1e-6:
            self.rewind()
            self.rewound = True
        for i, p in enumerate(planned):
            on_path(i)
            if self.cyl.switched:
                self.draw_path_switched(p, i, len(planned))
            else:
                self.draw_path(p, i, len(planned), planned[i + 1] if i + 1 < len(planned) else None)
        on_path(len(planned))
        self.report(step="bereit")
