from __future__ import annotations

import json
import math
import os
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from ur5e import Robot
from ur5e.cli import argument_parser, robot_kwargs
from ur5e.config import is_sim, resolve_host
from ur5e.connection import RobotConnection

import reach
from executor import Aborted, Plotter
from lamp import Lamp, LampError
from geometry import HERE, Cylinder, load_config
from planner import JobError, JobQueue, Lap, make_job, plan_job

PAGE = HERE.parent / "tonspur-tuch.html"
HISTORY = 40
HOST_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9.\-]{0,62}$")


def dashboard_stop(host: str) -> str:
    from dashboard_client import DashboardClient
    dc = DashboardClient(host)
    dc.connect(2000)
    try:
        dc.stop()
    finally:
        dc.disconnect()
    return "Programm über Dashboard gestoppt"


class Hub:
    def __init__(self, cfg, cyl, target, host, robot_kw, allow_real=False, idle=False):
        self.cfg, self.cyl = cfg, cyl
        # idle: don't connect until the page presses «Verbinden» (used by --real,
        # so the arm is only taken over after the confirm dialog on the page)
        self.idle = idle
        self.target, self.host, self.robot_kw = target, host, robot_kw
        self.allow_real = allow_real or target == "real"
        self.sim_host = resolve_host("sim")
        self.gen = 0
        self.tag = "[SIM]" if target == "sim" else "[ECHTER ROBOTER]"
        self.lock = threading.RLock()
        self.queue = JobQueue(cfg)
        self.lap = Lap(cyl)
        self.current = None
        self.history = []
        self.estop = False
        self.secure_now = False
        self.seed = None
        self.watch_tool = reach.Tool(cfg["lamp_tcp"])
        self.cloth_reset_pending = False
        self.lamp = Lamp(cfg, target == "real") if cyl.switched else None
        self.abort = threading.Event()
        self.wake = threading.Event()
        self.version = 0
        self.vmax = 0.25
        self.status = {"connected": False,
                       "step": "nicht verbunden: auf der Seite verbinden" if idle else "verbinde …", "error": "", "rewinding": False,
                       "base_range": None, "estop_msg": ""}
        self.live = {}

    def log(self, msg):
        print(f"{self.tag} {time.strftime('%H:%M:%S')} {msg}", flush=True)

    def bump(self):
        self.version += 1

    def report(self, **kw):
        with self.lock:
            self.status.update(kw)

    def add_job(self, data):
        job = make_job(data, self.cfg)
        with self.lock:
            if self.estop:
                raise JobError("NOT-AUS aktiv: erst zurücksetzen")
            gone = self.queue.add(job)
            self.bump()
        self.log(f"Auftrag {job['id']} «{job['name']}» angenommen ({len(job['paths'])} Pfade)")
        if gone is not None:
            self.log(f"Warteschlange voll: Auftrag {gone['id']} verworfen ({self.queue.policy})")
        self.wake.set()
        return job, gone

    def estop_now(self, reason=None):
        if self.lamp is not None:
            self.lamp.set(False, wait=False)
        with self.lock:
            self.estop = True
            self.abort.set()
            cleared = self.queue.clear()
            self.bump()
        self.log(f"NOT-AUS: {len(cleared)} wartende Aufträge gelöscht")
        try:
            msg = dashboard_stop(self.host)
        except Exception as e:
            msg = f"Dashboard-Stopp fehlgeschlagen ({e}); Arm hält nach dem laufenden Segment an"
        if reason:
            msg = f"{reason}. {msg}"
        self.log(msg)
        self.report(estop_msg=msg, step="NOT-AUS")
        return msg

    def reset(self):
        with self.lock:
            self.estop = False
            self.secure_now = True
            self.abort.clear()
            self.status.update(error="", estop_msg="")
            self.bump()
        self.log("Zurückgesetzt")
        self.wake.set()

    def park(self):
        with self.lock:
            if self.estop:
                raise JobError("NOT-AUS aktiv: erst zurücksetzen")
            if self.current:
                raise JobError("Ein Auftrag läuft: erst danach")
            if not self.status.get("connected"):
                raise JobError("nicht verbunden")
            self.secure_now = True
        self.log("Fahrstellung angefordert")
        self.wake.set()

    def set_dry(self, on: bool):
        with self.lock:
            if self.current:
                raise JobError("Ein Auftrag läuft: erst danach umschalten")
            lamp = self.lamp
            with lamp.cond:
                lamp.dry = on
                lamp.required = self.target == "real" and not on
                lamp.on = False
                lamp.seq += 1
                lamp.lit_by_dmx = False
                lamp.cond.notify_all()
            self.bump()
        self.log("TROCKENLAUF an: nur Bewegung, Lampe bleibt aus" if on else "Trockenlauf aus: Lampe wird wieder geschaltet")

    def switch(self, data):
        target = data.get("target")
        host = str(data.get("host") or "").strip()
        try:
            payload = float(data.get("payload") or 0.0)
        except (TypeError, ValueError):
            raise JobError("Last muss eine Zahl in kg sein")
        if target not in ("sim", "real"):
            raise JobError("Ziel muss sim oder real sein")
        if target == "sim" and not host:
            host = self.sim_host
        if not HOST_RE.match(host):
            raise JobError(f"«{host}» ist keine gültige IP-Adresse oder kein Hostname")
        if target == "sim" and not is_sim(host):
            raise JobError(f"{host} ist kein Simulator: für einen echten Roboter «Echter Roboter» wählen")
        if target == "real":
            if not self.allow_real:
                raise JobError("Echter Roboter gesperrt: Server mit --allow-real starten (z. B. ./start --allow-real)")
            if is_sim(host):
                raise JobError(f"{host} ist der Simulator: dafür «Simulator» wählen")
            if not 0 < payload <= 5:
                raise JobError("Für den echten Roboter die gemessene Masse der Lampe in kg angeben (0 < Last ≤ 5)")
        with self.lock:
            if self.current:
                raise JobError("Ein Auftrag läuft: warten oder NOT-AUS drücken, dann wechseln")
            cleared = len(self.queue.clear())
            kw = {"target": target, "host": host, "payload_kg": payload if target == "real" else 0.0,
                  "tcp": tuple(self.cfg["lamp_tcp"])}
            if target == "real":
                kw["allow_real"] = True
            self.target, self.host, self.robot_kw = target, host, kw
            self.tag = "[SIM]" if target == "sim" else "[ECHTER ROBOTER]"
            if self.lamp is not None:
                self.lamp.required = target == "real"
                self.lamp.dry = False
            self.gen += 1
            self.idle = False
            self.live = {}
            self.status.update(connected=False, step="wechsle Verbindung …", error="", base_range=None)
            self.bump()
        who = f"Simulator {host}" if target == "sim" else f"ECHTER ROBOTER {host}, Last {payload} kg"
        self.log(f"Verbindung gewechselt zu {who}" + (f"; {cleared} wartende Aufträge gelöscht" if cleared else ""))
        self.wake.set()
        return who, cleared

    def reset_cloth(self):
        with self.lock:
            self.history = []
            if self.current:
                self.cloth_reset_pending = True
                msg = "Ansicht geleert; nach dem laufenden Auftrag beginnt Runde 1 bei 0°"
            else:
                self.lap.lap, self.lap.cursor = 1, 0.0
                msg = "neues Tuch: Runde 1 ab 0°"
            self.bump()
        self.log(msg)
        return msg

    def clear_queue(self):
        with self.lock:
            n = len(self.queue.clear())
            self.bump()
        self.log(f"Warteschlange geleert ({n})")
        return n

    def job_view(self, job, paths=True):
        out = {k: job.get(k) for k in ("id", "name", "mode", "mic", "trigger", "state", "lap", "a0", "a1", "rewind", "progress")}
        out["npaths"] = len(job["paths"])
        if paths and job.get("planned") is not None:
            out["paths"] = [{"p": [[round(a, 3), round(h, 4)] for a, h in p["pts"]],
                             "v": round(p["speed"], 4), "d": round(p["dwell"], 2)} for p in job["planned"]]
        return out

    def plan_view(self):
        with self.lock:
            jobs = [self.job_view(j) for j in self.history[-HISTORY:]]
            lap, cursor = self.lap.lap, self.lap.cursor
            if self.current:
                jobs.append(self.job_view(self.current))
                lap, cursor = self.current["lap"], self.current["a1"]
            for j in self.queue.items:
                lap, a0, rew = self.lap.place(lap, cursor)
                planned = plan_job(self.cyl, self.cfg, j, a0, self.vmax)
                v = self.job_view(dict(j, lap=lap, a0=a0, a1=a0 + self.cyl.job_arc, rewind=rew, planned=planned))
                jobs.append(v)
                cursor = a0 + self.cyl.job_arc
            return {"version": self.version, "jobs": jobs}

    def status_view(self):
        with self.lock:
            cur = self.current
            return dict(self.status, target=self.target, host=self.host, estop=self.estop, idle=self.idle,
                        allow_real=self.allow_real, sim_host=self.sim_host,
                        payload=self.robot_kw.get("payload_kg") or 0.0,
                        version=self.version, lap=self.lap.lap, cursor=self.lap.cursor,
                        live=self.live, lamp=self.lamp.view() if self.lamp else {"control": "always_on"},
                        queue=[self.job_view(j, False) for j in self.queue.items],
                        dropped=len(self.queue.dropped),
                        current=self.job_view(cur, False) if cur else None)

    def config_view(self):
        return dict(self.cfg, target=self.target, host=self.host, vmax=self.vmax,
                    arc_m=self.cyl.arc_m(self.cyl.job_arc))

    def worker(self):
        robot = plotter = None
        gen = self.gen
        while True:
            if gen != self.gen:
                if robot is not None:
                    robot.disconnect()
                    robot = plotter = None
                gen = self.gen
            if self.idle:
                self.wake.wait(0.5)
                self.wake.clear()
                continue
            if self.estop:
                if robot is not None:
                    robot.disconnect()
                    robot = plotter = None
                    self.report(connected=False, step="NOT-AUS: zurücksetzen, um weiterzumachen")
                self.wake.wait(0.5)
                self.wake.clear()
                continue
            if robot is None:
                try:
                    self.report(step="verbinde …")
                    with self.lock:
                        kw = dict(self.robot_kw)
                    robot = Robot(**kw).connect()
                    if gen != self.gen:
                        robot.disconnect()
                        robot = plotter = None
                        continue
                    self.vmax = robot.limits.tcp_speed_max
                    plotter = Plotter(robot, self.cyl, self.cfg, self.abort, self.report, self.lamp)
                    rng = plotter.setup(self.seed)
                    self.report(connected=True, error="", base_range=rng, step="bereit")
                    self.log(f"verbunden mit {self.host}, Profil '{robot.limits.name}', "
                             f"Basisgelenk {rng['base_start_deg']:.0f}° bis {rng['base_end_deg']:.0f}°")
                    with self.lock:
                        self.bump()
                except Exception as e:
                    if robot is not None:
                        robot.disconnect()
                    robot = plotter = None
                    self.report(connected=False, error=f"Verbindung: {e}", step="keine Verbindung")
                    self.wake.wait(3)
                    self.wake.clear()
                    continue
            if self.secure_now and gen == self.gen:
                self.secure_now = False
                self.log("fahre in die Fahrstellung (zurückziehen, aufrichten, Fahrstellung)")
                try:
                    plotter.recover(home=True)
                    self.report(step="bereit", error="")
                    self.log("in der Fahrstellung am Tuchanfang (0°)")
                except Aborted:
                    self.log("Fahrstellung abgebrochen")
                    plotter.state = None
                except Exception as e:
                    self.log(f"Fahrstellung nicht erreicht: {e}")
                    self.report(error=f"Fahrstellung: {e}", step="bereit")
                    plotter.state = None
                continue
            with self.lock:
                job = self.queue.pop() if gen == self.gen else None
                if job:
                    lap, a0, rew = self.lap.place()
                    job.update(lap=lap, a0=a0, a1=a0 + self.cyl.job_arc, rewind=rew, progress=0, state="zeichnet",
                               planned=plan_job(self.cyl, self.cfg, job, a0, self.vmax))
                    self.current = job
                    self.bump()
            if not job:
                self.wake.wait(0.5)
                self.wake.clear()
                continue
            self.log(f"zeichne Auftrag {job['id']} in Runde {lap}, {a0:.1f}° bis {a0 + self.cyl.job_arc:.1f}°"
                     + (" (vorher Rücklauf)" if rew else ""))

            def on_path(i, job=job):
                with self.lock:
                    job["progress"] = i
                    self.bump()

            try:
                plotter.run_job(job["planned"], a0, rew, on_path)
                state = "fertig"
            except Aborted:
                state = "abgebrochen"
            except LampError as e:
                state = f"Fehler: {e}"
                self.report(error=f"Auftrag {job['id']}: {e}")
                self.estop_now(f"Lampe: {e}")
            except Exception as e:
                state = "abgebrochen" if self.abort.is_set() else f"Fehler: {e}"
                if not self.abort.is_set():
                    self.report(error=f"Auftrag {job['id']}: {e}")
            self.log(f"Auftrag {job['id']}: {state}")
            with self.lock:
                job["state"] = state
                if state == "fertig" or job.get("progress") or plotter.rewound:
                    self.lap.commit(lap, a0)
                if self.cloth_reset_pending:
                    self.cloth_reset_pending = False
                    self.lap.lap, self.lap.cursor = 1, 0.0
                    self.log("neues Tuch: Runde 1 ab 0°")
                else:
                    self.history.append(job)
                    self.history = self.history[-HISTORY:]
                self.current = None
                self.bump()
            if state != "fertig":
                plotter.state = None
                robot.disconnect()
                robot = plotter = None

    def poller(self):
        conn = None
        host = None
        while True:
            try:
                if conn is not None and host != self.host:
                    conn.disconnect()
                    conn = None
                if conn is None:
                    host = self.host
                    conn = RobotConnection(host).connect()
                q = conn.receive.getActualQ()
                pose = conn.receive.getActualTCPPose()
                d = self.cyl.describe_pose(pose, q[0])
                live = {"a": round(d["a"], 2), "h": round(d["h"], 4), "r": round(d["r"], 4),
                        "facing": d["facing"], "gap": None if d["gap"] is None else round(d["gap"], 4),
                        "base_deg": round(math.degrees(q[0]), 1), "t": time.time()}
                with self.lock:
                    self.live = live
                    armed = not self.estop and not self.idle
                bad = reach.below_base(q, self.watch_tool)
                if bad and armed:
                    self.log(f"WÄCHTER: {bad} unter der Basis (Gelenke {[round(math.degrees(v)) for v in q]}°)")
                    self.estop_now(f"Wächter: {bad} unter der Basis")
                if self.lamp is not None and armed and self.lamp.lost_while_on():
                    self.log("WÄCHTER: DMX-Verbindung verloren, während die Lampe an war")
                    self.estop_now("Wächter: DMX-Verbindung verloren, Lampe evtl. noch an")
                time.sleep(0.1)
            except Exception:
                if conn is not None:
                    conn.disconnect()
                conn = None
                with self.lock:
                    self.live = {}
                time.sleep(2)


class Handler(BaseHTTPRequestHandler):
    hub: Hub = None

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json; charset=utf-8"):
        data = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        path = urlparse(self.path).path
        hub = self.hub
        if path in ("/", "/index.html", "/tonspur-tuch.html"):
            return self._send(200, PAGE.read_bytes(), "text/html; charset=utf-8")
        if path == "/api/config":
            return self._send(200, hub.config_view())
        if path == "/api/status":
            return self._send(200, hub.status_view())
        if path == "/api/plan":
            return self._send(200, hub.plan_view())
        self._send(404, {"error": "not found"})

    def do_POST(self):
        origin = self.headers.get("Origin")
        if origin and urlparse(origin).netloc != self.headers.get("Host"):
            return self._send(403, {"error": "fremde Herkunft"})
        if not (self.headers.get("Content-Type") or "").startswith("application/json"):
            return self._send(415, {"error": "application/json erwartet"})
        n = int(self.headers.get("Content-Length") or 0)
        try:
            data = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            return self._send(400, {"error": "kein gültiges JSON"})
        hub, path = self.hub, urlparse(self.path).path
        if path.startswith("/api/lamp/"):
            if hub.lamp is None:
                return self._send(409, {"error": "lamp_control ist nicht dmx"})
            try:
                if path == "/api/lamp/poll":
                    return self._send(200, hub.lamp.poll(int(data.get("since", -1)), str(data.get("device") or "DMX")[:60]))
                if path == "/api/lamp/ack":
                    hub.lamp.ack(int(data["seq"]), str(data.get("device") or "DMX")[:60])
                    return self._send(200, {"ok": True})
                if path == "/api/lamp/dry":
                    hub.set_dry(bool(data.get("on")))
                    return self._send(200, {"ok": True})
                if path == "/api/lamp/release":
                    hub.lamp.release()
                    return self._send(200, {"ok": True})
            except (KeyError, TypeError, ValueError, JobError) as e:
                return self._send(400, {"error": str(e)})
        if path == "/api/estop":
            return self._send(200, {"ok": True, "msg": hub.estop_now()})
        if path == "/api/connect":
            try:
                who, cleared = hub.switch(data)
            except JobError as e:
                return self._send(400, {"error": str(e)})
            return self._send(200, {"ok": True, "msg": f"verbinde mit {who}", "cleared": cleared})
        if path == "/api/park":
            try:
                hub.park()
            except JobError as e:
                return self._send(400, {"error": str(e)})
            return self._send(200, {"ok": True})
        if path == "/api/reset":
            hub.reset()
            return self._send(200, {"ok": True})
        if path == "/api/cloth/reset":
            return self._send(200, {"ok": True, "msg": hub.reset_cloth()})
        if path == "/api/queue/clear":
            return self._send(200, {"ok": True, "cleared": hub.clear_queue()})
        if path == "/api/jobs":
            try:
                job, gone = hub.add_job(data)
            except (JobError, KeyError, TypeError, ValueError) as e:
                return self._send(400, {"error": str(e)})
            return self._send(200, {"ok": True, "id": job["id"], "dropped": gone["id"] if gone else None})
        self._send(404, {"error": "not found"})


def main():
    ap = argument_parser("Tonspur-Tuch: UV-Plotter auf dem Zylinder. Standardziel ist der Simulator.")
    ap.add_argument("--config", default=None, help="config.json (Standard: neben server.py)")
    ap.add_argument("--port", type=int, default=None, help="Port der Vorschau (Standard aus config.json)")
    ap.add_argument("--allow-real", action="store_true",
                    help="erlaubt, auf der Seite zu einem echten Roboter zu wechseln (IP und Last dort eingeben)")
    ap.add_argument("--real", action="store_true",
                    help="ohne Simulator starten: Seite steht auf «Echter Roboter» (IP aus --host / UR_HOST, "
                         "sonst 192.168.1.11), verbunden wird erst mit dem Knopf auf der Seite")
    args = ap.parse_args()
    if args.real:
        if args.target == "real":
            raise SystemExit("--real und --target real nicht zusammen verwenden")
        args.allow_real = True
    cfg = load_config(args.config)
    cyl = Cylinder(cfg)
    roll, problems, seed = reach.choose_roll(cyl)
    if problems:
        raise SystemExit(f"config.json verlangt Posen, die der Arm nicht erreicht (beste Drehung der Lampe {roll:.0f}°):\n  "
                         + "\n  ".join(problems))
    cyl.roll = roll
    if args.real:
        host = args.host or os.environ.get("UR_HOST") or "192.168.1.11"
        if is_sim(host):
            raise SystemExit(f"--real: {host} ist der Simulator")
        kw = {"target": "real", "host": host, "allow_real": True, "payload_kg": args.payload,
              "tcp": tuple(cfg["lamp_tcp"])}
        hub = Hub(cfg, cyl, "real", host, kw, True, idle=True)
        return serve(hub, cfg, cyl, args, host, seed)
    kw = robot_kwargs(args)
    if args.target == "real" and args.payload <= 0:
        raise SystemExit("--target real braucht --payload KG mit der gemessenen Masse der Lampe")
    kw["payload_kg"] = args.payload
    kw["tcp"] = tuple(cfg["lamp_tcp"])
    host = kw.get("host") or resolve_host(args.target)
    kw["host"] = host
    hub = Hub(cfg, cyl, args.target, host, kw, args.allow_real)
    serve(hub, cfg, cyl, args, host, seed)


def serve(hub, cfg, cyl, args, host, seed=None):
    hub.seed = seed
    Handler.hub = hub
    target = hub.target
    port = args.port or cfg["port"]
    line = "=" * 64
    if not cyl.switched:
        lamp_line = "immer an, wird zum Fahren nach unten gedreht"
    elif target == "real":
        lamp_line = f"über DMX Kanal {cfg['dmx_channel']}, Pflicht: Aufträge warten, bis die Seite DMX verbunden hat"
    else:
        lamp_line = f"über DMX Kanal {cfg['dmx_channel']}, ohne DMX-Verbindung nur simuliert"
    who = f"SIMULATOR ({host})" if target == "sim" else f"!!! ECHTER ROBOTER {host} !!!"
    if hub.idle:
        who += ("\n  noch NICHT verbunden: auf der Seite Lampe kg eingeben und"
                "\n  «Mit echtem Roboter verbinden» drücken")
    if hub.allow_real and target == "sim":
        who += "\n  !!! --allow-real: auf der Seite kann zu einem echten Roboter gewechselt werden !!!"
    print(f"{line}\n  ZIEL: {who}\n  Vorschau: http://localhost:{port}/\n"
          f"  Tuch r={cyl.R} m, Lampe {cfg['lamp_distance_m'] * 100:.0f} cm davor, "
          f"Höhe {cyl.h_min:.2f}…{cyl.h_max:.2f} m, Start {cyl.start:.0f}°, {cyl.job_arc:.0f}° pro Auftrag\n"
          f"  Lampe TCP {[round(v, 4) for v in cfg['lamp_tcp']]}, um den Strahl gedreht {cyl.roll:.0f}°\n"
          f"  Lampe: {lamp_line}\n{line}",
          flush=True)
    threading.Thread(target=hub.worker, daemon=True).start()
    threading.Thread(target=hub.poller, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        hub.estop_now()
        print("beendet", flush=True)


if __name__ == "__main__":
    main()
