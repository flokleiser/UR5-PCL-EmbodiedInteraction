from __future__ import annotations

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from ur5e import Robot
from ur5e.cli import argument_parser, robot_kwargs
from ur5e.config import resolve_host
from ur5e.connection import RobotConnection

import reach
from executor import Aborted, Plotter
from geometry import HERE, Cylinder, load_config
from planner import JobError, JobQueue, Lap, make_job, plan_job

PAGE = HERE.parent / "tonspur-tuch.html"
HISTORY = 40


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
    def __init__(self, cfg, cyl, target, host, robot_kw):
        self.cfg, self.cyl = cfg, cyl
        self.target, self.host, self.robot_kw = target, host, robot_kw
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
        self.abort = threading.Event()
        self.wake = threading.Event()
        self.version = 0
        self.vmax = 0.25
        self.status = {"connected": False, "step": "verbinde …", "error": "", "rewinding": False,
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
            return dict(self.status, target=self.target, host=self.host, estop=self.estop,
                        version=self.version, lap=self.lap.lap, cursor=self.lap.cursor,
                        live=self.live, queue=[self.job_view(j, False) for j in self.queue.items],
                        dropped=len(self.queue.dropped),
                        current=self.job_view(cur, False) if cur else None)

    def config_view(self):
        return dict(self.cfg, target=self.target, host=self.host, vmax=self.vmax,
                    arc_m=self.cyl.arc_m(self.cyl.job_arc))

    def worker(self):
        robot = plotter = None
        while True:
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
                    robot = Robot(**self.robot_kw).connect()
                    self.vmax = robot.limits.tcp_speed_max
                    plotter = Plotter(robot, self.cyl, self.cfg, self.abort, self.report)
                    rng = plotter.setup(self.seed)
                    self.report(connected=True, error="", base_range=rng, step="bereit")
                    self.log(f"verbunden mit {self.host}, Profil '{robot.limits.name}', "
                             f"Basisgelenk {rng['base_start_deg']:.0f}° bis {rng['base_end_deg']:.0f}°")
                    with self.lock:
                        self.bump()
                    if self.secure_now:
                        plotter.recover()
                        self.secure_now = False
                        self.report(step="bereit")
                except Exception as e:
                    if robot is not None:
                        robot.disconnect()
                    robot = plotter = None
                    self.report(connected=False, error=f"Verbindung: {e}", step="keine Verbindung")
                    self.wake.wait(3)
                    self.wake.clear()
                    continue
            with self.lock:
                job = self.queue.pop()
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
        while True:
            try:
                if conn is None:
                    conn = RobotConnection(self.host).connect()
                q = conn.receive.getActualQ()
                pose = conn.receive.getActualTCPPose()
                d = self.cyl.describe_pose(pose, q[0])
                live = {"a": round(d["a"], 2), "h": round(d["h"], 4), "r": round(d["r"], 4),
                        "facing": d["facing"], "gap": None if d["gap"] is None else round(d["gap"], 4),
                        "base_deg": round(math.degrees(q[0]), 1), "t": time.time()}
                with self.lock:
                    self.live = live
                    armed = not self.estop
                bad = reach.below_base(q, self.watch_tool)
                if bad and armed:
                    self.log(f"WÄCHTER: {bad} unter der Basis (Gelenke {[round(math.degrees(v)) for v in q]}°)")
                    self.estop_now(f"Wächter: {bad} unter der Basis")
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
        if path == "/api/estop":
            return self._send(200, {"ok": True, "msg": hub.estop_now()})
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
    args = ap.parse_args()
    cfg = load_config(args.config)
    cyl = Cylinder(cfg)
    roll, problems, seed = reach.choose_roll(cyl)
    if problems:
        raise SystemExit(f"config.json verlangt Posen, die der Arm nicht erreicht (beste Drehung der Lampe {roll:.0f}°):\n  "
                         + "\n  ".join(problems))
    cyl.roll = roll
    kw = robot_kwargs(args)
    if args.target == "real" and args.payload <= 0:
        raise SystemExit("--target real braucht --payload KG mit der gemessenen Masse der Lampe")
    kw["payload_kg"] = args.payload
    kw["tcp"] = tuple(cfg["lamp_tcp"])
    host = kw.get("host") or resolve_host(args.target)
    port = args.port or cfg["port"]
    hub = Hub(cfg, cyl, args.target, host, kw)
    hub.seed = seed
    Handler.hub = hub
    line = "=" * 64
    who = f"SIMULATOR ({host})" if args.target == "sim" else f"!!! ECHTER ROBOTER {host} !!!"
    print(f"{line}\n  ZIEL: {who}\n  Vorschau: http://localhost:{port}/\n"
          f"  Tuch r={cyl.R} m, Lampe {cfg['lamp_distance_m'] * 100:.0f} cm davor, "
          f"Höhe {cyl.h_min:.2f}…{cyl.h_max:.2f} m, Start {cyl.start:.0f}°, {cyl.job_arc:.0f}° pro Auftrag\n"
          f"  Lampe TCP {[round(v, 4) for v in cfg['lamp_tcp']]}, um den Strahl gedreht {cyl.roll:.0f}°\n{line}",
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
