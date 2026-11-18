from __future__ import annotations

import threading
import time

CLIENT_TIMEOUT = 1.5


class LampError(Exception):
    pass


class Lamp:
    def __init__(self, cfg: dict, required: bool):
        self.required = required
        # dry: movement test without light. The lamp is never switched on (DMX gets
        # "off" even when connected); jobs don't wait for DMX.
        self.dry = False
        self.timeout = cfg["lamp_switch_timeout_s"]
        self.cond = threading.Condition()
        self.on = False
        self.seq = 0
        self.acked = 0
        self.seen = 0.0
        self.device = ""
        self.lit_by_dmx = False

    def connected(self) -> bool:
        return bool(self.device) and time.time() - self.seen < CLIENT_TIMEOUT

    def set(self, on: bool, wait: bool = True) -> bool:
        with self.cond:
            self.on = on
            self.seq += 1
            s = self.seq
            self.cond.notify_all()
            if self.dry:
                self.lit_by_dmx = False
                return False
            if not self.connected():
                if on and self.required:
                    raise LampError("DMX nicht verbunden: Lampe kann nicht eingeschaltet werden")
                if not on:
                    self.lit_by_dmx = False
                return False
            if not wait:
                return False
            end = time.time() + self.timeout
            while self.acked < s:
                left = end - time.time()
                if left <= 0:
                    raise LampError(f"DMX hat das {'Einschalten' if on else 'Ausschalten'} nicht bestätigt")
                self.cond.wait(min(left, 0.05))
            self.lit_by_dmx = on
            return True

    def poll(self, since: int, device: str, timeout: float = 1.0) -> dict:
        end = time.time() + timeout
        with self.cond:
            self.device = device
            self.seen = time.time()
            while self.seq == since and time.time() < end:
                self.cond.wait(min(0.2, end - time.time()))
                self.seen = time.time()
            return {"seq": self.seq, "on": self.on and not self.dry}

    def ack(self, seq: int, device: str) -> None:
        with self.cond:
            self.device = device
            self.seen = time.time()
            if self.acked < seq <= self.seq:
                self.acked = seq
            self.cond.notify_all()

    def release(self) -> None:
        with self.cond:
            self.device = ""
            self.cond.notify_all()

    def lost_while_on(self) -> bool:
        with self.cond:
            return self.lit_by_dmx and not self.connected()

    def view(self) -> dict:
        with self.cond:
            return {"control": "dmx", "on": self.on, "confirmed": self.acked >= self.seq,
                    "dmx": self.connected(), "device": self.device if self.connected() else "",
                    "required": self.required, "dry": self.dry}
