from __future__ import annotations

import asyncio, json, math, subprocess, threading, time, uuid
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Iterator
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from smbus2 import SMBus
from . import SERVO_CHANNELS, WHEEL_CHANNELS, _configure_pca9685, _set_pulse, _set_pwm, _stop_wheels
from .aruco import ArucoFollower

STATIC = Path(__file__).parent / "static"
WATCHDOG_SECONDS = .60

@dataclass
class RobotState:
    connected: bool = False
    owner: str | None = None
    armed: bool = False
    forward: float = 0.0
    turn: float = 0.0
    speed_limit: int = 1800
    pan_us: int = 1500
    tilt_us: int = 1500
    aruco_enabled: bool = False
    aruco_follow: bool = False
    aruco_visible: bool = False
    aruco_id: int | None = None
    aruco_distance_m: float | None = None
    aruco_error_x: float | None = None
    aruco_corners: list[list[float]] = field(default_factory=list)
    aruco_status: str = "aruco-disabled"
    wheels: dict[str, int] = field(default_factory=lambda: {n: 0 for n in WHEEL_CHANNELS})
    stopped: bool = True
    reason: str = "startup"
    revision: int = 0

class CameraStream:
    def __init__(self):
        self.frame: bytes | None = None; self.seq = 0
        self.cv = threading.Condition(); self.stop = threading.Event()
        self.process = None; self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True, name="camera-stream")
        self.thread.start()

    def _run(self):
        cmd = ["rpicam-vid", "--timeout", "0", "--nopreview", "--codec", "mjpeg",
               "--width", "640", "--height", "480", "--framerate", "20",
               "--quality", "75", "--rotation", "180", "--flush", "--output", "-"]
        while not self.stop.is_set():
            try:
                self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
                buf = bytearray()
                while not self.stop.is_set():
                    chunk = self.process.stdout.read(16384)
                    if not chunk: break
                    buf.extend(chunk)
                    while True:
                        a = buf.find(b"\xff\xd8"); b = buf.find(b"\xff\xd9", a + 2) if a >= 0 else -1
                        if a < 0 or b < 0:
                            if len(buf) > 2_000_000: del buf[:-2]
                            break
                        frame = bytes(buf[a:b + 2]); del buf[:b + 2]
                        with self.cv:
                            self.frame = frame; self.seq += 1; self.cv.notify_all()
            except Exception: pass
            finally:
                if self.process:
                    self.process.kill(); self.process.wait(); self.process = None
            self.stop.wait(1)

    def frames(self) -> Iterator[bytes]:
        seen = -1
        while not self.stop.is_set():
            with self.cv:
                self.cv.wait_for(lambda: self.seq != seen or self.stop.is_set(), timeout=2)
                if self.stop.is_set(): return
                frame, seen = self.frame, self.seq
            if frame:
                yield b"--frame\r\nContent-Type: image/jpeg\r\nCache-Control: no-store\r\n\r\n" + frame + b"\r\n"

    def latest(self) -> tuple[bytes | None, int]:
        with self.cv:
            return self.frame, self.seq

    def close(self):
        self.stop.set()
        with self.cv: self.cv.notify_all()
        if self.process: self.process.terminate()
        if self.thread: self.thread.join(timeout=3)

class RobotController:
    def __init__(self):
        self.state = RobotState(); self.bus = None
        self.lock = asyncio.Lock(); self.hw_lock = threading.RLock()
        self.clients = {}; self.last_drive = 0.0
        self.guard_stop = threading.Event(); self.guard_thread = None

    async def start(self):
        self.bus = SMBus(1); self.bus.read_byte_data(0x40, 0); _configure_pca9685(self.bus); _stop_wheels(self.bus)
        self.state.connected = True
        self.guard_thread = threading.Thread(target=self._guard, daemon=True, name="motor-deadman")
        self.guard_thread.start()

    def _guard(self):
        while not self.guard_stop.wait(.05):
            if self.state.armed and not self.state.stopped and time.monotonic() - self.last_drive > WATCHDOG_SECONDS:
                with self.hw_lock:
                    if self.bus: _stop_wheels(self.bus)
                self.state.forward = self.state.turn = 0
                self.state.wheels = {n: 0 for n in WHEEL_CHANNELS}
                self.state.stopped = True; self.state.armed = False; self.state.owner = None
                self.state.reason = "watchdog"; self.state.revision += 1

    async def close(self):
        self.guard_stop.set()
        if self.guard_thread: self.guard_thread.join(timeout=1)
        await self.stop("shutdown", True)
        if self.bus: self.bus.close(); self.bus = None

    async def arm(self, cid):
        async with self.lock:
            if self.state.owner not in (None, cid): raise PermissionError("controller busy")
            self.state.owner = cid; self.state.armed = True; self.state.reason = "armed"; self.state.revision += 1
        await self.broadcast()

    async def camera_move(self, cid, axis, delta):
        async with self.lock:
            if self.state.owner != cid or not self.state.armed: raise PermissionError("arm controls first")
            if axis not in SERVO_CHANNELS: raise ValueError
            delta = max(-50, min(50, int(delta)))
            attr = "pan_us" if axis == "pan" else "tilt_us"
            pulse = max(1000, min(2000, getattr(self.state, attr) + delta))
            with self.hw_lock:
                if not self.bus: raise RuntimeError("I2C unavailable")
                _set_pulse(self.bus, SERVO_CHANNELS[axis], pulse)
            setattr(self.state, attr, pulse); self.state.reason = f"camera-{axis}"; self.state.revision += 1
        await self.broadcast()

    def _write(self, wheels):
        if not self.bus: raise RuntimeError("I2C unavailable")
        with self.hw_lock:
            for name, duty in wheels.items():
                rev, fwd = WHEEL_CHANNELS[name]
                if duty > 0: _set_pwm(self.bus, rev, 0); _set_pwm(self.bus, fwd, duty)
                elif duty < 0: _set_pwm(self.bus, fwd, 0); _set_pwm(self.bus, rev, abs(duty))
                else: _set_pwm(self.bus, rev, 4095); _set_pwm(self.bus, fwd, 4095)

    async def drive(self, cid, forward, turn, limit, autonomous=False):
        async with self.lock:
            if self.state.owner != cid or not self.state.armed: raise PermissionError("arm controls first")
            if aruco.follow and not autonomous: raise PermissionError("disable ArUco follow before manual drive")
            if not math.isfinite(forward) or not math.isfinite(turn): raise ValueError
            forward = max(-1., min(1., forward)); turn = max(-1., min(1., turn)); limit = max(500, min(1800, limit))
            motor_forward = -forward
            left, right = motor_forward + turn, motor_forward - turn; scale = max(1., abs(left), abs(right)); left /= scale; right /= scale
            wheels = {"front-left": round(left*limit), "rear-left": round(left*limit), "front-right": round(right*limit), "rear-right": round(right*limit)}
            self._write(wheels); self.last_drive = time.monotonic()
            self.state.forward = forward; self.state.turn = turn; self.state.speed_limit = limit; self.state.wheels = wheels
            self.state.stopped = not any(wheels.values()); self.state.reason = "command-zero" if self.state.stopped else "drive"; self.state.revision += 1
        await self.broadcast()

    async def stop(self, reason, release=False):
        async with self.lock:
            with self.hw_lock:
                if self.bus: _stop_wheels(self.bus)
            self.state.forward = self.state.turn = 0; self.state.wheels = {n: 0 for n in WHEEL_CHANNELS}
            self.state.stopped = True; self.state.reason = reason; self.state.revision += 1
            if release: self.state.owner = None; self.state.armed = False
        await self.broadcast()

    async def disconnect(self, cid):
        self.clients.pop(cid, None)
        if self.state.owner == cid: await self.stop("controller-disconnected", True)
        else: await self.broadcast()

    def payload(self, cid=None):
        p = asdict(self.state); p.update(clients=len(self.clients), you_are_owner=bool(cid and self.state.owner == cid), watchdog_ms=600)
        return {"type": "state", "state": p}

    async def broadcast(self):
        dead = []
        for cid, ws in list(self.clients.items()):
            try: await ws.send_json(self.payload(cid))
            except Exception: dead.append(cid)
        for cid in dead: self.clients.pop(cid, None)

controller = RobotController(); camera = CameraStream(); aruco = ArucoFollower(camera, controller)
app = FastAPI(title="What's That Smoke Control", docs_url=None, redoc_url=None)
app.mount("/static", StaticFiles(directory=STATIC), name="static")

@app.on_event("startup")
async def startup(): await controller.start(); camera.start(); aruco.start()
@app.on_event("shutdown")
async def shutdown(): await aruco.close(); camera.close(); await controller.close()
@app.get("/")
async def index(): return FileResponse(STATIC / "index.html")
@app.get("/stream.mjpg")
def stream():
    return StreamingResponse(camera.frames(), media_type="multipart/x-mixed-replace; boundary=frame", headers={"Cache-Control":"no-store", "X-Accel-Buffering":"no"})
@app.get("/api/state")
async def state(): return controller.payload()

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept(); cid = uuid.uuid4().hex[:8]; controller.clients[cid] = ws
    await ws.send_json(controller.payload(cid)); await controller.broadcast()
    try:
        while True:
            try:
                m = json.loads(await ws.receive_text()); kind = m.get("type")
                if kind == "drive": await controller.drive(cid, float(m.get("forward",0)), float(m.get("turn",0)), int(m.get("speed_limit",1800)))
                elif kind == "arm": await controller.arm(cid)
                elif kind == "camera": await controller.camera_move(cid, str(m.get("axis")), int(m.get("delta", 0)))
                elif kind == "aruco": await aruco.configure(cid, bool(m.get("enabled")), m.get("follow"))
                elif kind == "heartbeat": await ws.send_json(controller.payload(cid))
                elif kind == "stop" and controller.state.owner in (None,cid): await controller.stop("client-stop", True)
                else: await ws.send_json({"type":"error","error":"unknown message"})
            except PermissionError as e: await ws.send_json({"type":"error","error":str(e)})
            except (TypeError, ValueError, json.JSONDecodeError): await ws.send_json({"type":"error","error":"invalid message"})
    except WebSocketDisconnect: pass
    finally:
        if aruco.owner == cid: await aruco.disable_follow("aruco-owner-disconnected")
        await controller.disconnect(cid)

def main(): uvicorn.run("whats_that_smoke.web:app", host="0.0.0.0", port=8765, reload=False)
