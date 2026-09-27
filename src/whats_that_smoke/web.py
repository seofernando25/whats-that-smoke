from __future__ import annotations

import asyncio, json, math, subprocess, threading, time, uuid
from collections import deque
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
from .heading import RelativeHeading
from .imu import ImuStream
from .motion import GyroMotion, MotionFault, ramp_wheels

STATIC = Path(__file__).parent / "static"
WATCHDOG_SECONDS = .60

@dataclass
class RobotState:
    connected: bool = False
    owner: str | None = None
    armed: bool = False
    forward: float = 0.0
    strafe: float = 0.0
    turn: float = 0.0
    turn_target_dps: float = 0.0
    motion_mode: str = "idle"
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
    aruco_markers: list[dict] = field(default_factory=list)
    aruco_status: str = "aruco-disabled"
    imu_connected: bool = False
    imu_status: str = "starting"
    imu_age_ms: int | None = None
    imu_rate_hz: float = 0.0
    imu_accel_g: dict[str, float] = field(default_factory=lambda: {axis: 0.0 for axis in "xyz"})
    imu_gyro_dps: dict[str, float] = field(default_factory=lambda: {axis: 0.0 for axis in "xyz"})
    heading_deg: float = 0.0
    heading_rate_dps: float = 0.0
    heading_calibrated: bool = False
    heading_stationary: bool = False
    heading_status: str = "hold still · zeroing"
    heading_progress: float = 0.0
    wheels: dict[str, int] = field(default_factory=lambda: {n: 0 for n in WHEEL_CHANNELS})
    stopped: bool = True
    reason: str = "startup"
    revision: int = 0

class CameraStream:
    def __init__(self):
        self.frame: bytes | None = None; self.seq = 0
        self.received_at = 0.0; self.frame_times = deque(maxlen=90)
        self.cv = threading.Condition(); self.stop = threading.Event()
        self.process = None; self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True, name="camera-stream")
        self.thread.start()

    def _run(self):
        cmd = ["rpicam-vid", "--timeout", "0", "--nopreview", "--codec", "mjpeg",
               "--width", "1280", "--height", "720", "--framerate", "30",
               "--quality", "75", "--shutter", "8000", "--gain", "1.5", "--denoise", "cdn_off",
               "--flush", "--output", "-"]
        while not self.stop.is_set():
            try:
                self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
                buf = bytearray()
                while not self.stop.is_set():
                    # BufferedReader.read(n) may wait to fill n bytes; that
                    # batches several small MJPEG frames and adds latency.
                    chunk = self.process.stdout.read1(16384)
                    if not chunk: break
                    buf.extend(chunk)
                    while True:
                        a = buf.find(b"\xff\xd8"); b = buf.find(b"\xff\xd9", a + 2) if a >= 0 else -1
                        if a < 0 or b < 0:
                            if len(buf) > 2_000_000: del buf[:-2]
                            break
                        frame = bytes(buf[a:b + 2]); del buf[:b + 2]
                        with self.cv:
                            self.received_at = time.monotonic(); self.frame_times.append(self.received_at)
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
                if self.seq == seen: continue
                frame, seen = self.frame, self.seq
            if frame:
                yield b"--frame\r\nContent-Type: image/jpeg\r\nCache-Control: no-store\r\n\r\n" + frame + b"\r\n"

    def latest(self) -> tuple[bytes | None, int]:
        with self.cv:
            return self.frame, self.seq

    def latest_sample(self):
        with self.cv:
            return self.frame, self.seq, self.received_at

    def metrics(self):
        with self.cv:
            now = time.monotonic()
            recent = [t for t in self.frame_times if now - t < 3]
            age = now - self.received_at if self.received_at else None
            fps = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 1 and age < 0.5 else 0.0
            return {"camera_fps": round(fps, 1), "camera_frame_age_ms": round(age * 1000) if age is not None else None,
                    "camera_sequence": self.seq}

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
        self.motion = GyroMotion()
        self.motion_task: asyncio.Task | None = None
        self.last_operator_command = 0.0
        self.last_pwm_at = time.monotonic()
        self.imu_sample_at: float | None = None
        self.heading = RelativeHeading()

    def imu_sample(self, acceleration: list[float], gyroscope: list[float], rate_hz: float) -> None:
        self.state.imu_accel_g = dict(zip("xyz", acceleration))
        self.state.imu_gyro_dps = dict(zip("xyz", gyroscope))
        self.state.imu_rate_hz = rate_hz
        self.state.imu_connected = True
        self.state.imu_status = "streaming"
        self.imu_sample_at = time.monotonic()
        estimate = self.heading.update(acceleration, gyroscope, self.imu_sample_at, moving=not self.state.stopped)
        for key, value in estimate.items():
            setattr(self.state, key, value)

    def imu_status(self, connected: bool, status: str) -> None:
        self.state.imu_connected = connected
        self.state.imu_status = status
        if not connected:
            self.state.imu_rate_hz = 0.0
            self.heading.disconnected()
            self.state.heading_status = "imu-disconnected"
            self.state.heading_rate_dps = 0.0

    async def start(self):
        self.bus = SMBus(1); self.bus.read_byte_data(0x40, 0); _configure_pca9685(self.bus); _stop_wheels(self.bus)
        self.state.connected = True
        self.guard_thread = threading.Thread(target=self._guard, daemon=True, name="motor-deadman")
        self.guard_thread.start()
        self.motion_task = asyncio.create_task(self._motion_loop(), name="gyro-drive")

    def _imu_ready(self, now):
        return (self.state.imu_connected and self.state.heading_calibrated
                and self.imu_sample_at is not None and now - self.imu_sample_at <= .15)

    def _brake_locked(self, reason, release=False):
        self.motion.cancel()
        if self.bus: _stop_wheels(self.bus)
        self.state.forward = self.state.strafe = self.state.turn = self.state.turn_target_dps = 0
        self.state.wheels = {n: 0 for n in WHEEL_CHANNELS}
        self.state.stopped = True; self.state.motion_mode = "idle"
        self.state.reason = reason; self.state.revision += 1
        if release:
            self.state.armed = False; self.state.owner = None

    def _guard_check(self, now):
        with self.hw_lock:
            if not self.state.armed: return
            reason = None
            if self.motion.mode and now - self.last_operator_command > WATCHDOG_SECONDS:
                reason = "input-watchdog"
            elif self.motion.requires_gyro and not self._imu_ready(now):
                reason = "gyro-unavailable"
            elif not self.state.stopped and now - self.last_drive > WATCHDOG_SECONDS:
                reason = "watchdog"
            if reason: self._brake_locked(reason, True)

    def _guard(self):
        while not self.guard_stop.wait(.05):
            self._guard_check(time.monotonic())

    async def _motion_loop(self):
        broadcast_at = 0.0
        while True:
            await asyncio.sleep(.02)
            async with self.lock:
                with self.hw_lock:
                    if not self.motion.mode or not self.state.armed: continue
                    now = time.monotonic()
                    try:
                        output = self.motion.step(now, self.state.heading_deg,
                                                  self.state.heading_rate_dps, self._imu_ready(now))
                        if output is None: continue
                        self.state.motion_mode = self.motion.mode
                        self.state.turn_target_dps = output.target_rate
                        self._apply_vector(output.forward, output.turn, output.limit, output.reason,
                                           smooth=bool(output.forward or output.turn))
                    except MotionFault as error:
                        self._brake_locked(str(error), True)
                    except Exception:
                        self._brake_locked("motor-control-error", True)
            if now - broadcast_at >= .10:
                broadcast_at = now
                await self.broadcast()

    async def close(self):
        if self.motion_task:
            self.motion_task.cancel()
            try: await self.motion_task
            except asyncio.CancelledError: pass
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

    def _apply_vector(self, forward: float, turn: float, limit: int, reason: str = "drive", smooth=False) -> None:
        motor_forward = -forward
        left, right = motor_forward + turn, motor_forward - turn
        scale = max(1.0, abs(left), abs(right))
        wheels = {
            "front-left": round(left / scale * limit), "rear-left": round(left / scale * limit),
            "front-right": round(right / scale * limit), "rear-right": round(right / scale * limit),
        }
        with self.hw_lock:
            if not self.state.armed: return
            now = time.monotonic()
            if smooth: wheels = ramp_wheels(self.state.wheels, wheels, now - self.last_pwm_at)
            self._write(wheels); self.last_drive = self.last_pwm_at = now
            self.state.forward = forward; self.state.turn = turn; self.state.speed_limit = limit; self.state.wheels = wheels
            self.state.stopped = not any(wheels.values()); self.state.reason = "command-zero" if self.state.stopped else reason; self.state.revision += 1

    async def sidestep(self, cid: str, direction: int, limit: int) -> None:
        async with self.lock:
            if self.state.owner != cid or not self.state.armed: raise PermissionError("arm controls first")
            if aruco.follow: raise PermissionError("disable ArUco follow before side-step")
            direction = max(-1, min(1, int(direction))); limit = max(500, min(1800, int(limit)))
            with self.hw_lock:
                now = time.monotonic()
                if not direction:
                    self._brake_locked("key-release")
                elif not self._imu_ready(now):
                    self._brake_locked("gyro-unavailable", True)
                    raise PermissionError("gyro must be connected and calibrated")
                else:
                    self.last_operator_command = now
                    self.motion.sidestep(direction, limit, now, self.state.heading_deg)
                    self.state.strafe = float(direction)
        await self.broadcast()

    async def drive(self, cid, forward, strafe, turn, limit, autonomous=False):
        async with self.lock:
            if self.state.owner != cid or not self.state.armed: raise PermissionError("arm controls first")
            if aruco.follow and not autonomous: raise PermissionError("disable ArUco follow before manual drive")
            if not all(math.isfinite(value) for value in (forward, strafe, turn)): raise ValueError
            forward = max(-1., min(1., forward)); strafe = max(-1., min(1., strafe)); turn = max(-1., min(1., turn)); limit = max(500, min(1800, limit))
            if strafe: raise ValueError("use side-step command for ordinary wheels")
            with self.hw_lock:
                now = time.monotonic()
                self.state.strafe = 0.0
                if autonomous:
                    self.motion.cancel()
                    self.state.motion_mode = "aruco-follow"
                    self.state.turn_target_dps = 0
                    self._apply_vector(forward, turn, limit)
                elif not forward and not turn:
                    self._brake_locked("key-release")
                elif turn and not self._imu_ready(now):
                    self._brake_locked("gyro-unavailable", True)
                    raise PermissionError("gyro must be connected and calibrated")
                else:
                    self.last_operator_command = now
                    self.motion.drive(forward, turn, limit, now)
        await self.broadcast()

    async def stop(self, reason, release=False):
        async with self.lock:
            with self.hw_lock:
                self._brake_locked(reason, release)
        await self.broadcast()

    async def disconnect(self, cid):
        self.clients.pop(cid, None)
        if self.state.owner == cid: await self.stop("controller-disconnected", True)
        else: await self.broadcast()

    def payload(self, cid=None):
        self.state.imu_age_ms = round((time.monotonic() - self.imu_sample_at) * 1000) if self.imu_sample_at else None
        p = asdict(self.state); p.update(clients=len(self.clients), you_are_owner=bool(cid and self.state.owner == cid), watchdog_ms=600)
        p.update(camera.metrics()); p.update(aruco.metrics())
        p["motion_input_age_ms"] = round((time.monotonic() - self.last_operator_command) * 1000) if self.motion.mode else None
        return {"type": "state", "state": p}

    async def broadcast(self):
        dead = []
        for cid, ws in list(self.clients.items()):
            try: await ws.send_json(self.payload(cid))
            except Exception: dead.append(cid)
        for cid in dead: self.clients.pop(cid, None)

controller = RobotController(); camera = CameraStream(); aruco = ArucoFollower(camera, controller)
imu = ImuStream(controller.imu_sample, controller.imu_status)
app = FastAPI(title="What's That Smoke Control", docs_url=None, redoc_url=None)
app.mount("/static", StaticFiles(directory=STATIC), name="static")

@app.on_event("startup")
async def startup(): await controller.start(); camera.start(); aruco.start(); imu.start()
@app.on_event("shutdown")
async def shutdown(): await aruco.close(); imu.close(); camera.close(); await controller.close()
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
                if kind == "drive": await controller.drive(cid, float(m.get("forward",0)), float(m.get("strafe",0)), float(m.get("turn",0)), int(m.get("speed_limit",1800)))
                elif kind == "sidestep": await controller.sidestep(cid, int(m.get("direction",0)), int(m.get("speed_limit",1200)))
                elif kind == "arm": await controller.arm(cid)
                elif kind == "camera": await controller.camera_move(cid, str(m.get("axis")), int(m.get("delta", 0)))
                elif kind == "aruco": await aruco.configure(cid, bool(m.get("enabled")), m.get("follow"))
                elif kind == "heading-calibrate": controller.heading.request_calibration()
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
