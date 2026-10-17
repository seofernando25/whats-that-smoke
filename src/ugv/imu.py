from __future__ import annotations

import glob
import json
import threading
import time
from collections import deque
from typing import Callable

import serial

NANO_PATH = "/dev/serial/by-id/usb-Arduino_Nano_33_BLE_*-if00"


def parse_imu_line(line: bytes) -> tuple[list[float], list[float]] | None:
    try:
        message = json.loads(line)
        acceleration = [float(value) for value in message["a"]]
        gyroscope = [float(value) for value in message["g"]]
        if message.get("type") != "imu" or len(acceleration) != 3 or len(gyroscope) != 3:
            return None
        return acceleration, gyroscope
    except (KeyError, TypeError, ValueError, json.JSONDecodeError, UnicodeDecodeError):
        return None


class ImuStream:
    def __init__(self, on_sample: Callable[[list[float], list[float], float], None], on_status: Callable[[bool, str], None]):
        self.on_sample = on_sample
        self.on_status = on_status
        self.stop = threading.Event()
        self.thread: threading.Thread | None = None

    def start(self) -> None:
        self.thread = threading.Thread(target=self._run, daemon=True, name="nano-imu")
        self.thread.start()

    def _run(self) -> None:
        samples: deque[float] = deque(maxlen=100)
        while not self.stop.is_set():
            paths = glob.glob(NANO_PATH)
            if not paths:
                self.on_status(False, "nano-not-found")
                self.stop.wait(1)
                continue
            try:
                self.on_status(False, "connecting")
                with serial.Serial(paths[0], 115200, timeout=1) as port:
                    port.reset_input_buffer()
                    while not self.stop.is_set():
                        parsed = parse_imu_line(port.readline())
                        if not parsed:
                            continue
                        now = time.monotonic()
                        samples.append(now)
                        cutoff = now - 1.0
                        while samples and samples[0] < cutoff:
                            samples.popleft()
                        self.on_sample(*parsed, float(len(samples)))
            except (OSError, serial.SerialException) as error:
                self.on_status(False, f"serial-error:{type(error).__name__}")
                self.stop.wait(1)

    def close(self) -> None:
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=2)
