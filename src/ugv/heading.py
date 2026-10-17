"""Relative, gyro-integrated heading for the chassis-mounted Nano IMU."""

from __future__ import annotations

import math
import threading


class RelativeHeading:
    """Estimate clockwise yaw from gyro samples; zero is the calibration pose."""

    CALIBRATION_SECONDS = 3.0
    REST_SECONDS = 0.5
    ACCEL_NORM_TOLERANCE_G = 0.10
    QUIET_GYRO_DPS = 4.0
    MAX_INTEGRATION_DT = 0.15

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._calibration_requested = True
        self._samples: list[tuple[float, tuple[float, ...], tuple[float, ...]]] = []
        self._last_time: float | None = None
        self._rest_since: float | None = None
        self._bias = (0.0, 0.0, 0.0)
        self._up = (0.0, 0.0, 1.0)
        self._heading = 0.0
        self._rate = 0.0
        self._calibrated = False
        self._stationary = False
        self._status = "hold still · zeroing"
        self._progress = 0.0

    def request_calibration(self) -> None:
        with self._lock:
            self._calibration_requested = True
            self._calibrated = False
            self._stationary = False
            self._samples.clear()
            self._rest_since = None
            self._progress = 0.0
            self._status = "hold still · zeroing"

    def disconnected(self) -> None:
        with self._lock:
            self._last_time = None
            self._rest_since = None
            self._stationary = False
            self._rate = 0.0
            self._calibrated = False
            self._calibration_requested = True
            self._samples.clear()
            self._progress = 0.0
            self._status = "imu-disconnected"

    @staticmethod
    def _norm(vector: tuple[float, ...]) -> float:
        return math.sqrt(sum(value * value for value in vector))

    @staticmethod
    def _mean(vectors: list[tuple[float, ...]]) -> tuple[float, ...]:
        count = len(vectors)
        return tuple(sum(vector[i] for vector in vectors) / count for i in range(3))

    @staticmethod
    def _max_std(vectors: list[tuple[float, ...]]) -> float:
        mean = RelativeHeading._mean(vectors)
        return max(
            math.sqrt(sum((vector[i] - mean[i]) ** 2 for vector in vectors) / len(vectors))
            for i in range(3)
        )

    def _snapshot(self) -> dict[str, float | bool | str]:
        return {
            "heading_deg": self._heading,
            "heading_rate_dps": self._rate,
            "heading_calibrated": self._calibrated,
            "heading_stationary": self._stationary,
            "heading_status": self._status,
            "heading_progress": self._progress,
        }

    def snapshot(self) -> dict[str, float | bool | str]:
        with self._lock:
            return self._snapshot()

    def update(
        self,
        acceleration_g: list[float],
        gyro_dps: list[float],
        now: float,
        moving: bool,
    ) -> dict[str, float | bool | str]:
        with self._lock:
            if len(acceleration_g) != 3 or len(gyro_dps) != 3:
                return self._snapshot()
            accel = tuple(float(value) for value in acceleration_g)
            gyro = tuple(float(value) for value in gyro_dps)
            if not all(math.isfinite(value) for value in (*accel, *gyro)):
                return self._snapshot()

            dt = now - self._last_time if self._last_time is not None else 0.0
            self._last_time = now
            accel_norm = self._norm(accel)
            gyro_norm = self._norm(gyro)
            quiet = (
                not moving
                and abs(accel_norm - 1.0) <= self.ACCEL_NORM_TOLERANCE_G
                and gyro_norm <= self.QUIET_GYRO_DPS
            )

            if dt > self.MAX_INTEGRATION_DT:
                self._samples.clear()
                self._progress = 0.0
                self._rest_since = None
                self._stationary = False
                self._rate = 0.0
                self._status = "sample gap · heading held" if self._calibrated else "hold still · zeroing"
                return self._snapshot()

            if self._calibration_requested:
                if quiet:
                    if not self._samples:
                        self._samples.append((now, accel, gyro))
                    else:
                        self._samples.append((now, accel, gyro))
                    elapsed = now - self._samples[0][0]
                    self._progress = min(1.0, elapsed / self.CALIBRATION_SECONDS)
                    self._status = f"zeroing · hold still {elapsed:.1f}/3.0s"
                    if elapsed >= self.CALIBRATION_SECONDS:
                        accels = [sample[1] for sample in self._samples]
                        gyros = [sample[2] for sample in self._samples]
                        gravity = self._mean(accels)
                        gravity_norm = self._norm(gravity)
                        if (
                            self._max_std(gyros) > 1.5
                            or self._max_std(accels) > 0.025
                            or not 0.9 < gravity_norm < 1.1
                        ):
                            self._samples.clear()
                            self._progress = 0.0
                            self._status = "movement detected · hold still"
                        else:
                            self._bias = self._mean(gyros)
                            self._up = tuple(value / gravity_norm for value in gravity)
                            self._heading = 0.0
                            self._rate = 0.0
                            self._calibrated = True
                            self._calibration_requested = False
                            self._stationary = True
                            self._rest_since = now
                            self._progress = 1.0
                            self._status = "relative · ready"
                else:
                    self._samples.clear()
                    self._progress = 0.0
                    self._status = "stop car · hold still 3s" if moving else "hold still · zeroing"
                return self._snapshot()

            if dt <= 0.0:
                self._rate = 0.0
                self._rest_since = None
                self._stationary = False
                self._status = "sample gap · heading held" if self._calibrated else "hold still · zeroing"
                return self._snapshot()

            if quiet:
                if self._rest_since is None:
                    self._rest_since = now
                self._stationary = now - self._rest_since >= self.REST_SECONDS
            else:
                self._rest_since = None
                self._stationary = False

            if self._stationary:
                # Slowly follow true zero-rate bias only after a half-second of rest.
                alpha = 1.0 - math.exp(-dt / 2.0)
                self._bias = tuple(old + alpha * (value - old) for old, value in zip(self._bias, gyro))

            # Positive displayed heading is clockwise, like a game compass.
            self._rate = -sum((value - bias) * up for value, bias, up in zip(gyro, self._bias, self._up))
            if self._stationary and abs(self._rate) < 0.35:
                self._rate = 0.0
            self._heading = (self._heading + self._rate * dt) % 360.0
            self._status = "relative · rest" if self._stationary else "relative · tracking"
            return self._snapshot()
