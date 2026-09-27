"""Hardware-independent manual motion controller. Positive turn = CCW/left."""
from __future__ import annotations

from dataclasses import dataclass
import math


def clamp(value, low, high):
    return max(low, min(high, value))


def angle_error(target, actual):
    return (target - actual + 180.0) % 360.0 - 180.0


class MotionFault(RuntimeError):
    pass


@dataclass
class Output:
    forward: float
    turn: float
    limit: int
    reason: str
    target_rate: float = 0.0  # clockwise deg/s, same convention as dashboard


class GyroMotion:
    LEASE_SECONDS = .45
    TURN_RATE = 40.0
    TURN_LIMIT = 1800  # Respect the user's slider, not the former hidden 1500 cap.
    ARC_CORRECTION = .65
    SIDE_ANGLE = 12.0
    LEG_SECONDS = .35  # Displacement remains unmeasured without odometry.

    def __init__(self):
        self.cancel()

    def cancel(self):
        self.mode = None
        self.forward = self.turn = 0.0
        self.direction = 0
        self.limit = 1000
        self.renewed_at = 0.0
        self.previous_at = None
        self.rate_setpoint = self.integral = 0.0
        self.stall_since = self.wrong_since = None
        self.phase = 0
        self.phase_at = self.origin = 0.0
        self.settled_at = None

    @property
    def requires_gyro(self):
        return self.mode == "sidestep" or (self.mode == "drive" and bool(self.turn))

    def drive(self, forward, turn, limit, now):
        if self.mode != "drive":
            self.cancel()
        self.mode = "drive"
        self.forward, self.turn = forward, turn
        self.limit, self.renewed_at = limit, now

    def sidestep(self, direction, limit, now, heading):
        if self.mode != "sidestep" or self.direction != direction:
            self.cancel()
            self.mode, self.direction = "sidestep", direction
            self.origin, self.phase_at = heading, now
        self.limit, self.renewed_at = limit, now

    def _advance(self, now):
        self.phase = (self.phase + 1) % 5
        self.phase_at = now
        self.settled_at = None
        self.rate_setpoint = self.integral = 0.0
        self.stall_since = self.wrong_since = None

    def _yaw(self, target_ccw, measured_clockwise, dt, now, pivot=False):
        self.rate_setpoint += clamp(target_ccw - self.rate_setpoint, -90 * dt, 90 * dt)
        measured = -measured_clockwise
        error = self.rate_setpoint - measured
        proposed = clamp(self.integral + .003 * error * dt, -.18, .18)
        raw = .60 * self.rate_setpoint / self.TURN_RATE + .008 * error + proposed
        if abs(raw) <= 1 or raw * error < 0:
            self.integral = proposed
        power = clamp(.60 * self.rate_setpoint / self.TURN_RATE + .008 * error + self.integral, -1, 1)
        # Break static tire scrub only when measured yaw is nearly stationary.
        # PWM ramp still applies; never exceed the selected speed limit.
        # Drop boost immediately when rotating; do not boost braking/reversal.
        if pivot and abs(measured) < 3 and abs(self.rate_setpoint) > 2 and power * self.rate_setpoint > 0:
            power = math.copysign(max(abs(power), .90), power)
        stalled = abs(self.rate_setpoint) > 12 and abs(measured) < 3 and abs(power) > .35
        self.stall_since = (now if self.stall_since is None else self.stall_since) if stalled else None
        wrong = abs(self.rate_setpoint) > 12 and abs(measured) > 6 and measured * self.rate_setpoint < 0
        self.wrong_since = (now if self.wrong_since is None else self.wrong_since) if wrong else None
        if self.stall_since is not None and now - self.stall_since > 1.2:
            raise MotionFault("turn-no-response")
        if self.wrong_since is not None and now - self.wrong_since > .30:
            raise MotionFault("gyro-direction-mismatch")
        return power

    def step(self, now, heading, rate, imu_ready):
        if self.mode is None:
            return None
        if now - self.renewed_at > self.LEASE_SECONDS:
            raise MotionFault("input-expired")
        if self.requires_gyro and (not imu_ready or not math.isfinite(heading) or not math.isfinite(rate)):
            raise MotionFault("gyro-unavailable")
        dt = .02 if self.previous_at is None else clamp(now - self.previous_at, 0, .05)
        self.previous_at = now
        if self.mode == "drive":
            if not self.turn:
                self.rate_setpoint = self.integral = 0.0
                self.stall_since = self.wrong_since = None
                return Output(self.forward, 0, self.limit, "drive-ramped")
            power = self._yaw(self.turn * self.TURN_RATE, rate, dt, now, pivot=not self.forward)
            if self.forward:
                # Both sides keep the travel direction: a rolling arc, not a pivot.
                power = clamp(power, -self.ARC_CORRECTION * abs(self.forward), self.ARC_CORRECTION * abs(self.forward))
            return Output(self.forward, power, min(self.limit, self.TURN_LIMIT), "gyro-turn", -self.rate_setpoint)

        # Heading targets are relative to the starting orientation, not time.
        # Turn left, move forward, turn right, reverse, restore heading.
        targets = (self.origin - self.direction * self.SIDE_ANGLE,
                   self.origin - self.direction * self.SIDE_ANGLE,
                   self.origin + self.direction * self.SIDE_ANGLE,
                   self.origin + self.direction * self.SIDE_ANGLE, self.origin)
        error_clockwise = angle_error(targets[self.phase], heading)
        elapsed = now - self.phase_at
        limit = min(self.limit, self.TURN_LIMIT)
        if self.phase in (0, 2, 4):
            if elapsed > 3:
                raise MotionFault("sidestep-turn-timeout")
            if abs(error_clockwise) <= 2:
                if abs(rate) <= 5:
                    self.settled_at = now if self.settled_at is None else self.settled_at
                    if now - self.settled_at >= .08:
                        self._advance(now)
                else:
                    self.settled_at = None
                return Output(0, 0, limit, "sidestep-settle")
            self.settled_at = None
            desired = -clamp(error_clockwise * 3, -30, 30)
            turn = self._yaw(desired, rate, dt, now, pivot=True)
            return Output(0, turn, limit, "sidestep-turn", -self.rate_setpoint)
        if elapsed >= self.LEG_SECONDS:
            if elapsed >= self.LEG_SECONDS + .12:
                self._advance(now)
            return Output(0, 0, limit, "sidestep-brake")
        if abs(error_clockwise) > 10:
            raise MotionFault("sidestep-heading-lost")
        desired = -clamp(error_clockwise * 3, -20, 20)
        turn = self._yaw(desired, rate, dt, now)
        return Output(.40 if self.phase == 1 else -.40, turn, limit, "sidestep-travel", -self.rate_setpoint)


def ramp_wheels(previous, requested, dt, per_second=3500.0):
    """Bound PWM changes; direction reversal must first pass through zero."""
    delta = per_second * clamp(dt, 0, .05)
    result = {}
    for name, target in requested.items():
        old = previous.get(name, 0)
        if old * target < 0:
            target = 0
        result[name] = round(old + clamp(target - old, -delta, delta))
    return result
