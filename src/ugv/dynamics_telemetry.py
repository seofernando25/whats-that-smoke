"""Bounded, read-only raw IMU/control history for dynamics identification."""
from collections import deque
import math
import threading


class DynamicsTelemetry:
    def __init__(self, capacity=3000):
        self.samples = deque(maxlen=capacity)
        self.sequence = 0
        self.lock = threading.Lock()

    def append(self, now, acceleration, gyro, state):
        if not all(math.isfinite(v) for v in [now, *acceleration, *gyro]):
            return
        with self.lock:
            self.sequence += 1
            self.samples.append(dict(seq=self.sequence, t=now, accel_g=list(acceleration),
                gyro_dps=list(gyro), forward=state.forward, turn=state.turn,
                pwm_limit=state.speed_limit, wheels=dict(state.wheels),
                stopped=state.stopped, armed=state.armed, revision=state.revision))

    def snapshot(self, after=0):
        with self.lock:
            first = self.samples[0]['seq'] if self.samples else self.sequence+1
            return dict(clock='Pi monotonic seconds', latest_seq=self.sequence,
                        dropped=max(0, first-after-1) if after else 0,
                        samples=[s for s in self.samples if s['seq'] > after])
