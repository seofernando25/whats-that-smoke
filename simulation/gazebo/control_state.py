"""Bounded keyboard commands and an expiring client heartbeat, independent of Gazebo."""
import math
import threading
import time

KEYS = {'KeyW', 'KeyA', 'KeyS', 'KeyD', 'KeyQ', 'KeyE'}

class ControlState:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.RLock()
        self.deadline = 0
        self.linear = self.angular = 0.0
        self.pwm = None
        self.left_duty = self.right_duty = 0.0
        self.owner = None
        self.sequences = {}

    def drive(self, client, sequence, keys, speed, pwm=None):
        if not isinstance(client, str) or not 1 <= len(client) <= 80:
            raise ValueError('Invalid controller ID')
        if type(sequence) is not int or sequence < 0:
            raise ValueError('Invalid sequence')
        if not isinstance(keys, list) or len(keys) > 6 or any(k not in KEYS for k in keys):
            raise ValueError('Invalid driving keys')
        if isinstance(speed, bool) or not isinstance(speed, (int, float)) or not math.isfinite(speed):
            raise ValueError('Invalid speed')
        if pwm is not None and (type(pwm) is not int or not 500 <= pwm <= 1800):
            raise ValueError("PWM must be an integer from 500 to 1800")
        speed = max(.03, min(.3, speed))
        with self.lock:
            if sequence <= self.sequences.get(client, -1):
                return False
            if len(self.sequences) >= 32 and client not in self.sequences:
                raise ValueError('Too many controller sessions; restart the panel server')
            # An idle tab may send a release when the simulation pauses or
            # reconnects. It must not claim ownership and block a driving tab.
            if not keys:
                self.sequences[client] = sequence
                if self.owner in (None, client):
                    self.linear = self.angular = 0.0
                    self.left_duty = self.right_duty = 0.0
                    self.deadline = 0
                    self.owner = None
                return True
            if self.owner not in (None, client) and self.clock() < self.deadline:
                raise ValueError('Another panel is controlling the car')
            self.sequences[client] = sequence
            self.owner = client
            forward = int('KeyW' in keys) - int('KeyS' in keys)
            pivot = int('KeyQ' in keys) - int('KeyE' in keys)
            steering = int('KeyA' in keys) - int('KeyD' in keys)
            self.pwm = pwm
            turn = pivot or steering
            scale=max(1, abs(forward-turn), abs(forward+turn))
            self.left_duty = (forward-turn)/scale*(pwm or 0)/4095
            self.right_duty = (forward+turn)/scale*(pwm or 0)/4095
            self.linear = forward * speed
            self.angular = (pivot or steering) * min(1, speed * 5)
            self.deadline = self.clock() + .4
            return True

    def command(self, enabled=True):
        with self.lock:
            if not enabled or self.clock() >= self.deadline:
                return 0.0, 0.0
            return self.linear, self.angular

    def duty_command(self, enabled=True):
        with self.lock:
            if not enabled or self.clock() >= self.deadline:
                return 0.0, 0.0
            return self.left_duty, self.right_duty

    def stop(self):
        with self.lock:
            self.linear = self.angular = 0.0
            self.left_duty = self.right_duty = 0.0
            self.deadline = 0
