"""Wheel-space command limiting primitives; velocities are tangential m/s.

This module deliberately does not implement a protective-stop interface.
"""
import math


class WheelLimiter:
    # Match Gazebo DiffDrive: geometric track 0.50 m * calibration 1.10.
    def __init__(self, track=0.55, max_speed=1.0, max_acceleration=0.5,
                 timeout=0.5, max_dt=0.1):
        values = (track, max_speed, max_acceleration, timeout, max_dt)
        if not all(math.isfinite(x) and x > 0 for x in values):
            raise ValueError('limiter parameters must be finite and positive')
        self.track, self.max_speed, self.max_acceleration, self.timeout, self.max_dt = values
        self.left = self.right = 0.0
        self.target = (0.0, 0.0)
        self.last_command = None
        self.last_time = None

    def command(self, linear, angular, now):
        if not all(math.isfinite(x) for x in (linear, angular, now)):
            raise ValueError('command and time must be finite')
        left = linear - self.track * angular / 2
        right = linear + self.track * angular / 2
        scale = min(1.0, self.max_speed / max(abs(left), abs(right), self.max_speed))
        self.target = left * scale, right * scale
        self.last_command = now

    def step(self, now):
        if not math.isfinite(now):
            raise ValueError('time must be finite')
        if self.last_time is None:
            self.last_time = now
            return self.body()
        if now < self.last_time:
            self.last_time = now
            self.last_command = None
            self.target = (0.0, 0.0)
            return self.body()
        dt = min(now - self.last_time, self.max_dt)
        self.last_time = now
        if self.last_command is None or now - self.last_command > self.timeout:
            target = (0.0, 0.0)
        else:
            target = self.target
        dl, dr = target[0] - self.left, target[1] - self.right
        scale = min(1.0, self.max_acceleration * dt / max(abs(dl), abs(dr), 1e-300))
        self.left += dl * scale
        self.right += dr * scale
        return self.body()

    def body(self):
        return (self.left + self.right) / 2, (self.right - self.left) / self.track
