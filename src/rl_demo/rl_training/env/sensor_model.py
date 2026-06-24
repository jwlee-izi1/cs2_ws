"""Thermal sensor mirroring thermal_sensor_node.py.

Samples a 16×16 grid of the underlying ThermalField at the drone's pose,
adds Gaussian noise σ=0.5°C, and exposes the latest sample with a
configurable observation-delay buffer that mirrors ROS message-transport
latency (~30 ms).

API:
    sensor = ThermalSensor(field, ...)
    sensor.sample(position_xy, t)   # called at sensor_rate_hz (5 Hz)
    obs = sensor.read(t)            # called by policy; returns obs that is
                                    #   ~latency seconds stale
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np


class ThermalSensor:

    def __init__(
        self,
        field,
        altitude: float = 0.6,
        fov_deg: float = 35.0,
        resolution: int = 16,
        noise_sigma: float = 0.5,
        latency_seconds: float = 0.030,
        sensor_rate_hz: float = 5.0,
        rng: np.random.Generator | None = None,
    ):
        self.field = field
        self.altitude = float(altitude)
        self.fov_deg = float(fov_deg)
        self.resolution = int(resolution)
        self.noise_sigma = float(noise_sigma)
        self.latency = float(latency_seconds)
        self.sensor_dt = 1.0 / float(sensor_rate_hz)
        self.rng = rng if rng is not None else np.random.default_rng()

        # Precompute the normalized 16×16 pixel-center grid in sensor frame.
        h = self.altitude * math.tan(math.radians(self.fov_deg) / 2.0)
        # half-side of the footprint at ground (h is the half-width here).
        # Pixel centers in [-1, +1].
        u = (np.arange(self.resolution, dtype=np.float32) + 0.5) / self.resolution * 2.0 - 1.0
        gx, gy = np.meshgrid(u, u, indexing="xy")
        self._gx_h = (gx * h).astype(np.float32)
        self._gy_h = (gy * h).astype(np.float32)
        self.footprint_half = float(h)

        # Buffer of (timestamp, frame) — observation appears delayed.
        self._buffer: deque[tuple[float, np.ndarray]] = deque(maxlen=64)
        self._latest_obs: np.ndarray | None = None
        self._last_sample_t: float = -1e9

    def sample(self, position_xy: np.ndarray, t: float):
        """Sample the truth field at the drone's pose at time t.

        Called at sensor_rate_hz (5 Hz) by the env. Adds Gaussian noise and
        buffers (t, frame) for the read() side to retrieve with latency.
        """
        px, py = float(position_xy[0]), float(position_xy[1])
        xs = px + self._gx_h
        ys = py + self._gy_h
        truth = self.field.evaluate(xs, ys, t)
        noisy = truth + self.rng.normal(0.0, self.noise_sigma, truth.shape).astype(np.float32)
        self._buffer.append((float(t), noisy.astype(np.float32)))
        self._last_sample_t = float(t)

    def read(self, t: float) -> np.ndarray:
        """Return the most recent sample older than t - latency.

        Falls back to the newest available sample (or a zero frame if none).
        """
        cutoff = float(t) - self.latency
        for ts, frame in reversed(self._buffer):
            if ts <= cutoff:
                self._latest_obs = frame
                return frame
        if self._buffer:
            self._latest_obs = self._buffer[0][1]
            return self._buffer[0][1]
        return np.zeros((self.resolution, self.resolution), dtype=np.float32)

    def reset(self):
        self._buffer.clear()
        self._latest_obs = None
        self._last_sample_t = -1e9
