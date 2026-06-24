"""Fused thermal map + age_seconds tracker mirroring thermal_mapper_node.py.

Maintains a fixed grid (default 500×500 at 0.01m res over a 5×5m arena),
stores last-write-wins temperature + last_update_time per cell. The
age_seconds layer is computed on read as ``now - last_update_time``.

Exposes ``sector_age_crop(...)`` which downsamples a sector-wide bounding box
to a fixed 32×32 grid for the policy's observation, with cells outside the
wedge masked to 0 (the observation interprets masked cells as "irrelevant
because they're not in my sector").
"""

from __future__ import annotations

import math

import numpy as np


class ThermalMap:

    def __init__(
        self,
        length_x: float = 5.0,
        length_y: float = 5.0,
        resolution: float = 0.01,
        center_xy: tuple[float, float] = (0.0, 0.0),
        sensor_footprint: float = 0.378,
        sensor_resolution: int = 16,
    ):
        self.length_x = float(length_x)
        self.length_y = float(length_y)
        self.resolution = float(resolution)
        self.center_x = float(center_xy[0])
        self.center_y = float(center_xy[1])
        self.cols = int(round(self.length_x / self.resolution))
        self.rows = int(round(self.length_y / self.resolution))
        # last_update_time stores wall-clock seconds at last write; NaN = never.
        self.last_update_time = np.full((self.rows, self.cols), np.nan, dtype=np.float32)
        # Temperature layer (last-write-wins).
        self.thermal = np.full((self.rows, self.cols), np.nan, dtype=np.float32)
        # How many cells one sensor pixel projects onto (half-radius in cells).
        pixel_size = sensor_footprint / max(sensor_resolution, 1)
        self.pix_half_cells = max(
            int(math.ceil(pixel_size / (2.0 * self.resolution) - 0.5)),
            0,
        )

    def reset(self):
        self.last_update_time.fill(np.nan)
        self.thermal.fill(np.nan)

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        col = int(round((x - (self.center_x - self.length_x / 2.0)) / self.resolution))
        row = int(round((y - (self.center_y - self.length_y / 2.0)) / self.resolution))
        return row, col

    def update_from_frame(
        self,
        drone_xy: np.ndarray,
        sensor_grid_xy: np.ndarray,   # shape (res, res, 2): world (x,y) per pixel
        sensor_values: np.ndarray,    # shape (res, res): temperature per pixel
        t: float,
    ):
        """Project a 16×16 sensor frame onto the grid. Last-write-wins."""
        res = sensor_values.shape[0]
        for i in range(res):
            for j in range(res):
                x = float(sensor_grid_xy[i, j, 0])
                y = float(sensor_grid_xy[i, j, 1])
                row, col = self.world_to_grid(x, y)
                # Broadcast pixel value to a (2*half+1)² neighborhood.
                r0 = max(row - self.pix_half_cells, 0)
                r1 = min(row + self.pix_half_cells + 1, self.rows)
                c0 = max(col - self.pix_half_cells, 0)
                c1 = min(col + self.pix_half_cells + 1, self.cols)
                if r1 <= r0 or c1 <= c0:
                    continue
                self.thermal[r0:r1, c0:c1] = sensor_values[i, j]
                self.last_update_time[r0:r1, c0:c1] = t

    def age_seconds(self, now: float) -> np.ndarray:
        """Per-cell seconds since last observation. NaN if never observed."""
        return now - self.last_update_time

    def sector_age_crop(
        self,
        now: float,
        phi_mid_rad: float,
        phi_half_rad: float,
        r_max: float,
        crop_size: int = 32,
        unobserved_age_cap: float = 360.0,
    ) -> np.ndarray:
        """Downsample the sector's bounding box to a (crop_size × crop_size) grid.

        Cells outside the sector wedge are masked to 0. Unobserved cells
        (never-visited) get ``unobserved_age_cap`` — saturates the reward
        signal so the policy doesn't perceive them as infinitely valuable.
        """
        # Bounding box of the wedge: angular range [phi_mid - phi_half, phi_mid + phi_half],
        # radial range [0, r_max].
        angles = np.linspace(phi_mid_rad - phi_half_rad, phi_mid_rad + phi_half_rad, 32)
        cos_a = np.cos(angles)
        sin_a = np.sin(angles)
        bx = r_max * cos_a
        by = r_max * sin_a
        x_lo = float(min(0.0, bx.min(), self.center_x))
        x_hi = float(max(0.0, bx.max(), self.center_x))
        y_lo = float(min(0.0, by.min(), self.center_y))
        y_hi = float(max(0.0, by.max(), self.center_y))
        # Pad slightly to ensure full coverage.
        pad = 0.02
        x_lo -= pad
        x_hi += pad
        y_lo -= pad
        y_hi += pad

        # Build the downsampled grid.
        xs = np.linspace(x_lo, x_hi, crop_size, dtype=np.float32)
        ys = np.linspace(y_lo, y_hi, crop_size, dtype=np.float32)
        gx, gy = np.meshgrid(xs, ys, indexing="xy")
        ages = np.full((crop_size, crop_size), unobserved_age_cap, dtype=np.float32)
        # For each downsampled cell, read the closest underlying grid cell.
        cols = ((gx - (self.center_x - self.length_x / 2.0)) / self.resolution).astype(np.int32)
        rows = ((gy - (self.center_y - self.length_y / 2.0)) / self.resolution).astype(np.int32)
        in_bounds = (
            (cols >= 0) & (cols < self.cols) &
            (rows >= 0) & (rows < self.rows)
        )
        # Gather last_update_time, then age = now - lut.
        lut = np.where(in_bounds, self.last_update_time[np.clip(rows, 0, self.rows - 1),
                                                       np.clip(cols, 0, self.cols - 1)], np.nan)
        observed = ~np.isnan(lut)
        ages = np.where(observed, now - lut, unobserved_age_cap).astype(np.float32)
        ages = np.minimum(ages, unobserved_age_cap)

        # Mask cells outside the wedge.
        r_cells = np.sqrt((gx - self.center_x) ** 2 + (gy - self.center_y) ** 2)
        phi_cells = np.arctan2(gy - self.center_y, gx - self.center_x)
        # angular distance to phi_mid, wrapped to [-pi, pi]
        d_phi = np.mod(phi_cells - phi_mid_rad + np.pi, 2 * np.pi) - np.pi
        in_wedge = (np.abs(d_phi) <= phi_half_rad) & (r_cells <= r_max)
        ages = np.where(in_wedge, ages, np.float32(0.0))

        return ages.astype(np.float32)

    def cells_in_sector(
        self,
        phi_mid_rad: float,
        phi_half_rad: float,
        r_max: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return (row_indices, col_indices) of underlying grid cells inside the
        sector wedge. Used for reward computation (visited-this-step etc.)."""
        # Build full-grid cell centers.
        ys = (np.arange(self.rows, dtype=np.float32) + 0.5) * self.resolution + (
            self.center_y - self.length_y / 2.0
        )
        xs = (np.arange(self.cols, dtype=np.float32) + 0.5) * self.resolution + (
            self.center_x - self.length_x / 2.0
        )
        gy, gx = np.meshgrid(ys, xs, indexing="ij")
        r = np.sqrt((gx - self.center_x) ** 2 + (gy - self.center_y) ** 2)
        phi = np.arctan2(gy - self.center_y, gx - self.center_x)
        d_phi = np.mod(phi - phi_mid_rad + np.pi, 2 * np.pi) - np.pi
        mask = (np.abs(d_phi) <= phi_half_rad) & (r <= r_max)
        rows, cols = np.where(mask)
        return rows, cols
