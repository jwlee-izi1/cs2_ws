"""Ground-truth thermal field for the simulated downward-facing sensor.

Two modes can coexist:

1. **Static (legacy)** — sum of 2D Gaussian hot spots:
       T(x, y) = ambient + sum_i  peak_i * exp(-d_i^2 / (2 * radius_i^2))
   Used by the original static-thermal-mapping demo.

2. **Dynamic fire (M3 wavefront + burn profile)** — radial wavefront sweeps
   out from an ignition point at constant velocity `v_front`. Every location
   the front reaches follows a per-cell burn profile (rise → plateau →
   decay):
       T(x, y, t) = ambient + peak * phi(tau)
       tau        = (t - ignition_time) - ||(x,y) - p_ign|| / v_front
       phi(tau)   = piecewise-linear rise/plateau/decay bell

Both terms add on top of `ambient`. Either block may be absent.

YAML schema:
    ambient: 22.0
    hot_spots:                            # optional, legacy static field
      - {x: 1.0, y: 1.0, peak: 50.0, radius: 0.3}
    fire:                                 # optional, dynamic M3 wavefront
      ignition_point: [-2.0, -2.0]
      ignition_time: 0.0                  # seconds (front emits at this t)
      v_front: 0.023                      # m/s
      peak_temperature: 80.0              # degC above ambient at burn peak
      burn:
        rise_duration: 15.0
        plateau_duration: 30.0
        decay_duration: 60.0
      random_seed: 42                     # reserved; deterministic for now
    qi:                                   # optional, scale for q_i estimator
      scale: 0.07                         # alpha: multiplies mean(T - ambient)
"""

from dataclasses import dataclass
from typing import List, Optional

import numpy as np
import yaml


@dataclass
class HotSpot:
    x: float
    y: float
    peak: float
    radius: float


@dataclass
class BurnProfile:
    """Piecewise-linear rise → plateau → decay envelope.

    Defines phi(tau): 0 outside [0, rise+plateau+decay], piecewise-linear
    inside. Cell heats up over `rise_duration`, holds peak for
    `plateau_duration`, then linearly cools back to ambient over
    `decay_duration` (burnout / fuel consumed).
    """
    rise_duration: float
    plateau_duration: float
    decay_duration: float

    @property
    def total_duration(self) -> float:
        return self.rise_duration + self.plateau_duration + self.decay_duration

    def evaluate(self, tau):
        """Evaluate phi(tau) where tau is a scalar or array of seconds-since-front."""
        tau = np.asarray(tau, dtype=np.float32)
        rise = self.rise_duration
        plateau_end = rise + self.plateau_duration
        decay_end = plateau_end + self.decay_duration
        phi = np.zeros_like(tau, dtype=np.float32)
        in_rise = (tau >= 0.0) & (tau < rise)
        in_plateau = (tau >= rise) & (tau < plateau_end)
        in_decay = (tau >= plateau_end) & (tau < decay_end)
        phi = np.where(in_rise, tau / max(rise, 1e-9), phi)
        phi = np.where(in_plateau, np.float32(1.0), phi)
        phi = np.where(
            in_decay,
            1.0 - (tau - plateau_end) / max(self.decay_duration, 1e-9),
            phi,
        )
        return phi.astype(np.float32)


@dataclass
class FireWavefront:
    """Single-ignition radial wavefront with per-cell burn profile (M3/M4).

    Optional `arena_radius` bounds the fuel region: cells outside a disc of
    that radius around (arena_center_x, arena_center_y) contribute zero
    (no fuel available beyond the boundary).

    Optional `anisotropy` enables wind-driven asymmetric spread (M4 model).
    With `wind_direction_deg = θ_w` and `anisotropy = a ∈ [0, 1)`, the front
    speed in direction θ from the ignition point is:
        v_eff(θ) = v_front * (1 + a * cos(θ - θ_w))
    Downwind (θ = θ_w): v_eff = v_front * (1 + a)  (fastest)
    Perp (θ ⊥ wind):    v_eff = v_front           (nominal)
    Upwind (θ - θ_w = π): v_eff = v_front * (1 - a)  (slowest)
    Front shape: ellipse stretched in wind direction. a=0 ⇒ no wind (M3).

    Optional **radius-cap suppression** models firefighters holding an inner
    line along an angular arc. In the suppression arc (defined by
    `suppression_center_deg` ± `suppression_half_angle_deg`), the maximum
    fire radius is capped at `suppression_max_radius` (which should be
    smaller than `arena_radius`). Cells beyond this cap in the suppression
    arc contribute zero. Cells outside the arc are unaffected (only the
    arena_radius fuel-disc applies). A linear taper across
    `suppression_edge_taper_deg` smoothly interpolates the effective cap
    between `max_radius` (inside the arc) and `arena_radius` (outside the
    arc), avoiding hard angular discontinuities in the truth field.

    suppression_max_radius <= 0 OR suppression_half_angle_deg <= 0 ⇒ no
    radius-cap (legacy behavior).
    """
    ignition_x: float
    ignition_y: float
    ignition_time: float
    v_front: float
    peak_temperature: float
    burn: BurnProfile
    random_seed: int = 42
    arena_radius: float = 0.0      # 0 = unbounded fuel (legacy); >0 = bound to disc
    arena_center_x: float = 0.0
    arena_center_y: float = 0.0
    wind_direction_deg: float = 0.0  # degrees at t=0, 0 = +x, 90 = +y
    wind_rotation_rate_deg_per_sec: float = 0.0  # rotation rate; 1.0 = 90° per 90s
    anisotropy: float = 0.0        # 0 = isotropic M3; in [0, 1) for wind-driven M4
    # Firefighter radius-cap suppression arc (world frame, fixed throughout
    # the run). max_radius <= 0 or half_angle_deg <= 0 ⇒ no suppression.
    suppression_center_deg: float = 0.0
    suppression_half_angle_deg: float = 0.0
    suppression_max_radius: float = 0.0
    suppression_edge_taper_deg: float = 5.0

    def evaluate(self, x, y, t):
        """Excess (above ambient) contribution from the fire at time t."""
        x = np.asarray(x, dtype=np.float32)
        y = np.asarray(y, dtype=np.float32)
        dx = x - self.ignition_x
        dy = y - self.ignition_y
        d = np.sqrt(dx * dx + dy * dy).astype(np.float32)

        # Direction-dependent effective front speed.
        # v_eff(θ) = v_front * (1 + a * cos(θ - θ_w(t)))
        # where (dx, dy)/d is the unit vector toward this cell from ignition.
        # Wind direction can rotate over time: θ_w(t) = direction_deg + rotation_rate * t.
        v_eff = np.full_like(d, max(self.v_front, 1e-9), dtype=np.float32)
        if self.anisotropy > 0.0:
            wind_dir_at_t = (
                self.wind_direction_deg
                + self.wind_rotation_rate_deg_per_sec * (t - self.ignition_time)
            )
            theta_w = np.deg2rad(wind_dir_at_t)
            wx, wy = np.cos(theta_w), np.sin(theta_w)
            # cos(theta - theta_w) = (dx*wx + dy*wy) / d  for d > 0.
            # For d == 0, set cos to 0 (the ignition cell itself; v_eff value
            # doesn't affect τ = t since d/v = 0).
            safe_d = np.where(d > 1e-9, d, np.float32(1.0))
            cos_th = (dx * wx + dy * wy) / safe_d
            cos_th = np.where(d > 1e-9, cos_th, np.float32(0.0))
            v_eff = np.maximum(
                self.v_front * (1.0 + self.anisotropy * cos_th),
                1e-9,
            ).astype(np.float32)

        tau = (t - self.ignition_time) - d / v_eff
        contribution = self.peak_temperature * self.burn.evaluate(tau)
        if self.arena_radius > 0.0:
            rx = x - self.arena_center_x
            ry = y - self.arena_center_y
            r_from_arena_center = np.sqrt(rx * rx + ry * ry)
            in_fuel = r_from_arena_center <= self.arena_radius
            contribution = np.where(in_fuel, contribution, np.float32(0.0))
        if (self.suppression_max_radius > 0.0
                and self.suppression_half_angle_deg > 0.0):
            # World-frame cell angle from arena center.
            ax = x - self.arena_center_x
            ay = y - self.arena_center_y
            theta_cell = np.rad2deg(np.arctan2(ay, ax))
            r_from_arena_center = np.sqrt(ax * ax + ay * ay)
            rel = ((theta_cell - self.suppression_center_deg + 180.0) % 360.0) - 180.0
            abs_rel = np.abs(rel)
            half = self.suppression_half_angle_deg
            taper = max(self.suppression_edge_taper_deg, 1e-6)
            # Outer bound on fire reach for this cell:
            #   inside arc (|abs_rel| <= half):                effective_r_max = max_radius
            #   in taper band (half < abs_rel <= half+taper):  linear from max_radius up to arena_radius
            #   outside (abs_rel > half + taper):              arena_radius (no cap, unaffected)
            unsuppressed_r = (
                float(self.arena_radius) if self.arena_radius > 0.0
                else float(np.inf)
            )
            in_arc = abs_rel <= half
            in_taper = (abs_rel > half) & (abs_rel <= half + taper)
            effective_r_max = np.full_like(abs_rel, unsuppressed_r, dtype=np.float32)
            effective_r_max = np.where(
                in_arc, np.float32(self.suppression_max_radius), effective_r_max
            )
            # Linear interpolation across the taper band from max_radius → unsuppressed_r.
            t = (abs_rel - half) / taper
            taper_value = (
                self.suppression_max_radius
                + t * (unsuppressed_r - self.suppression_max_radius)
            )
            effective_r_max = np.where(in_taper, taper_value, effective_r_max)
            # Zero out cells beyond the (now direction-dependent) effective r_max.
            in_allowed_disc = r_from_arena_center <= effective_r_max
            contribution = np.where(
                in_allowed_disc, contribution, np.float32(0.0)
            )
        return contribution.astype(np.float32)


class ThermalField:

    def __init__(
        self,
        ambient: float,
        hot_spots: List[HotSpot],
        fire: Optional[FireWavefront] = None,
        qi_scale: float = 1.0,
    ):
        self.ambient = float(ambient)
        self.hot_spots = list(hot_spots)
        self.fire = fire
        self.qi_scale = float(qi_scale)
        self._cx = np.array([h.x for h in hot_spots], dtype=np.float32)
        self._cy = np.array([h.y for h in hot_spots], dtype=np.float32)
        self._peak = np.array([h.peak for h in hot_spots], dtype=np.float32)
        self._inv_two_r2 = np.array(
            [1.0 / (2.0 * h.radius * h.radius) for h in hot_spots],
            dtype=np.float32,
        )

    @classmethod
    def from_yaml(cls, path: str) -> 'ThermalField':
        with open(path, 'r') as f:
            doc = yaml.safe_load(f)
        ambient = doc.get('ambient', 20.0)
        spots = [HotSpot(**s) for s in doc.get('hot_spots', []) or []]
        fire = None
        fire_doc = doc.get('fire')
        if fire_doc:
            ig = list(fire_doc['ignition_point'])
            burn = BurnProfile(**fire_doc['burn'])
            arena_center = fire_doc.get('arena_center', [0.0, 0.0])
            wind = fire_doc.get('wind') or {}
            suppression = fire_doc.get('suppression') or {}

            ignition_x = float(ig[0])
            ignition_y = float(ig[1])
            wind_direction_deg = float(wind.get('direction_deg', 0.0))
            suppression_center_deg = float(
                suppression.get('center_deg', (wind_direction_deg + 180.0) % 360.0)
            )

            # randomize_seed > 0 applies a deterministic seeded rotation to the
            # wind init AND the suppression cone (so the cone stays wind-locked
            # across seeds), plus a small jitter on the ignition point.
            randomize_seed = int(fire_doc.get('randomize_seed', 0))
            if randomize_seed > 0:
                rng = np.random.default_rng(randomize_seed)
                wind_delta = float(rng.uniform(0.0, 360.0))
                wind_direction_deg = (wind_direction_deg + wind_delta) % 360.0
                suppression_center_deg = (
                    suppression_center_deg + wind_delta
                ) % 360.0
                jitter_r = float(fire_doc.get('ignition_jitter_radius', 0.0))
                if jitter_r > 0.0:
                    r = jitter_r * float(np.sqrt(rng.uniform(0.0, 1.0)))
                    phi = float(rng.uniform(0.0, 2.0 * np.pi))
                    ignition_x += r * float(np.cos(phi))
                    ignition_y += r * float(np.sin(phi))

            fire = FireWavefront(
                ignition_x=ignition_x,
                ignition_y=ignition_y,
                ignition_time=float(fire_doc.get('ignition_time', 0.0)),
                v_front=float(fire_doc['v_front']),
                peak_temperature=float(fire_doc['peak_temperature']),
                burn=burn,
                random_seed=int(fire_doc.get('random_seed', 42)),
                arena_radius=float(fire_doc.get('arena_radius', 0.0)),
                arena_center_x=float(arena_center[0]),
                arena_center_y=float(arena_center[1]),
                wind_direction_deg=wind_direction_deg,
                wind_rotation_rate_deg_per_sec=float(
                    wind.get('rotation_rate_deg_per_sec', 0.0)
                ),
                anisotropy=float(wind.get('anisotropy', 0.0)),
                suppression_center_deg=suppression_center_deg,
                suppression_half_angle_deg=float(
                    suppression.get('half_angle_deg', 0.0)
                ),
                suppression_max_radius=float(
                    suppression.get('max_radius', 0.0)
                ),
                suppression_edge_taper_deg=float(
                    suppression.get('edge_taper_deg', 5.0)
                ),
            )
        qi_scale = float((doc.get('qi') or {}).get('scale', 1.0))
        return cls(ambient, spots, fire=fire, qi_scale=qi_scale)

    def evaluate(self, x, y, t: float = 0.0):
        """Evaluate T(x, y, t) at point arrays of arbitrary shape.

        Returns same-shape array of temperatures in degC.
        Backward compatible: callers passing only (x, y) get T at t=0.
        """
        x = np.asarray(x, dtype=np.float32)
        y = np.asarray(y, dtype=np.float32)
        out = np.full(x.shape, self.ambient, dtype=np.float32)
        # Static Gaussian hot spots (legacy)
        if self._cx.size > 0:
            dx = x[..., None] - self._cx
            dy = y[..., None] - self._cy
            d2 = dx * dx + dy * dy
            contributions = self._peak * np.exp(-d2 * self._inv_two_r2)
            out += contributions.sum(axis=-1).astype(np.float32)
        # Dynamic fire (M3)
        if self.fire is not None:
            out += self.fire.evaluate(x, y, t).astype(np.float32)
        return out
