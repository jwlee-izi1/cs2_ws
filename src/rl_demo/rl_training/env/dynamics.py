"""xy drone dynamics for the coverage env.

Two models are provided:

  * ``PointMassDrone`` — legacy rate-limited point mass (used to train iter1–9).
    Pos updates as ``pos += clip(target - pos, ±v_max·dt)``. No inertia. Useful
    only as a fast smoke-test / regression baseline.

  * ``CascadedPidDrone`` — a faithful 2D approximation of the crazyflie
    firmware's cascaded position-velocity-attitude controller. This is the
    controller `stabilizer.controller = 1`, the one cf2-sitl uses by default
    and the one your HW Crazyflie runs.

CascadedPidDrone collapses the three firmware loops (position P → velocity PI
→ attitude PID) into the two outer loops, since the attitude loop runs
~10× faster than the position loop and settles much faster than the policy
tick. Tilt-to-acceleration is the small-angle approximation
``a = g·tan(θ) ≈ g·θ`` since the firmware clips tilt to ±20°.

Gains default to the firmware's `platform_defaults_sitl.h` values:
    PID_POS_X_KP        = 2.0  m/s per m of position error
    PID_POS_VEL_X_MAX   = 1.0  m/s soft cap on velocity setpoint
    PID_VEL_X_KP        = 25.0 deg per m/s of velocity error
    PID_VEL_X_KI        = 1.0  deg·s per m/s of velocity error
    tilt cap            ≈ 20°  (firmware default soft limit)

Override with ``CascadedPidDrone(..., kp_pos=..., kp_vel=..., ki_vel=...)`` for
domain randomization during training.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


# Firmware-default cascaded PID gains, taken from crazyflie-firmware's
# `src/platform/interface/platform_defaults_sitl.h`. Identical to the cf2
# defaults (`platform_defaults_cf2.h`), so the same gains apply to HW.
SITL_DEFAULT_KP_POS = 2.0          # m/s per m position error
SITL_DEFAULT_KP_VEL_DEG_PER_MS = 25.0    # deg per (m/s) velocity error
SITL_DEFAULT_KI_VEL_DEG_S_PER_MS = 1.0   # deg·s per (m/s) velocity error
SITL_DEFAULT_VEL_SETPOINT_MAX = 1.0  # m/s, hard saturation on velocity setpoint
SITL_DEFAULT_TILT_MAX_DEG = 20.0     # soft tilt cap
GRAVITY = 9.81                       # m/s^2


@dataclass
class PidGains:
    """All cascaded-PID gains a CascadedPidDrone needs.

    Each scalar maps 1:1 to a firmware ``platform_defaults_*.h`` constant.
    Use ``PidGains.from_firmware()`` for the default cf2-sitl build; perturb
    fields for domain randomization (env passes a new PidGains per reset).
    """
    kp_pos: float = SITL_DEFAULT_KP_POS
    kp_vel_deg_per_ms: float = SITL_DEFAULT_KP_VEL_DEG_PER_MS
    ki_vel_deg_s_per_ms: float = SITL_DEFAULT_KI_VEL_DEG_S_PER_MS
    vel_setpoint_max: float = SITL_DEFAULT_VEL_SETPOINT_MAX
    tilt_max_deg: float = SITL_DEFAULT_TILT_MAX_DEG

    @classmethod
    def from_firmware(cls) -> "PidGains":
        return cls()


# ---------------------------------------------------------------------------
# Legacy point-mass drone (kept for backwards compat with iter1–9 checkpoints).
# ---------------------------------------------------------------------------
class PointMassDrone:

    def __init__(
        self,
        initial_xy: tuple[float, float] = (0.0, 0.0),
        altitude: float = 0.6,
        max_velocity: float = 1.5,
        streamer_rate_hz: float = 20.0,
    ):
        self.position = np.array(initial_xy, dtype=np.float32)
        self.altitude = float(altitude)
        self.max_velocity = float(max_velocity)
        self.streamer_dt = 1.0 / float(streamer_rate_hz)
        self.prev_position = self.position.copy()
        self.last_dt = self.streamer_dt

    def step(self, target_xy: np.ndarray, dt: float):
        self.prev_position = self.position.copy()
        self.last_dt = float(dt)
        n_substeps = max(1, int(round(dt / self.streamer_dt)))
        substep_dt = dt / n_substeps
        max_step = self.max_velocity * substep_dt
        target = np.asarray(target_xy, dtype=np.float32)
        for _ in range(n_substeps):
            delta = target - self.position
            dist = float(np.linalg.norm(delta))
            if dist <= max_step or dist < 1e-9:
                self.position = target.copy()
                break
            self.position = self.position + (delta * (max_step / dist)).astype(np.float32)

    @property
    def velocity(self) -> np.ndarray:
        return (self.position - self.prev_position) / max(self.last_dt, 1e-9)

    def reset(self, position_xy: tuple[float, float]):
        self.position = np.array(position_xy, dtype=np.float32)
        self.prev_position = self.position.copy()
        self.last_dt = self.streamer_dt


# ---------------------------------------------------------------------------
# Firmware-realistic cascaded PID drone (used to train iter10+).
# ---------------------------------------------------------------------------
class CascadedPidDrone:
    """2D approximation of the crazyflie firmware's cascaded PID controller.

    Inner loops:
        1. Position P → velocity setpoint
              vel_sp = clip(kp_pos · (target - pos), ±vel_setpoint_max)
        2. Velocity PI → tilt angle (deg)
              tilt_deg = kp_vel · (vel_sp - vel) + ki_vel · integral
              tilt_deg = clip(tilt_deg, ±tilt_max_deg)
        3. Tilt → accel (small-angle):  accel = g · tan(deg2rad(tilt_deg))

    State is integrated at ``inner_rate_hz`` substeps per outer ``step(dt)``
    call so the PI loop sees a sane dt regardless of the env's policy tick
    rate. The firmware position+velocity loops both run at 100 Hz, so
    inner_rate_hz defaults to 100.
    """

    def __init__(
        self,
        initial_xy: tuple[float, float] = (0.0, 0.0),
        altitude: float = 0.6,
        gains: PidGains | None = None,
        inner_rate_hz: float = 100.0,
        # Backwards-compat with the PointMassDrone constructor signature so
        # call sites that still pass max_velocity / streamer_rate_hz don't
        # break. These are now interpreted as overrides on the firmware's
        # ``vel_setpoint_max`` and inner loop rate respectively, but the
        # default firmware gains win unless caller overrides ``gains``.
        max_velocity: float | None = None,
        streamer_rate_hz: float | None = None,
    ):
        self.position = np.array(initial_xy, dtype=np.float32)
        self.altitude = float(altitude)
        self.gains = gains if gains is not None else PidGains.from_firmware()
        if max_velocity is not None:
            # Caller is treating this like the old PointMass v_max; reinterpret
            # as a velocity-setpoint cap. Firmware always saturates here.
            self.gains.vel_setpoint_max = float(max_velocity)
        if streamer_rate_hz is not None:
            self.inner_dt = 1.0 / float(streamer_rate_hz)
        else:
            self.inner_dt = 1.0 / float(inner_rate_hz)
        # Inner state.
        self.velocity_state = np.zeros(2, dtype=np.float32)
        self.i_vel = np.zeros(2, dtype=np.float32)
        # Snapshot for the public ``velocity`` property (avg over last step).
        self.prev_position = self.position.copy()
        self.last_dt = self.inner_dt

    def step(self, target_xy: np.ndarray, dt: float):
        self.prev_position = self.position.copy()
        self.last_dt = float(dt)
        target = np.asarray(target_xy, dtype=np.float32)
        # Substep at the inner-loop rate.
        n = max(1, int(round(dt / self.inner_dt)))
        sdt = dt / n
        for _ in range(n):
            # Stage 1 — Position P → velocity setpoint, saturated.
            vel_sp = self.gains.kp_pos * (target - self.position)
            np.clip(vel_sp, -self.gains.vel_setpoint_max,
                    self.gains.vel_setpoint_max, out=vel_sp)
            # Stage 2 — Velocity PI → tilt (degrees).
            err_v = vel_sp - self.velocity_state
            self.i_vel = self.i_vel + err_v * sdt
            tilt_deg = (
                self.gains.kp_vel_deg_per_ms * err_v
                + self.gains.ki_vel_deg_s_per_ms * self.i_vel
            )
            np.clip(tilt_deg, -self.gains.tilt_max_deg,
                    self.gains.tilt_max_deg, out=tilt_deg)
            # Stage 3 — Tilt → accel (small-angle exact via tan).
            accel = GRAVITY * np.tan(np.deg2rad(tilt_deg))
            # Integrate.
            self.velocity_state = self.velocity_state + accel * sdt
            self.position = self.position + self.velocity_state * sdt

    @property
    def velocity(self) -> np.ndarray:
        return (self.position - self.prev_position) / max(self.last_dt, 1e-9)

    @property
    def max_velocity(self) -> float:
        """Compatibility shim — env reads this off the drone to size things."""
        return float(self.gains.vel_setpoint_max)

    def reset(self, position_xy: tuple[float, float]):
        self.position = np.array(position_xy, dtype=np.float32)
        self.velocity_state = np.zeros(2, dtype=np.float32)
        self.i_vel = np.zeros(2, dtype=np.float32)
        self.prev_position = self.position.copy()
        self.last_dt = self.inner_dt

    def reset_with_gains(self, position_xy: tuple[float, float], gains: PidGains):
        """Reset with a fresh gain set — used for per-episode domain randomization."""
        self.gains = gains
        self.reset(position_xy)


# ---------------------------------------------------------------------------
# Domain randomization helper for training.
# ---------------------------------------------------------------------------
def randomize_gains(
    rng: np.random.Generator,
    *,
    kp_pos_range: tuple[float, float] = (1.5, 2.5),
    kp_vel_range: tuple[float, float] = (20.0, 30.0),
    ki_vel_range: tuple[float, float] = (0.5, 1.5),
    vel_setpoint_max_range: tuple[float, float] = (0.9, 1.1),
    tilt_max_range: tuple[float, float] = (18.0, 22.0),
) -> PidGains:
    """Sample a PidGains within ±25% of firmware defaults.

    Default ranges bracket the firmware's nominal gains by enough to cover:
      * Battery sag and prop wear (effective thrust changes by ~10%)
      * Ground effect during low hover
      * SITL vs HW manufacturing variance
      * Cross-axis coupling (modeled here as gain uncertainty)
    """
    return PidGains(
        kp_pos=float(rng.uniform(*kp_pos_range)),
        kp_vel_deg_per_ms=float(rng.uniform(*kp_vel_range)),
        ki_vel_deg_s_per_ms=float(rng.uniform(*ki_vel_range)),
        vel_setpoint_max=float(rng.uniform(*vel_setpoint_max_range)),
        tilt_max_deg=float(rng.uniform(*tilt_max_range)),
    )
