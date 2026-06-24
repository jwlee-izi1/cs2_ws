"""Radial-spokes coverage policy (POC).

Drone fans out N spokes across its sector. Each spoke is one out+in cycle:
  1. OUTBOUND: target = (leash, spoke_phi)         radial out
  2. INBOUND:  target = (inner_radius, spoke_phi)  radial back

After a spoke cycle completes, spoke_idx advances by spoke_direction; direction
flips at the sector-extreme spokes so the fan oscillates within the wedge forever.

Spokes are placed at evenly-spaced angles inside the sector, offset by half a
step from each boundary so adjacent drones never visit the same line.

Visual character (with num_spokes=5):
  cf1 (leash 1.99): 5 long radial lines, r=0.13 -> 1.99 across east wedge.
  cf4 (leash 0.43): 5 tiny radial lines, r=0.13 -> 0.43 across south wedge.

Drone is never idle - always either OUTBOUND or INBOUND. Asymmetry is visible
from frame 1 (different leashes -> different spoke lengths).

If leash retracts below current_r, drone snaps back via RETRACTING branch; the
spoke cycle resumes after the retract settles.

Constraint correctness: drone physical r stays in [inner_radius, leash] except
during a retraction transient. Algorithmic iterate r_i^k never violates the
budget by Theorem 1.

THROWAWAY when RL lands.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Tuple

from .sector_geometry import Sector, wrap_angle


class Mode(Enum):
    OUTBOUND = 'outbound'        # going from inner_radius to leash
    INBOUND = 'inbound'          # going from leash to inner_radius
    RETRACTING = 'retracting'    # snapping back because leash dropped below current r
    IDLE = 'idle'                # leash too small to leave inner_radius


def arc_radius(arc_idx: int, footprint_radius: float) -> float:
    """Kept for API compat with old tests; not used by spokes policy."""
    return (2 * arc_idx - 1) * footprint_radius


@dataclass
class PolarLawnmower:
    sector: Sector
    footprint_radius: float = 0.13       # api compat
    deadband: float = 0.10               # api compat
    waypoint_tolerance: float = 0.05     # m
    drone_speed: float = 1.5             # api compat
    planner_rate_hz: float = 5.0         # api compat
    num_spokes: int = 5                  # spokes per sector
    inner_radius: float = 0.5            # inner endpoint for spokes - keep away from origin
                                         # to prevent cross-drone collisions at sector corners

    def __post_init__(self):
        self.spoke_idx: int = 0          # 0 .. num_spokes-1
        self.spoke_direction: int = +1   # +1 advances toward phi_high, -1 toward phi_low
        self.going_out: bool = True      # True = OUTBOUND leg, False = INBOUND
        # diagnostic (preserved for any consumer using these)
        self.frontier_radius: float = 0.0
        self.arc_idx: int = 1
        self.arc_direction: int = +1
        self.arc_done: bool = False

    def _spoke_phi(self) -> float:
        """Angle of the current spoke. Spokes are evenly spaced inside the wedge,
        offset by half a step from each boundary."""
        step = 2.0 * self.sector.phi_half / self.num_spokes
        phi_rel = -self.sector.phi_half + (self.spoke_idx + 0.5) * step
        return wrap_angle(self.sector.phi_mid + phi_rel)

    def step(self, current_r: float, current_phi: float, leash: float) -> Tuple[float, float, Mode]:
        leash = max(0.0, min(self.sector.r_max, leash))
        # Adaptive inner endpoint: drones with small leash (e.g. cf4) would idle if we
        # used the global inner_radius. Shrink the inner endpoint so the drone always
        # has at least min_spoke_range of radial motion. Drones with large leash use
        # the configured inner_radius unchanged.
        min_spoke_range = 0.15
        effective_inner = min(self.inner_radius,
                              max(self.footprint_radius, leash - min_spoke_range))
        outer = max(effective_inner, leash)

        # Branch 1: snap-back if leash retreated well below current r.
        if leash + self.waypoint_tolerance < current_r:
            return (leash, current_phi, Mode.RETRACTING)

        spoke_phi = self._spoke_phi()

        # If leash is so small the drone can't leave effective_inner, idle in place
        if outer - effective_inner < self.waypoint_tolerance:
            return (effective_inner, spoke_phi, Mode.IDLE)

        target_r = outer if self.going_out else effective_inner

        # Detect arrival on the current spoke leg
        r_done = abs(current_r - target_r) < self.waypoint_tolerance
        phi_rel_drone = wrap_angle(current_phi - self.sector.phi_mid)
        phi_rel_spoke = wrap_angle(spoke_phi - self.sector.phi_mid)
        phi_done = abs(phi_rel_drone - phi_rel_spoke) < 0.05  # ~3 deg

        if r_done and phi_done:
            if self.going_out:
                # finished OUTBOUND leg -> switch to INBOUND
                self.going_out = False
                self.frontier_radius = max(self.frontier_radius, current_r)
            else:
                # finished INBOUND leg -> advance to next spoke
                next_idx = self.spoke_idx + self.spoke_direction
                if next_idx >= self.num_spokes:
                    self.spoke_direction = -1
                    next_idx = self.spoke_idx + self.spoke_direction
                elif next_idx < 0:
                    self.spoke_direction = +1
                    next_idx = self.spoke_idx + self.spoke_direction
                self.spoke_idx = next_idx
                self.going_out = True
                spoke_phi = self._spoke_phi()
                target_r = outer

        mode = Mode.OUTBOUND if self.going_out else Mode.INBOUND
        return (target_r, spoke_phi, mode)
