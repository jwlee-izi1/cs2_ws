"""Unit tests for radial-spokes coverage policy."""

import math

from cf_coverage_planner.polar_lawnmower import Mode, PolarLawnmower
from cf_coverage_planner.sector_geometry import Sector


def _make(num_spokes=5, footprint=0.13):
    s = Sector(
        phi_mid=0.0,
        phi_half=math.radians(45.0),
        r_max=2.3,
        r_star=2.0,
        q=1.0,
        c=1.0,
    )
    return PolarLawnmower(sector=s, footprint_radius=footprint,
                          num_spokes=num_spokes, drone_speed=1.5, planner_rate_hz=5.0)


def test_first_spoke_targets_phi_low_side():
    """Initial spoke (idx=0) should sit half-step inside phi_low."""
    lm = _make(num_spokes=5)
    r, phi, mode = lm.step(current_r=0.13, current_phi=lm.sector.phi_low, leash=2.0)
    assert mode is Mode.OUTBOUND
    # phi_rel = -45 + 0.5*(90/5) = -45 + 9 = -36 deg
    assert math.isclose(math.degrees(phi), -36.0, abs_tol=0.1)


def test_outbound_targets_leash():
    lm = _make()
    r, phi, mode = lm.step(current_r=0.13, current_phi=lm.sector.phi_low, leash=2.0)
    assert mode is Mode.OUTBOUND
    assert math.isclose(r, 2.0)


def test_outbound_to_inbound_flip():
    lm = _make()
    # arrive at (leash, spoke_phi) -> next call should switch to INBOUND
    spoke_phi = lm._spoke_phi()
    lm.step(current_r=2.0, current_phi=spoke_phi, leash=2.0)
    assert lm.going_out is False
    # next step should target inner_radius
    r, phi, mode = lm.step(current_r=2.0, current_phi=spoke_phi, leash=2.0)
    assert mode is Mode.INBOUND
    assert math.isclose(r, lm.inner_radius)


def test_advance_to_next_spoke():
    lm = _make()
    spoke_phi = lm._spoke_phi()
    # complete outbound
    lm.step(current_r=2.0, current_phi=spoke_phi, leash=2.0)
    # complete inbound -> should advance to spoke 1
    lm.step(current_r=lm.inner_radius, current_phi=spoke_phi, leash=2.0)
    assert lm.spoke_idx == 1
    assert lm.going_out is True


def test_direction_flips_at_last_spoke():
    lm = _make(num_spokes=3)
    # Drive through all 3 spokes; after spoke 2 outbound+inbound, next advance
    # should flip direction (spoke_idx stays at 2 or goes back to 1).
    for _ in range(20):
        spoke_phi = lm._spoke_phi()
        lm.step(current_r=2.0, current_phi=spoke_phi, leash=2.0)         # OUTBOUND done
        lm.step(current_r=lm.inner_radius, current_phi=spoke_phi, leash=2.0)  # INBOUND done
        if lm.spoke_direction == -1:
            break
    assert lm.spoke_direction == -1


def test_snap_back_on_leash_retreat():
    lm = _make()
    r, phi, mode = lm.step(current_r=1.5, current_phi=0.0, leash=0.3)
    assert mode is Mode.RETRACTING
    assert math.isclose(r, 0.3)


def test_idle_when_leash_at_inner_radius():
    lm = _make()
    # leash equals inner_radius -> drone can't leave; idle
    r, phi, mode = lm.step(current_r=0.13, current_phi=0.0, leash=0.13)
    assert mode is Mode.IDLE


def test_leash_clamped_to_rmax():
    lm = _make()
    r, phi, mode = lm.step(current_r=0.13, current_phi=0.0, leash=100.0)
    assert r <= lm.sector.r_max + 1e-9


def test_wrap_around_sector():
    """cf3-like: sector phi_mid=pi spanning [+135, +225] crossing the +/-pi seam."""
    s = Sector(phi_mid=math.pi, phi_half=math.radians(45.0),
               r_max=2.3, r_star=1.0, q=1.0, c=1.0)
    lm = PolarLawnmower(sector=s, num_spokes=3, drone_speed=1.5, planner_rate_hz=5.0)
    # Drone at phi=+135 (phi_low), at inner. First spoke should be at phi_rel ~= -30 deg
    # (i.e. phi_mid - 30 deg = 150 deg). target_phi wrapped to [-pi, pi] = 150 deg.
    r, target_phi, mode = lm.step(current_r=0.13, current_phi=math.radians(135.0), leash=1.0)
    assert mode is Mode.OUTBOUND
    # The relative angle of target should be inside the sector
    phi_rel_target = math.atan2(math.sin(target_phi - math.pi), math.cos(target_phi - math.pi))
    assert -math.pi/4 <= phi_rel_target <= math.pi/4
