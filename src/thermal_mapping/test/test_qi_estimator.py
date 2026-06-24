"""Tests for the q_i estimator (compute_sector_qi).

Focus: sanity checks on per-sector heat integration.
- Uniform field: all sectors equal q_i
- One sector hot, others cold: that sector's q_i dominates
- Scale factor α applied correctly
- Sector wedge geometry correct (wrap around ±π handled)
"""

import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))

from thermal_mapping.thermal_field import (
    BurnProfile,
    FireWavefront,
    HotSpot,
    ThermalField,
)
from thermal_mapping.qi_estimator import SectorGeom, compute_sector_qi


def make_4_sectors(r_outer=2.0):
    """4 equal angular sectors (90° each) around origin, matching arena_4drone."""
    return [
        SectorGeom('cf1', phi_mid=math.radians(0),   phi_half=math.radians(45),
                   r_outer=r_outer, center_xy=(0.0, 0.0)),
        SectorGeom('cf2', phi_mid=math.radians(90),  phi_half=math.radians(45),
                   r_outer=r_outer, center_xy=(0.0, 0.0)),
        SectorGeom('cf3', phi_mid=math.radians(180), phi_half=math.radians(45),
                   r_outer=r_outer, center_xy=(0.0, 0.0)),
        SectorGeom('cf4', phi_mid=math.radians(270), phi_half=math.radians(45),
                   r_outer=r_outer, center_xy=(0.0, 0.0)),
    ]


def test_uniform_field_gives_equal_qi():
    """A field that's uniformly above ambient → equal q_i across all sectors."""
    # Build a synthetic uniform-excess "fire" — use a degenerate burn profile that's always at plateau.
    # Easier: directly construct a field with no fire but a tweaked ambient-via-evaluate is harder.
    # Instead use a fire that's in plateau everywhere at the chosen t.
    burn = BurnProfile(rise_duration=0.1, plateau_duration=1e6, decay_duration=0.1)
    # v_front huge → front has reached everywhere by t=1
    fire = FireWavefront(ignition_x=0.0, ignition_y=0.0, ignition_time=0.0,
                         v_front=1e6, peak_temperature=10.0, burn=burn)
    field = ThermalField(ambient=20.0, hot_spots=[], fire=fire, qi_scale=1.0)
    sectors = make_4_sectors()
    qi = compute_sector_qi(field, t=1.0, sectors=sectors)
    # Field is uniformly ambient+10 in plateau, so excess=10 everywhere → q_i should all equal 10.
    assert np.allclose(qi, 10.0, atol=0.5)
    # And they should be very close to each other.
    assert qi.max() - qi.min() < 0.5


def test_hot_in_one_sector_dominates():
    """If only one sector contains a hot spot, that sector's q_i is highest."""
    hs = [HotSpot(x=1.0, y=0.0, peak=50.0, radius=0.3)]  # in cf1's sector (phi_mid=0)
    field = ThermalField(ambient=20.0, hot_spots=hs, fire=None, qi_scale=1.0)
    sectors = make_4_sectors()
    qi = compute_sector_qi(field, t=0.0, sectors=sectors)
    # cf1 should have the largest q_i
    assert np.argmax(qi) == 0, f"expected cf1 dominant, got qi={qi}"
    # cf1 should be at least 3x the others
    assert qi[0] > 3.0 * max(qi[1], qi[2], qi[3])


def test_hot_in_cf3_sector():
    """Hot spot at phi=180° → cf3 (phi_mid=180°) dominates."""
    hs = [HotSpot(x=-1.0, y=0.0, peak=50.0, radius=0.3)]
    field = ThermalField(ambient=20.0, hot_spots=hs, fire=None, qi_scale=1.0)
    sectors = make_4_sectors()
    qi = compute_sector_qi(field, t=0.0, sectors=sectors)
    assert np.argmax(qi) == 2, f"expected cf3 dominant, got qi={qi}"


def test_hot_in_cf4_sector_wrap_handling():
    """Hot spot at phi=-90° = 270° → cf4 (phi_mid=270°) dominates.
    Tests that phi wrap-around at ±π is handled correctly."""
    hs = [HotSpot(x=0.0, y=-1.0, peak=50.0, radius=0.3)]
    field = ThermalField(ambient=20.0, hot_spots=hs, fire=None, qi_scale=1.0)
    sectors = make_4_sectors()
    qi = compute_sector_qi(field, t=0.0, sectors=sectors)
    assert np.argmax(qi) == 3, f"expected cf4 dominant, got qi={qi}"


def test_qi_scale_applied():
    """qi_scale multiplies the returned values."""
    hs = [HotSpot(x=1.0, y=0.0, peak=50.0, radius=0.3)]
    field_a = ThermalField(ambient=20.0, hot_spots=hs, fire=None, qi_scale=1.0)
    field_b = ThermalField(ambient=20.0, hot_spots=hs, fire=None, qi_scale=2.0)
    sectors = make_4_sectors()
    qi_a = compute_sector_qi(field_a, t=0.0, sectors=sectors)
    qi_b = compute_sector_qi(field_b, t=0.0, sectors=sectors)
    # field_b should be exactly 2x field_a (linear scale)
    np.testing.assert_allclose(qi_b, 2.0 * qi_a, rtol=1e-4)


def test_ambient_only_gives_zero_qi():
    """When field is at ambient everywhere, excess=0 → q_i=0."""
    field = ThermalField(ambient=20.0, hot_spots=[], fire=None, qi_scale=1.0)
    sectors = make_4_sectors()
    qi = compute_sector_qi(field, t=0.0, sectors=sectors)
    assert np.allclose(qi, 0.0, atol=1e-3)


def test_fire_at_corner_lights_up_sector_first():
    """M3 fire ignited in cf3's quadrant lights up cf3 first."""
    burn = BurnProfile(rise_duration=10.0, plateau_duration=20.0, decay_duration=30.0)
    fire = FireWavefront(ignition_x=-1.5, ignition_y=-1.5, ignition_time=0.0,
                         v_front=0.05, peak_temperature=50.0, burn=burn)
    field = ThermalField(ambient=20.0, hot_spots=[], fire=fire, qi_scale=1.0)
    sectors = make_4_sectors()
    # At t=20, the fire has spread ~1m from ignition → cf3 area is heating up
    qi = compute_sector_qi(field, t=20.0, sectors=sectors)
    # cf3 is at phi_mid=180°, ignition at (-1.5,-1.5) is at phi=225° (between cf3 and cf4).
    # Both should heat but cf3 OR cf4 should be the leader, not cf1 (opposite quadrant).
    assert qi[2] > qi[0] and qi[3] > qi[0], f"qi={qi}"
