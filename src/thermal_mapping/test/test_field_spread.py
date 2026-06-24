"""Tests for the M3 fire model (wavefront + burn profile) in thermal_field.py.

Focus: confirm the closed-form math has the right shape.
- pre-ignition: T == ambient everywhere
- at ignition point at t=0: just after τ=0, T starts to rise
- on the wavefront (τ=0 +): cells just begin heating
- mid-burn (τ in plateau): peak temperature reached
- post-burn (τ > total_duration): back to ambient (burnout)
- vectorization: same answer for scalar and array inputs
"""

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


def make_field(v_front=0.5, peak=80.0, rise=10.0, plateau=20.0, decay=30.0):
    burn = BurnProfile(rise_duration=rise, plateau_duration=plateau, decay_duration=decay)
    fire = FireWavefront(
        ignition_x=0.0, ignition_y=0.0, ignition_time=0.0,
        v_front=v_front, peak_temperature=peak, burn=burn,
    )
    return ThermalField(ambient=20.0, hot_spots=[], fire=fire, qi_scale=1.0)


def test_pre_ignition_is_ambient():
    """Before t=0 (and at any location), T == ambient everywhere."""
    field = make_field()
    x = np.array([[0.0, 0.5, 1.0], [-1.0, 0.0, 2.0]])
    y = np.zeros_like(x)
    T = field.evaluate(x, y, t=-5.0)
    assert np.allclose(T, 20.0)


def test_ignition_point_at_peak_time():
    """At ignition (0,0), τ = t. At t=rise (front-just-passed-peak start) we hit peak."""
    field = make_field(rise=10.0)
    T = field.evaluate(np.array([0.0]), np.array([0.0]), t=10.0)
    # at τ=rise we enter the plateau → φ=1 → T = ambient + peak
    assert T[0] == pytest.approx(20.0 + 80.0, abs=1e-3)


def test_far_from_ignition_pre_arrival():
    """A point far away hasn't been reached by the front yet."""
    field = make_field(v_front=0.5)
    # Point at distance 10m, v_front=0.5 → front arrives at t=20s
    T_before = field.evaluate(np.array([10.0]), np.array([0.0]), t=5.0)
    assert T_before[0] == pytest.approx(20.0, abs=1e-3)


def test_burn_profile_rises_at_front():
    """Right after the front arrives at a point, T rises above ambient."""
    field = make_field(v_front=0.5, rise=10.0)
    # Point at distance 5m → front arrives at t=10s. At t=15s, τ=5 (mid-rise)
    T = field.evaluate(np.array([5.0]), np.array([0.0]), t=15.0)
    # Mid-rise: φ should be ~0.5 → T ≈ ambient + 0.5*peak
    assert 20.0 + 0.3 * 80.0 < T[0] < 20.0 + 0.7 * 80.0


def test_burnout_returns_to_ambient():
    """Long after burn completes at a point, T returns to ambient."""
    field = make_field(v_front=0.5, rise=10.0, plateau=20.0, decay=30.0)
    # Total burn = 60s. Point at distance 1m → front arrives at t=2s.
    # Burn ends at t = 2 + 60 = 62s. At t=100s, well burnt out.
    T = field.evaluate(np.array([1.0]), np.array([0.0]), t=100.0)
    assert T[0] == pytest.approx(20.0, abs=1e-3)


def test_monotonic_distance_pre_arrival():
    """At a fixed early time, distances beyond v_front*t are still ambient."""
    field = make_field(v_front=0.5)
    # At t=5, front has reached r=2.5m. Beyond that → ambient.
    xs = np.array([3.0, 4.0, 5.0])
    ys = np.zeros_like(xs)
    T = field.evaluate(xs, ys, t=5.0)
    assert np.allclose(T, 20.0)


def test_vectorization_consistency():
    """Scalar and array inputs give the same answers at matching points."""
    field = make_field()
    pts = [(0.0, 0.0), (1.0, 1.0), (2.0, 0.0), (-1.0, 0.5)]
    t = 25.0
    # Array call
    xs = np.array([p[0] for p in pts])
    ys = np.array([p[1] for p in pts])
    T_arr = field.evaluate(xs, ys, t)
    # Scalar calls
    T_scalar = np.array([
        field.evaluate(np.array([p[0]]), np.array([p[1]]), t)[0] for p in pts
    ])
    assert np.allclose(T_arr, T_scalar, atol=1e-5)


def test_plateau_is_flat():
    """During the plateau period, T stays at peak."""
    field = make_field(v_front=0.5, rise=10.0, plateau=20.0)
    # Origin, τ = t. Plateau from τ=10 to τ=30.
    for t in [12.0, 18.0, 25.0, 29.0]:
        T = field.evaluate(np.array([0.0]), np.array([0.0]), t=t)
        assert T[0] == pytest.approx(20.0 + 80.0, abs=1e-3), f"failed at t={t}"


def test_decay_is_monotonic():
    """During decay, T strictly decreases with time."""
    field = make_field(v_front=0.5, rise=10.0, plateau=20.0, decay=30.0)
    # Origin, plateau ends at τ=30, decay ends at τ=60. Sample during decay.
    ts = [35.0, 40.0, 50.0, 58.0]
    temps = [float(field.evaluate(np.array([0.0]), np.array([0.0]), t=t)[0]) for t in ts]
    for i in range(len(ts) - 1):
        assert temps[i] > temps[i + 1], f"not monotone: t={ts[i]}→{ts[i+1]}, T={temps[i]:.3f}→{temps[i+1]:.3f}"


def test_legacy_static_hotspots_still_work():
    """Backward compat: hot_spots without fire still gives static T(x,y)."""
    hs = [HotSpot(x=1.0, y=0.0, peak=10.0, radius=0.5)]
    field = ThermalField(ambient=20.0, hot_spots=hs, fire=None, qi_scale=1.0)
    T_at_spot = field.evaluate(np.array([1.0]), np.array([0.0]), t=0.0)
    assert T_at_spot[0] == pytest.approx(30.0, abs=1e-3)
    T_far = field.evaluate(np.array([10.0]), np.array([10.0]), t=0.0)
    assert T_far[0] == pytest.approx(20.0, abs=1e-3)


def test_bounded_fuel_clips_outside_disc():
    """Cells outside arena_radius contribute zero fire, regardless of front."""
    burn = BurnProfile(rise_duration=5.0, plateau_duration=10.0, decay_duration=15.0)
    fire = FireWavefront(
        ignition_x=0.0, ignition_y=0.0, ignition_time=0.0,
        v_front=1.0, peak_temperature=50.0, burn=burn,
        arena_radius=1.0, arena_center_x=0.0, arena_center_y=0.0,
    )
    field = ThermalField(ambient=20.0, hot_spots=[], fire=fire, qi_scale=1.0)
    # Inside the fuel disc: fire is active (front has reached, in plateau)
    T_in = field.evaluate(np.array([0.5]), np.array([0.0]), t=10.0)
    assert T_in[0] > 20.0 + 1.0, f"inside-disc point should be hot, got {T_in[0]}"
    # Outside the fuel disc: pure ambient even though front has mathematically arrived
    T_out = field.evaluate(np.array([1.5]), np.array([0.0]), t=10.0)
    assert T_out[0] == pytest.approx(20.0, abs=1e-3), \
        f"outside-disc point should be ambient, got {T_out[0]}"


def test_bounded_fuel_off_center_disc():
    """arena_center moves the fuel disc; cells far from center stay ambient."""
    burn = BurnProfile(rise_duration=5.0, plateau_duration=10.0, decay_duration=15.0)
    fire = FireWavefront(
        ignition_x=2.0, ignition_y=2.0, ignition_time=0.0,
        v_front=1.0, peak_temperature=50.0, burn=burn,
        arena_radius=1.0, arena_center_x=2.0, arena_center_y=2.0,
    )
    field = ThermalField(ambient=20.0, hot_spots=[], fire=fire, qi_scale=1.0)
    # Inside fuel disc (centered at 2,2): point at (2.5, 2.0) is 0.5 from center
    T_in = field.evaluate(np.array([2.5]), np.array([2.0]), t=10.0)
    assert T_in[0] > 20.0, f"inside-disc point should be hot, got {T_in[0]}"
    # Far from fuel disc: point at origin is ~2.83 from (2,2), beyond radius=1
    T_out = field.evaluate(np.array([0.0]), np.array([0.0]), t=10.0)
    assert T_out[0] == pytest.approx(20.0, abs=1e-3)


def test_wind_anisotropic_front_reaches_downwind_faster():
    """With wind in +x and anisotropy=0.5, downwind cell reaches plateau before upwind cell."""
    burn = BurnProfile(rise_duration=5.0, plateau_duration=10.0, decay_duration=15.0)
    fire = FireWavefront(
        ignition_x=0.0, ignition_y=0.0, ignition_time=0.0,
        v_front=1.0, peak_temperature=50.0, burn=burn,
        wind_direction_deg=0.0,    # wind in +x
        anisotropy=0.5,
    )
    field = ThermalField(ambient=20.0, hot_spots=[], fire=fire, qi_scale=1.0)
    # At t=10:
    #   Downwind cell at (10, 0):   v_eff = 1*(1+0.5*1) = 1.5;  tau = 10 - 10/1.5 = 3.33 (in rise)
    #   Upwind cell at (-10, 0):    v_eff = 1*(1+0.5*-1) = 0.5; tau = 10 - 10/0.5 = -10 (not reached)
    #   Perp cell at (0, 10):       v_eff = 1*(1+0.5*0) = 1.0;  tau = 10 - 10/1.0 = 0   (just reached)
    T_down = field.evaluate(np.array([10.0]), np.array([0.0]), t=10.0)
    T_up   = field.evaluate(np.array([-10.0]), np.array([0.0]), t=10.0)
    T_perp = field.evaluate(np.array([0.0]), np.array([10.0]), t=10.0)
    assert T_down[0] > 20.0 + 1.0, f"downwind should be heating, got {T_down[0]}"
    assert T_up[0] == pytest.approx(20.0, abs=1e-3), f"upwind not yet reached, got {T_up[0]}"
    # Perp: at tau=0 (just reached), phi=0 → T = ambient
    assert T_perp[0] == pytest.approx(20.0, abs=0.5), f"perp at front, got {T_perp[0]}"


def test_wind_anisotropy_zero_is_isotropic():
    """anisotropy=0 should produce identical results to no-wind case (M3)."""
    burn = BurnProfile(rise_duration=5.0, plateau_duration=10.0, decay_duration=15.0)
    # Without wind
    fire_iso = FireWavefront(
        ignition_x=0.0, ignition_y=0.0, ignition_time=0.0,
        v_front=1.0, peak_temperature=50.0, burn=burn,
    )
    # With wind but anisotropy=0
    fire_zero = FireWavefront(
        ignition_x=0.0, ignition_y=0.0, ignition_time=0.0,
        v_front=1.0, peak_temperature=50.0, burn=burn,
        wind_direction_deg=45.0, anisotropy=0.0,
    )
    fld_iso = ThermalField(ambient=20.0, hot_spots=[], fire=fire_iso)
    fld_zero = ThermalField(ambient=20.0, hot_spots=[], fire=fire_zero)
    xs = np.array([1.0, -1.0, 0.0, 0.0])
    ys = np.array([0.0, 0.0, 1.0, -1.0])
    T_iso = fld_iso.evaluate(xs, ys, t=5.0)
    T_zero = fld_zero.evaluate(xs, ys, t=5.0)
    np.testing.assert_allclose(T_iso, T_zero, rtol=1e-4)


def test_wind_load_from_yaml(tmp_path):
    """from_yaml correctly parses wind block."""
    yaml_text = """
ambient: 20.0
fire:
  ignition_point: [0.0, 0.0]
  ignition_time: 0.0
  v_front: 0.01
  peak_temperature: 80.0
  burn:
    rise_duration: 15.0
    plateau_duration: 30.0
    decay_duration: 60.0
  wind:
    direction_deg: 30.0
    anisotropy: 0.5
"""
    f = tmp_path / 'field.yaml'
    f.write_text(yaml_text)
    field = ThermalField.from_yaml(str(f))
    assert field.fire.wind_direction_deg == 30.0
    assert field.fire.anisotropy == 0.5


def test_unbounded_fuel_is_default():
    """arena_radius = 0 (default) means no fuel bounding (legacy behavior)."""
    burn = BurnProfile(rise_duration=5.0, plateau_duration=10.0, decay_duration=15.0)
    fire = FireWavefront(
        ignition_x=0.0, ignition_y=0.0, ignition_time=0.0,
        v_front=1.0, peak_temperature=50.0, burn=burn,
    )
    field = ThermalField(ambient=20.0, hot_spots=[], fire=fire, qi_scale=1.0)
    # Even far from origin, if front has arrived and cell is in burn cycle, T > ambient.
    # At t=15, front at distance 15. Point at (10, 0): τ = 15 - 10 = 5 = end of rise.
    T = field.evaluate(np.array([10.0]), np.array([0.0]), t=15.0)
    assert T[0] > 20.0 + 1.0, f"unbounded fuel should heat far cells, got {T[0]}"


def test_load_from_yaml(tmp_path):
    """from_yaml correctly parses ambient, hot_spots, fire, qi blocks."""
    yaml_text = """
ambient: 25.0
hot_spots: []
fire:
  ignition_point: [0.0, 0.0]
  ignition_time: 0.0
  v_front: 1.0
  peak_temperature: 50.0
  burn:
    rise_duration: 5.0
    plateau_duration: 10.0
    decay_duration: 15.0
  arena_radius: 2.0
  arena_center: [0.0, 0.0]
qi:
  scale: 0.5
"""
    f = tmp_path / 'field.yaml'
    f.write_text(yaml_text)
    field = ThermalField.from_yaml(str(f))
    assert field.ambient == 25.0
    assert field.qi_scale == 0.5
    assert field.fire is not None
    assert field.fire.v_front == 1.0
    assert field.fire.peak_temperature == 50.0
    assert field.fire.arena_radius == 2.0
    # At ignition origin at t=5 (just-entering-plateau), T = ambient + peak
    T = field.evaluate(np.array([0.0]), np.array([0.0]), t=5.0)
    assert T[0] == pytest.approx(25.0 + 50.0, abs=1e-3)
    # Far outside fuel disc (>2m): ambient even if front has reached
    T_far = field.evaluate(np.array([5.0]), np.array([0.0]), t=10.0)
    assert T_far[0] == pytest.approx(25.0, abs=1e-3)


# ---------------------------------------------------------------------------
# Suppression tests (2026-05-26 — firefighter radius-cap narrative layer)
# Mechanism: cells whose angle from arena center falls in the suppression arc
# AND whose radius from arena center exceeds `max_radius` contribute zero.
# Cells outside the arc, or inside the arc at small radius, burn normally.
# ---------------------------------------------------------------------------

def make_suppression_field(max_radius, center_deg, half_angle_deg=90.0,
                           edge_taper_deg=5.0, arena_radius=2.0, anisotropy=0.0):
    """Field with radius-cap suppression configured. Isotropic spread by
    default so cells at equal distance differ only by direction."""
    burn = BurnProfile(rise_duration=5.0, plateau_duration=10.0, decay_duration=15.0)
    fire = FireWavefront(
        ignition_x=0.0, ignition_y=0.0, ignition_time=0.0,
        v_front=0.5, peak_temperature=80.0, burn=burn,
        arena_radius=arena_radius, anisotropy=anisotropy,
        suppression_center_deg=center_deg,
        suppression_half_angle_deg=half_angle_deg,
        suppression_max_radius=max_radius,
        suppression_edge_taper_deg=edge_taper_deg,
    )
    return ThermalField(ambient=20.0, hot_spots=[], fire=fire, qi_scale=1.0)


def test_suppression_zero_max_radius_is_legacy():
    """max_radius=0 (or half_angle=0) ⇒ no suppression applied (regression guard)."""
    f_off = make_suppression_field(max_radius=0.0, center_deg=225.0)
    f_legacy = make_field(v_front=0.5, peak=80.0, rise=5.0, plateau=10.0, decay=15.0)
    f_legacy.fire.arena_radius = 2.0
    xs = np.array([0.5, -0.5, 0.0, 0.0, -1.0, 1.0])
    ys = np.array([0.0,  0.0, 0.5, -0.5, -1.0, -1.0])
    T_off = f_off.evaluate(xs, ys, t=10.0)
    T_legacy = f_legacy.evaluate(xs, ys, t=10.0)
    np.testing.assert_allclose(T_off, T_legacy, atol=1e-3)


def test_suppression_caps_outer_cells_inside_arc():
    """Inside the suppression arc, cells at radius > max_radius are zeroed.
    Cells at radius <= max_radius burn normally."""
    field = make_suppression_field(max_radius=1.0, center_deg=225.0,
                                   half_angle_deg=90.0)
    # cf3 axis (180°) is in the arc. Inner cell (r=0.5) burns; outer (r=1.5) zeroed.
    inner_x, inner_y = -0.5, 0.0   # r=0.5, θ=180°  (in arc, in cap)
    outer_x, outer_y = -1.5, 0.0   # r=1.5, θ=180°  (in arc, beyond cap)
    cf1_outer_x, cf1_outer_y = 1.5, 0.0   # r=1.5, θ=0°  (outside arc, unaffected)
    xs = np.array([inner_x, outer_x, cf1_outer_x])
    ys = np.array([inner_y, outer_y, cf1_outer_y])
    T = field.evaluate(xs, ys, t=10.0)
    # Inner cell in arc still burns (peak temperature reached at t=10s plateau).
    assert T[0] > 20.0 + 70.0, 'inner-arc cell should still be burning'
    # Outer cell in arc capped to ambient.
    assert T[1] == pytest.approx(20.0, abs=1e-3), \
        'outer-arc cell should be capped to ambient'
    # Outer cell OUTSIDE arc still burns (no cap there).
    assert T[2] > 20.0 + 70.0, 'outer cell outside arc should burn normally'


def test_suppression_arc_doesnt_affect_outside():
    """A cell far outside the suppression arc (cf1 axis vs arc centered at 225°)
    is identical to the no-suppression field at the same point."""
    f_with = make_suppression_field(max_radius=1.0, center_deg=225.0,
                                    half_angle_deg=90.0, edge_taper_deg=5.0)
    f_without = make_suppression_field(max_radius=0.0, center_deg=225.0)
    # cf1 axis at 0° is 135° away from arc center 225° — well outside half_angle+taper.
    xs = np.array([1.5])  # outer radius, outside arc
    ys = np.array([0.0])
    T_with = f_with.evaluate(xs, ys, t=10.0)
    T_without = f_without.evaluate(xs, ys, t=10.0)
    assert T_with[0] == pytest.approx(T_without[0], abs=1e-3)


def test_randomize_seed_zero_is_deterministic(tmp_path):
    """randomize_seed=0 ⇒ wind direction + ignition point + arc center unchanged."""
    yaml_text = """ambient: 22.0
hot_spots: []
fire:
  ignition_point: [0.0, 0.0]
  v_front: 0.01
  peak_temperature: 80.0
  burn: {rise_duration: 10.0, plateau_duration: 15.0, decay_duration: 25.0}
  arena_radius: 1.9
  wind: {direction_deg: 0.0, rotation_rate_deg_per_sec: 0.5, anisotropy: 0.8}
  suppression: {center_deg: 225.0, half_angle_deg: 90.0, max_radius: 1.0}
  randomize_seed: 0
qi: {scale: 0.18}
"""
    f = tmp_path / 'field.yaml'
    f.write_text(yaml_text)
    field = ThermalField.from_yaml(str(f))
    assert field.fire.wind_direction_deg == 0.0
    assert field.fire.ignition_x == 0.0
    assert field.fire.ignition_y == 0.0
    assert field.fire.suppression_center_deg == 225.0
    assert field.fire.suppression_max_radius == 1.0


def test_randomize_seed_nonzero_rotates_wind_and_arc(tmp_path):
    """randomize_seed > 0 ⇒ wind direction + arc center rotated by the SAME
    seeded amount (wind-locked), so dominant-vs-suppressed labels stay
    consistent across seeds."""
    yaml_template = """ambient: 22.0
hot_spots: []
fire:
  ignition_point: [0.0, 0.0]
  v_front: 0.01
  peak_temperature: 80.0
  burn: {{rise_duration: 10.0, plateau_duration: 15.0, decay_duration: 25.0}}
  arena_radius: 1.9
  wind: {{direction_deg: 0.0, rotation_rate_deg_per_sec: 0.5, anisotropy: 0.8}}
  suppression: {{center_deg: 225.0, half_angle_deg: 90.0, max_radius: 1.0}}
  randomize_seed: {seed}
  ignition_jitter_radius: 0.15
qi: {{scale: 0.18}}
"""
    rel_offsets = []
    for seed in [1, 2, 3]:
        f = tmp_path / f'field_{seed}.yaml'
        f.write_text(yaml_template.format(seed=seed))
        field = ThermalField.from_yaml(str(f))
        rel = (field.fire.suppression_center_deg - field.fire.wind_direction_deg) % 360.0
        rel_offsets.append(rel)
        r = np.hypot(field.fire.ignition_x, field.fire.ignition_y)
        assert r <= 0.15 + 1e-6, f'seed={seed}: ignition jitter exceeds disc'
    for rel in rel_offsets:
        assert rel == pytest.approx(225.0, abs=1e-3)
