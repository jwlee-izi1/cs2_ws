"""Unit tests for the rate-limited setpoint step (the snap-back protection)."""

import math

from cf_coverage_planner.setpoint_streamer_node import _step_toward


def _dist(a, b):
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def test_step_caps_at_max_step():
    out = _step_toward((0.0, 0.0, 1.0), (10.0, 0.0, 1.0), max_step=0.05)
    assert math.isclose(_dist((0.0, 0.0, 1.0), out), 0.05, rel_tol=1e-6)


def test_step_reaches_target_when_close():
    out = _step_toward((0.0, 0.0, 1.0), (0.01, 0.0, 1.0), max_step=0.05)
    assert math.isclose(out[0], 0.01)
    assert math.isclose(out[1], 0.0)


def test_snap_back_smoothed():
    """A 1.5 m policy-target retraction at v=1 m/s and 20 Hz should take 30 ticks."""
    current = (2.0, 0.0, 1.0)
    target = (0.5, 0.0, 1.0)
    max_step = 1.0 / 20.0  # 0.05 m/tick
    ticks = 0
    while _dist(current, target) > 1e-6 and ticks < 100:
        current = _step_toward(current, target, max_step)
        ticks += 1
    # 1.5 m / 0.05 m/tick = 30 ticks
    assert ticks == 30


def test_zero_distance_no_change():
    out = _step_toward((1.0, 1.0, 1.0), (1.0, 1.0, 1.0), max_step=0.05)
    assert out == (1.0, 1.0, 1.0)
