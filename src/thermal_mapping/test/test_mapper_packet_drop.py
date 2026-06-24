"""Tests for the mapper-side packet-drop visualization (Bucket C).

Synthetic ThermalFrames are injected directly into ThermalMapperNode._on_thermal.
We do not spin the executor — we just call the callback and check buffer state.

Asserted behavior:
- packet_drop_prob=0   → thermal layer written, dropped layer stays 0.
- packet_drop_prob=1   → thermal layer untouched (still NaN), dropped layer set
                         to 1.0 at the drone's CURRENT footprint cells (derived
                         from msg.pose, NOT any optimizer leash).
- A later accepted frame at the same footprint clears the dropped marker.
- Per-drone RNG streams are independent.
- packet_drop_prob=0 doesn't consume any RNG draws (backward-compat path).
"""

import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))

rclpy = pytest.importorskip('rclpy')
ThermalFrame = pytest.importorskip(
    'thermal_mapping_interfaces.msg'
).ThermalFrame

from rclpy.parameter import Parameter

from thermal_mapping.thermal_mapper_node import ThermalMapperNode


@pytest.fixture(scope='module', autouse=True)
def _rclpy_lifecycle():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def _make_node(packet_drop_prob: float, seed: int = 42) -> ThermalMapperNode:
    """Real ThermalMapperNode with a small grid for fast tests."""

    class _ParamMapperNode(ThermalMapperNode):
        def __init__(self):
            from rclpy.node import Node as _Node
            # Bypass ThermalMapperNode.__init__ until parameter overrides are in
            # place; then run the real init body via super().
            _Node.__init__(
                self,
                f'thermal_mapper_test_{id(self)}',
                parameter_overrides=[
                    Parameter('drone_names', Parameter.Type.STRING_ARRAY, ['cf1', 'cf2']),
                    Parameter('frame_id', Parameter.Type.STRING, 'world'),
                    Parameter('length_x', Parameter.Type.DOUBLE, 2.0),
                    Parameter('length_y', Parameter.Type.DOUBLE, 2.0),
                    Parameter('resolution', Parameter.Type.DOUBLE, 0.05),
                    Parameter('center_x', Parameter.Type.DOUBLE, 0.0),
                    Parameter('center_y', Parameter.Type.DOUBLE, 0.0),
                    Parameter('publish_rate_hz', Parameter.Type.DOUBLE, 2.0),
                    Parameter('packet_drop_prob', Parameter.Type.DOUBLE, packet_drop_prob),
                    Parameter('drop_seed', Parameter.Type.INTEGER, seed),
                ],
            )
            # Now manually do what ThermalMapperNode.__init__ does after
            # super().__init__ — declare params (overrides take effect), then
            # build the buffers.
            import math  # noqa
            import random
            from thermal_mapping.grid_map_helpers import grid_shape, make_info

            self.declare_parameter('drone_names', ['cf1', 'cf2', 'cf3', 'cf4'])
            self.declare_parameter('frame_id', 'world')
            self.declare_parameter('length_x', 5.0)
            self.declare_parameter('length_y', 5.0)
            self.declare_parameter('resolution', 0.01)
            self.declare_parameter('center_x', 0.0)
            self.declare_parameter('center_y', 0.0)
            self.declare_parameter('publish_rate_hz', 2.0)
            self.declare_parameter('packet_drop_prob', 0.0)
            self.declare_parameter('drop_seed', 42)

            drone_names = list(self.get_parameter('drone_names').value)
            self.frame_id = str(self.get_parameter('frame_id').value)
            length_x = float(self.get_parameter('length_x').value)
            length_y = float(self.get_parameter('length_y').value)
            resolution = float(self.get_parameter('resolution').value)
            center_xy = (
                float(self.get_parameter('center_x').value),
                float(self.get_parameter('center_y').value),
            )
            self.packet_drop_prob = float(
                self.get_parameter('packet_drop_prob').value
            )
            drop_seed = int(self.get_parameter('drop_seed').value)
            self._drop_rngs = {
                n: random.Random(drop_seed + i) for i, n in enumerate(drone_names)
            }

            self.info = make_info(length_x, length_y, resolution, center_xy)
            self.n_rows, self.n_cols = grid_shape(self.info)
            nan = np.float32(np.nan)
            self._value = np.full((self.n_rows, self.n_cols), nan, dtype=np.float32)
            self._last_update_time = np.full(
                (self.n_rows, self.n_cols), nan, dtype=np.float32
            )
            self._dropped = np.zeros((self.n_rows, self.n_cols), dtype=np.float32)

    return _ParamMapperNode()


def _make_frame(px: float, py: float, footprint_size: float = 0.2,
                width: int = 8, temp: float = 50.0) -> ThermalFrame:
    msg = ThermalFrame()
    msg.header.stamp.sec = 100
    msg.header.stamp.nanosec = 0
    msg.header.frame_id = 'world'
    msg.pose.position.x = float(px)
    msg.pose.position.y = float(py)
    msg.pose.position.z = 1.0
    msg.pose.orientation.w = 1.0
    msg.footprint_size = float(footprint_size)
    msg.width = width
    msg.height = width
    msg.data = [float(temp)] * (width * width)
    return msg


def test_no_drops_writes_thermal_leaves_dropped_zero():
    """packet_drop_prob=0: thermal cells get the value, dropped stays all-zero."""
    node = _make_node(packet_drop_prob=0.0)
    node._on_thermal('cf1', _make_frame(px=0.3, py=0.3, temp=42.0))
    # Some cells must now hold 42.0 (the frame's temperature).
    n_written = int(np.sum(node._value == 42.0))
    assert n_written > 0, f'expected some cells written, got {n_written}'
    # The cells that were written must NOT be marked as dropped.
    written_mask = (node._value == 42.0)
    assert np.all(node._dropped[written_mask] == 0.0)
    # And dropped should be all zero everywhere.
    assert np.count_nonzero(node._dropped) == 0
    node.destroy_node()


def test_full_drop_keeps_thermal_nan_sets_dropped_one():
    """packet_drop_prob=1: NO thermal writes; dropped layer set on the footprint."""
    node = _make_node(packet_drop_prob=1.0)
    node._on_thermal('cf1', _make_frame(px=-0.4, py=0.2, temp=80.0))
    # Thermal layer untouched — still all NaN.
    assert np.all(np.isnan(node._value))
    # Dropped layer has SOMETHING — the drone's footprint.
    n_dropped_cells = int(np.sum(node._dropped == 1.0))
    assert n_dropped_cells > 0, f'expected dropped cells, got {n_dropped_cells}'
    node.destroy_node()


def test_accepted_frame_clears_prior_dropped_marker():
    """Last-write-wins: an accepted frame at a previously-dropped footprint
    clears the marker and writes the thermal value."""
    node = _make_node(packet_drop_prob=1.0)
    node._on_thermal('cf1', _make_frame(px=0.1, py=0.1, temp=60.0))
    dropped_cells = (node._dropped == 1.0)
    assert dropped_cells.any()

    node.packet_drop_prob = 0.0
    node._on_thermal('cf1', _make_frame(px=0.1, py=0.1, temp=60.0))
    # Every cell that was dropped before must now read 60.0 thermally and 0.0 dropped.
    assert np.all(node._dropped[dropped_cells] == 0.0)
    assert np.allclose(node._value[dropped_cells], 60.0)
    node.destroy_node()


def test_drop_marker_lands_at_drone_current_pose_not_origin():
    """Drop marker location must follow msg.pose (drone's CURRENT pose),
    not (0,0) or any other fixed reference. Send two frames at different
    poses, confirm the dropped cells move with the drone."""
    node = _make_node(packet_drop_prob=1.0)
    node._on_thermal('cf1', _make_frame(px=0.5, py=0.5, temp=10.0))
    mask_a = (node._dropped == 1.0).copy()
    # Reset and try a different pose.
    node._dropped[:] = 0.0
    node._on_thermal('cf1', _make_frame(px=-0.5, py=-0.5, temp=10.0))
    mask_b = (node._dropped == 1.0)
    # Both must mark some cells, and the regions must NOT overlap (different pose).
    assert mask_a.any() and mask_b.any()
    overlap = int(np.sum(mask_a & mask_b))
    assert overlap == 0, f'pose 0.5,0.5 and -0.5,-0.5 should not overlap, got {overlap}'
    # And the centroids of the two masks should be ~1.0 m apart, confirming
    # they track the pose.
    rows_a, cols_a = np.where(mask_a)
    rows_b, cols_b = np.where(mask_b)
    centroid_a_row = rows_a.mean()
    centroid_b_row = rows_b.mean()
    centroid_a_col = cols_a.mean()
    centroid_b_col = cols_b.mean()
    res = node.info.resolution
    # (px=0.5, py=0.5) → small row, small col; (-0.5,-0.5) → large row, large col.
    # World→grid is monotone-decreasing in both axes, so swapping signs of px,py
    # produces a centroid shift in the opposite direction. Magnitude ≈ 1 m.
    row_shift_m = (centroid_b_row - centroid_a_row) * res
    col_shift_m = (centroid_b_col - centroid_a_col) * res
    distance_m = np.sqrt(row_shift_m**2 + col_shift_m**2)
    # Expected: sqrt((0.5-(-0.5))^2 + (0.5-(-0.5))^2) = sqrt(2) ≈ 1.414 m
    assert 1.3 < distance_m < 1.55, \
        f'expected ~1.414m centroid shift, got {distance_m:.3f}m'
    node.destroy_node()


def test_per_drone_rng_independence():
    a = _make_node(packet_drop_prob=0.5, seed=123)
    b = _make_node(packet_drop_prob=0.5, seed=123)
    for _ in range(20):
        a._on_thermal('cf1', _make_frame(px=0.0, py=0.0, temp=30.0))
    seq_a = [a._drop_rngs['cf2'].random() for _ in range(10)]
    seq_b = [b._drop_rngs['cf2'].random() for _ in range(10)]
    assert seq_a == seq_b
    a.destroy_node()
    b.destroy_node()


def test_zero_drop_rate_does_not_consume_rng():
    """packet_drop_prob=0 should not even roll the dice."""
    node = _make_node(packet_drop_prob=0.0, seed=7)
    before = node._drop_rngs['cf1'].getstate()
    for _ in range(50):
        node._on_thermal('cf1', _make_frame(px=0.2, py=-0.1, temp=25.0))
    after = node._drop_rngs['cf1'].getstate()
    assert before == after
    assert np.count_nonzero(node._dropped) == 0
    node.destroy_node()
