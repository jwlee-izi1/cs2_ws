"""Publishes the full ground-truth thermal field T(x, y, t) as a GridMap.

This is a visualization/eval-only node. It samples the same ThermalField the
sensors use, but on a fixed regular grid covering the arena, and pushes the
result onto /thermal_truth at a fixed rate. Sensors do NOT subscribe — they
still evaluate T in-process — but RViz, rosbag, and the future RL evaluator
can overlay the truth on top of the mapper's noisy estimate.

Time axis matches the sensors' convention: t = seconds since this node's
first tick. Tiny startup-offset drift across nodes (~ms) is acceptable
against a 360s scenario.
"""

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from thermal_mapping.grid_map_helpers import (
    grid_shape,
    make_grid_map,
    make_info,
)
from thermal_mapping.thermal_field import ThermalField


class ThermalGroundTruthNode(Node):

    def __init__(self):
        super().__init__('thermal_ground_truth')

        self.declare_parameter('field_yaml', '')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('length_x', 5.0)
        self.declare_parameter('length_y', 5.0)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('publish_rate_hz', 2.0)

        field_yaml = self.get_parameter('field_yaml').value
        if not field_yaml:
            raise RuntimeError('Parameter field_yaml must be set to a path.')
        self.field = ThermalField.from_yaml(field_yaml)

        self.frame_id = str(self.get_parameter('frame_id').value)
        length_x = float(self.get_parameter('length_x').value)
        length_y = float(self.get_parameter('length_y').value)
        resolution = float(self.get_parameter('resolution').value)
        center_xy = (
            float(self.get_parameter('center_x').value),
            float(self.get_parameter('center_y').value),
        )
        rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.info = make_info(length_x, length_y, resolution, center_xy)
        self.n_rows, self.n_cols = grid_shape(self.info)

        # Precompute per-cell world coordinates. GridMap row=0/col=0 sits at
        # largest x, largest y; row++ → x decreases, col++ → y decreases.
        rows = np.arange(self.n_rows, dtype=np.float32)
        cols = np.arange(self.n_cols, dtype=np.float32)
        col_grid, row_grid = np.meshgrid(cols, rows, indexing='xy')
        self._xs = (
            center_xy[0] + length_x * 0.5 - (row_grid + 0.5) * resolution
        ).astype(np.float32)
        self._ys = (
            center_xy[1] + length_y * 0.5 - (col_grid + 0.5) * resolution
        ).astype(np.float32)

        self._t0: float | None = None

        out_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(GridMap, '/thermal_truth', out_qos)
        self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f'thermal_ground_truth up: {self.n_rows}x{self.n_cols} cells '
            f'({length_x}x{length_y}m @ {resolution}m), publish={rate_hz}Hz'
        )

    def _tick(self) -> None:
        now_msg = self.get_clock().now().to_msg()
        now_sec = now_msg.sec + now_msg.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = now_sec
        scenario_t = now_sec - self._t0

        truth = self.field.evaluate(self._xs, self._ys, scenario_t)
        msg = make_grid_map(
            self.info,
            frame_id=self.frame_id,
            layers={'thermal_truth': truth.astype(np.float32)},
            basic_layers=['thermal_truth'],
            stamp=now_msg,
        )
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ThermalGroundTruthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
