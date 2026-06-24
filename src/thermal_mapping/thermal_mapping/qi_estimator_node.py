"""Oracle q_i estimator ROS node.

Reads the same `thermal_field.yaml` the sensors do AND the sector geometry from
the optimizer's `arena_4drone.yaml`. At a configurable rate, evaluates
`compute_sector_qi(field, t, sectors)` and publishes the result on
`/coverage/sector_weights` (Float64MultiArray) so the optimizer can pick up
non-stationary `q_i` values each round.

Time axis: t is measured in seconds since this node's first tick, matching
the convention used by `thermal_sensor_node` (each node anchors its own t0).
This means the fire's `ignition_time` in YAML is interpreted relative to the
demo's wall-clock start.

The published array order matches `arena.drone_names` from arena_4drone.yaml,
which is the same order the optimizer's q array uses.
"""

import math

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from thermal_mapping.qi_estimator import SectorGeom, compute_sector_qi
from thermal_mapping.thermal_field import ThermalField


class QiEstimatorNode(Node):

    def __init__(self):
        super().__init__('qi_estimator_node')

        self.declare_parameter('field_yaml', '')
        self.declare_parameter('arena_yaml', '')
        self.declare_parameter('rate_hz', 1.0)
        self.declare_parameter('grid_resolution', 0.05)

        field_yaml = self.get_parameter('field_yaml').value
        arena_yaml = self.get_parameter('arena_yaml').value
        rate_hz = float(self.get_parameter('rate_hz').value)
        self.grid_resolution = float(self.get_parameter('grid_resolution').value)

        if not field_yaml:
            raise RuntimeError('Parameter field_yaml must be set to a path.')
        if not arena_yaml:
            raise RuntimeError('Parameter arena_yaml must be set to a path.')

        self.field = ThermalField.from_yaml(field_yaml)
        self.drone_names, self.sectors = self._load_sectors(arena_yaml)
        self._t0: float | None = None

        self.pub = self.create_publisher(
            Float64MultiArray, '/coverage/sector_weights', 10,
        )
        self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f'qi_estimator up: rate={rate_hz}Hz, '
            f'drones={self.drone_names}, '
            f'r_outer={[round(s.r_outer, 2) for s in self.sectors]}, '
            f'qi_scale={self.field.qi_scale}'
        )

    def _load_sectors(self, arena_yaml: str):
        with open(arena_yaml, 'r') as f:
            doc = yaml.safe_load(f)
        arena = doc.get('arena', {})
        center_xy = tuple(arena.get('center_xy', [0.0, 0.0]))
        drone_names = list(doc.get('drone_names', []))
        sectors_doc = doc.get('sectors', {})

        # r_outer for q_i integration uses the fire model's arena_radius so the
        # wedge covers exactly the fuel disc (matches the offline previewer
        # convention after the dilution-bug fix on 2026-05-26).
        if self.field.fire is not None and self.field.fire.arena_radius > 0.0:
            r_outer = float(self.field.fire.arena_radius)
        else:
            # Fall back to per-drone r_max from YAML if no fire model present.
            r_outer = max(float(sectors_doc[n]['r_max']) for n in drone_names)

        sectors = []
        for name in drone_names:
            entry = sectors_doc[name]
            sectors.append(SectorGeom(
                name=name,
                phi_mid=math.radians(float(entry['phi_mid_deg'])),
                phi_half=math.radians(float(entry['phi_half_deg'])),
                r_outer=r_outer,
                center_xy=center_xy,
            ))
        return drone_names, sectors

    def _tick(self) -> None:
        now_msg = self.get_clock().now().to_msg()
        now_sec = now_msg.sec + now_msg.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = now_sec
        scenario_t = now_sec - self._t0

        qi = compute_sector_qi(
            self.field,
            scenario_t,
            self.sectors,
            grid_resolution=self.grid_resolution,
        )

        msg = Float64MultiArray()
        dim = MultiArrayDimension()
        dim.label = 'drone'
        dim.size = len(qi)
        dim.stride = len(qi)
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0
        msg.data = [float(v) for v in qi]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = QiEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
