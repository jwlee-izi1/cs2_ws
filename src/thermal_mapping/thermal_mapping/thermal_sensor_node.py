"""Per-drone simulated downward-facing thermal sensor.

Subscribes to /<drone_name>/odom, samples a YAML-defined truth field on a
square grid below the drone, adds Gaussian noise, and publishes a
ThermalFrame on /<drone_name>/thermal/raw at a fixed rate.

The drone's pose at the sample instant is embedded in every message so the
mapper does not need to time-sync against /odom.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from thermal_mapping_interfaces.msg import ThermalFrame

from thermal_mapping.thermal_field import ThermalField


class ThermalSensorNode(Node):

    def __init__(self):
        super().__init__('thermal_sensor')

        self.declare_parameter('drone_name', 'cf1')
        self.declare_parameter('field_yaml', '')
        self.declare_parameter('fov_deg', 45.0)
        self.declare_parameter('resolution', 16)
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('min_altitude', 0.2)
        self.declare_parameter('noise_sigma', 0.5)

        self.drone_name = self.get_parameter('drone_name').value
        field_yaml = self.get_parameter('field_yaml').value
        self.fov_deg = float(self.get_parameter('fov_deg').value)
        self.res = int(self.get_parameter('resolution').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.min_alt = float(self.get_parameter('min_altitude').value)
        self.noise_sigma = float(self.get_parameter('noise_sigma').value)

        if not field_yaml:
            raise RuntimeError('Parameter field_yaml must be set to a path.')
        self.field = ThermalField.from_yaml(field_yaml)
        self.tan_half_fov = math.tan(math.radians(self.fov_deg) * 0.5)

        # Fire model treats t=0 as ignition. Anchor "scenario t=0" to the
        # first tick after the node comes up so /thermal_truth, q_i, and
        # sensors all share one timeline.
        self._t0: float | None = None

        # Pre-built unit-grid coordinates in [-1, +1] across the sensor; scaled
        # by altitude on each tick. linspace endpoints are pixel CENTERS.
        u = (np.arange(self.res, dtype=np.float32) + 0.5) / self.res * 2.0 - 1.0
        self._gx, self._gy = np.meshgrid(u, u, indexing='xy')

        self._latest_odom: Odometry | None = None

        self.create_subscription(
            Odometry,
            f'/{self.drone_name}/odom',
            self._on_odom,
            qos_profile_sensor_data,
        )
        self.pub = self.create_publisher(
            ThermalFrame,
            f'/{self.drone_name}/thermal/raw',
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / self.rate_hz, self._on_tick)

        self.get_logger().info(
            f'thermal_sensor[{self.drone_name}] up: '
            f'FOV={self.fov_deg}deg, {self.res}x{self.res}, '
            f'{self.rate_hz}Hz, min_alt={self.min_alt}m, '
            f'noise_sigma={self.noise_sigma}, hot_spots={len(self.field.hot_spots)}'
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _on_tick(self) -> None:
        odom = self._latest_odom
        if odom is None:
            return
        px = odom.pose.pose.position.x
        py = odom.pose.pose.position.y
        pz = odom.pose.pose.position.z
        if pz < self.min_alt:
            return

        h = pz * self.tan_half_fov  # half ground-footprint width [m]
        xs = px + self._gx * h
        ys = py + self._gy * h
        now_msg = self.get_clock().now().to_msg()
        now_sec = now_msg.sec + now_msg.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = now_sec
        scenario_t = now_sec - self._t0
        truth = self.field.evaluate(xs, ys, scenario_t)
        noise = np.random.normal(
            0.0, self.noise_sigma, size=truth.shape
        ).astype(np.float32)
        sample = truth + noise

        out = ThermalFrame()
        out.header.stamp = now_msg
        out.header.frame_id = 'world'
        out.pose = odom.pose.pose
        out.footprint_size = float(2.0 * h)
        out.width = self.res
        out.height = self.res
        out.data = sample.ravel().tolist()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ThermalSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
