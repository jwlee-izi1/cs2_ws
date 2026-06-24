"""Publishes a constant leash vector on /coverage/leash.

Use to test the planner standalone — all drones get to expand to a fixed radius.
"""

import rclpy
from coverage_optimizer_interfaces.msg import Leash
from rclpy.node import Node


class ConstantLeashNode(Node):

    def __init__(self):
        super().__init__('constant_leash_node')

        self.declare_parameter('drone_names', ['cf1', 'cf2', 'cf3', 'cf4'])
        self.declare_parameter('radii', [2.0, 2.0, 2.0, 2.0])
        self.declare_parameter('round_rate_hz', 2.0)

        self.drone_names = list(self.get_parameter('drone_names').value)
        self.radii = [float(r) for r in self.get_parameter('radii').value]
        rate_hz = float(self.get_parameter('round_rate_hz').value)

        if len(self.radii) != len(self.drone_names):
            raise ValueError(
                f'radii length {len(self.radii)} != drone_names length {len(self.drone_names)}')

        self.pub = self.create_publisher(Leash, '/coverage/leash', 10)
        self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f'ConstantLeashNode: drones={self.drone_names} radii={self.radii} '
            f'@ {rate_hz} Hz')

    def _tick(self):
        msg = Leash()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drone_names = self.drone_names
        msg.radii = self.radii
        msg.gate_state = 1  # always feasible
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConstantLeashNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
