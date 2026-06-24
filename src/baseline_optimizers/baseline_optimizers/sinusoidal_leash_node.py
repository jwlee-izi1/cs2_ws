"""Publishes a sinusoidally-varying leash on /coverage/leash.

  r_i(t) = 0.5 * r_max_i * (1 + sin(omega * t + phase_i))

with phases offset by pi/2 between drones, so retraction events are visible
on different drones at different times. Tests the planner's expand/retract logic.
"""

import math

import rclpy
from coverage_optimizer_interfaces.msg import Leash
from rclpy.node import Node


class SinusoidalLeashNode(Node):

    def __init__(self):
        super().__init__('sinusoidal_leash_node')

        self.declare_parameter('drone_names', ['cf1', 'cf2', 'cf3', 'cf4'])
        self.declare_parameter('r_max_per_drone', [2.3, 2.3, 2.3, 2.3])
        self.declare_parameter('period_sec', 20.0)
        self.declare_parameter('round_rate_hz', 2.0)

        self.drone_names = list(self.get_parameter('drone_names').value)
        self.r_max = [float(r) for r in self.get_parameter('r_max_per_drone').value]
        self.period = float(self.get_parameter('period_sec').value)
        rate_hz = float(self.get_parameter('round_rate_hz').value)

        if len(self.r_max) != len(self.drone_names):
            raise ValueError(
                f'r_max length {len(self.r_max)} != drone_names length {len(self.drone_names)}')

        self.omega = 2.0 * math.pi / self.period
        n = len(self.drone_names)
        self.phases = [k * (math.pi / 2.0) for k in range(n)]
        self.t0 = self.get_clock().now()

        self.pub = self.create_publisher(Leash, '/coverage/leash', 10)
        self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f'SinusoidalLeashNode: drones={self.drone_names} period={self.period}s')

    def _tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        radii = [
            0.5 * rmax_i * (1.0 + math.sin(self.omega * t + ph_i))
            for rmax_i, ph_i in zip(self.r_max, self.phases)
        ]

        msg = Leash()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drone_names = self.drone_names
        msg.radii = [float(r) for r in radii]
        msg.gate_state = 1
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SinusoidalLeashNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
