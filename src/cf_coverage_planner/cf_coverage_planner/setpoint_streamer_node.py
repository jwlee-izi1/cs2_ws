"""Per-drone rate-limited setpoint streamer.

Subscribes to /cfN/policy_target (PoseStamped, ~5 Hz from whatever policy is running) and
republishes /cfN/cmd_position (crazyflie_interfaces/Position, 20 Hz) with the commanded
setpoint walking toward the target at most max_setpoint_velocity / setpoint_rate_hz per tick.

This is the actuation layer that SURVIVES the RL swap. The throwaway lawnmower planner
publishes targets here today; an RL policy node publishes the same targets later.

Smooths snap-back retractions: when the policy target jumps far in one tick, the firmware
sees a setpoint that walks at <= max_setpoint_velocity rather than a teleport.
"""

import math

import rclpy
from crazyflie_interfaces.msg import Position
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool


class SetpointStreamer(Node):

    def __init__(self):
        super().__init__('setpoint_streamer')

        self.declare_parameter('drone_name', 'cf1')
        self.declare_parameter('setpoint_rate_hz', 20.0)
        self.declare_parameter('max_setpoint_velocity', 1.0)

        self.drone_name = str(self.get_parameter('drone_name').value)
        self.rate_hz = float(self.get_parameter('setpoint_rate_hz').value)
        self.max_velocity = float(self.get_parameter('max_setpoint_velocity').value)
        self.max_step = self.max_velocity / self.rate_hz

        self.current_setpoint = None   # (x, y, z) — populated from first odom
        self.policy_target = None      # (x, y, z) — latest target from policy

        self.create_subscription(
            Odometry, f'/{self.drone_name}/odom', self._odom_cb, 10)
        self.create_subscription(
            PoseStamped, f'/{self.drone_name}/policy_target', self._target_cb, 10)
        # /demo/stopped (latched): when True, STOP publishing cmd_position so the optimizer's
        # land command isn't overridden by our low-level setpoint (the firmware prioritizes the
        # most-recent low-level setpoint over the high-level Land). Mirrors polynomial_streamer.
        self._demo_stopped = False
        self.create_subscription(
            Bool, '/demo/stopped', self._on_demo_stopped,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self.pub = self.create_publisher(
            Position, f'/{self.drone_name}/cmd_position', 10)

        self.create_timer(1.0 / self.rate_hz, self._tick)

        self.get_logger().info(
            f'SetpointStreamer({self.drone_name}): rate={self.rate_hz} Hz, '
            f'v_max={self.max_velocity} m/s, step_cap={self.max_step:.3f} m/tick')

    def _odom_cb(self, msg: Odometry):
        if self.current_setpoint is None:
            p = msg.pose.pose.position
            self.current_setpoint = (p.x, p.y, p.z)
            self.get_logger().info(
                f'{self.drone_name}: initial setpoint = '
                f'({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')

    def _target_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self.policy_target = (p.x, p.y, p.z)

    def _on_demo_stopped(self, msg: Bool):
        if msg.data and not self._demo_stopped:
            self._demo_stopped = True
            self.get_logger().info(
                '/demo/stopped=True → halting cmd_position (let the optimizer Land the drone)')

    def _tick(self):
        if self._demo_stopped:
            return   # demo over — stop commanding so the high-level Land takes effect
        if self.current_setpoint is None:
            return
        if self.policy_target is not None:
            self.current_setpoint = _step_toward(
                self.current_setpoint, self.policy_target, self.max_step)

        x, y, z = self.current_setpoint
        out = Position()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'world'
        out.x = float(x)
        out.y = float(y)
        out.z = float(z)
        out.yaw = 0.0
        self.pub.publish(out)


def _step_toward(current, target, max_step):
    """Walk current toward target by at most max_step (Euclidean)."""
    dx = target[0] - current[0]
    dy = target[1] - current[1]
    dz = target[2] - current[2]
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    if dist <= max_step or dist == 0.0:
        return tuple(target)
    scale = max_step / dist
    return (current[0] + dx * scale,
            current[1] + dy * scale,
            current[2] + dz * scale)


def main(args=None):
    rclpy.init(args=args)
    node = SetpointStreamer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
