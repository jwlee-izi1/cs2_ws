"""Full-state streamer for the RL coverage planner.

Replaces ``setpoint_streamer_node`` (which sent only cmd_position and let the
firmware estimate velocity from position-error — causing PI integrator
wind-up under sustained tracking error). This node sends pos + vel +
acceleration to ``/cfN/cmd_full_state`` so the firmware uses the velocity
as feedforward instead of computing it from error. No more wind-up.

Pipeline:
  /cfN/policy_target (PoseStamped @ 5 Hz from rl_planner_node)
      ↓ this node
  /cfN/cmd_full_state (FullState @ 20 Hz to crazyflie firmware)

Velocity = (policy_target_now - policy_target_prev) / dt_between_targets
Acceleration = (velocity_now - velocity_prev) / dt_between_targets
Both are low-pass filtered (EMA) to suppress numerical noise.

Between successive policy targets (200 ms gap), we interpolate position
linearly toward the latest target at the streamer rate. Velocity and
acceleration are held constant within the inter-policy interval.
"""

from __future__ import annotations

import math
import threading

import rclpy
from crazyflie_interfaces.msg import FullState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class FullStateStreamerNode(Node):

    def __init__(self):
        super().__init__('fullstate_streamer')

        self.declare_parameter('drone_name', 'cf1')
        self.declare_parameter('setpoint_rate_hz', 20.0)
        self.declare_parameter('max_velocity', 1.0)
        # EMA coefficient on velocity/acc. 1.0 = no smoothing; 0.3 = heavy smoothing.
        self.declare_parameter('feedforward_smooth', 0.5)

        self.drone_name = str(self.get_parameter('drone_name').value)
        self.rate_hz = float(self.get_parameter('setpoint_rate_hz').value)
        self.max_velocity = float(self.get_parameter('max_velocity').value)
        self.ff_smooth = float(self.get_parameter('feedforward_smooth').value)

        # State
        self._lock = threading.Lock()
        self._current_pos: tuple[float, float, float] | None = None
        # Policy targets and derived feedforward
        self._prev_target: tuple[float, float, float] | None = None
        self._latest_target: tuple[float, float, float] | None = None
        self._prev_target_t: float | None = None
        self._latest_target_t: float | None = None
        self._vel_smooth: list[float] = [0.0, 0.0, 0.0]
        self._acc_smooth: list[float] = [0.0, 0.0, 0.0]

        # Subscriptions
        self.create_subscription(Odometry, f'/{self.drone_name}/odom',
                                 self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, f'/{self.drone_name}/policy_target',
                                 self._target_cb, 10)

        # Publisher + timer
        self.pub = self.create_publisher(
            FullState, f'/{self.drone_name}/cmd_full_state', 10)
        self.create_timer(1.0 / self.rate_hz, self._tick)

        self.get_logger().info(
            f'FullStateStreamer({self.drone_name}): rate={self.rate_hz} Hz, '
            f'v_max={self.max_velocity} m/s, ff_smooth={self.ff_smooth}')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            if self._current_pos is None:
                p = msg.pose.pose.position
                self._current_pos = (p.x, p.y, p.z)

    def _target_cb(self, msg: PoseStamped):
        t = self._now()
        p = msg.pose.position
        with self._lock:
            self._prev_target = self._latest_target
            self._prev_target_t = self._latest_target_t
            self._latest_target = (p.x, p.y, p.z)
            self._latest_target_t = t

            # Compute feedforward velocity and acceleration when we have
            # at least one previous target.
            if self._prev_target is not None and self._prev_target_t is not None:
                dt = max(self._latest_target_t - self._prev_target_t, 1e-3)
                vx = (self._latest_target[0] - self._prev_target[0]) / dt
                vy = (self._latest_target[1] - self._prev_target[1]) / dt
                vz = (self._latest_target[2] - self._prev_target[2]) / dt
                # Clamp to velocity envelope.
                v_mag = math.sqrt(vx * vx + vy * vy)
                if v_mag > self.max_velocity and v_mag > 1e-6:
                    scale = self.max_velocity / v_mag
                    vx *= scale; vy *= scale
                # EMA smoothing on velocity
                a = self.ff_smooth
                old_vx, old_vy, old_vz = self._vel_smooth
                new_vx = a * vx + (1 - a) * old_vx
                new_vy = a * vy + (1 - a) * old_vy
                new_vz = a * vz + (1 - a) * old_vz
                # Acceleration from velocity diff.
                ax = (new_vx - old_vx) / dt
                ay = (new_vy - old_vy) / dt
                az = (new_vz - old_vz) / dt
                # EMA on acceleration too.
                old_ax, old_ay, old_az = self._acc_smooth
                new_ax = a * ax + (1 - a) * old_ax
                new_ay = a * ay + (1 - a) * old_ay
                new_az = a * az + (1 - a) * old_az
                self._vel_smooth = [new_vx, new_vy, new_vz]
                self._acc_smooth = [new_ax, new_ay, new_az]

    def _tick(self):
        with self._lock:
            if self._current_pos is None:
                return
            if self._latest_target is None:
                # Hold position until policy starts publishing.
                tx, ty, tz = self._current_pos
                vx, vy, vz = 0.0, 0.0, 0.0
                ax, ay, az = 0.0, 0.0, 0.0
            else:
                tx, ty, tz = self._latest_target
                vx, vy, vz = self._vel_smooth
                ax, ay, az = self._acc_smooth

        out = FullState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'world'
        out.pose.position.x = float(tx)
        out.pose.position.y = float(ty)
        out.pose.position.z = float(tz)
        # Identity quaternion (no yaw)
        out.pose.orientation.w = 1.0
        out.twist.linear.x = float(vx)
        out.twist.linear.y = float(vy)
        out.twist.linear.z = float(vz)
        # Angular twist stays zero
        out.acc.x = float(ax)
        out.acc.y = float(ay)
        out.acc.z = float(az)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FullStateStreamerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
