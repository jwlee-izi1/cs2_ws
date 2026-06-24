"""Trajectory streamer — replaces setpoint_streamer for the RL planner.

Instead of rate-limiting policy_target into cmd_position at 20 Hz, this node
calls the firmware's ``/cfN/go_to`` service every time a new policy_target
arrives. The firmware generates its own 7-degree polynomial trajectory from
the drone's current state to the target over ``go_to_duration`` seconds and
tracks it with feedforward, so there's no streamer-induced overshoot and the
drone follows a smooth path between commanded points.

Subscribes:
  /cfN/policy_target  (geometry_msgs/PoseStamped) — RL planner's target.

Calls:
  /cfN/go_to  (crazyflie_interfaces/srv/GoTo) — async, fire-and-forget.

Why not cmd_position rate-limited?
  cmd_position lets the firmware track each setpoint with a position-only PID.
  When the streamer's velocity cap and the firmware's PID interact, the drone
  overshoots — especially when the target changes faster than the firmware's
  position settle time. The go_to-per-tick approach lets the firmware plan a
  full polynomial from the CURRENT state (with its own velocity estimate) to
  the new target, so there's no integral wind-up.
"""

from __future__ import annotations

import threading

import rclpy
from crazyflie_interfaces.srv import GoTo
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class TrajectoryStreamerNode(Node):

    def __init__(self):
        super().__init__('trajectory_streamer')
        self.declare_parameter('drone_name', 'cf1')
        # Firmware polynomial duration. Should be >= the policy tick interval
        # (0.2s at 5 Hz) so each new go_to replaces the previous trajectory
        # before it finishes. 0.5s gives a smooth overlap.
        self.declare_parameter('go_to_duration', 0.5)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('min_target_jump', 0.02)  # m — debounce tiny jitter

        self.drone_name = str(self.get_parameter('drone_name').value)
        self.go_to_duration = float(self.get_parameter('go_to_duration').value)
        self.yaw = float(self.get_parameter('yaw').value)
        self.min_jump = float(self.get_parameter('min_target_jump').value)

        self._client = self.create_client(GoTo, f'/{self.drone_name}/go_to')
        # We don't wait_for_service() — that blocks until cf server is up.
        # Send-and-fail-silently is fine for the first few ticks.
        self._last_target = None
        self._lock = threading.Lock()
        self._inflight = 0
        self._max_inflight = 2  # cap to avoid service queue back-up

        self.create_subscription(PoseStamped,
                                 f'/{self.drone_name}/policy_target',
                                 self._on_target, 10)

        self.get_logger().info(
            f'TrajectoryStreamer({self.drone_name}): go_to duration='
            f'{self.go_to_duration}s, min_jump={self.min_jump}m'
        )

    def _on_target(self, msg: PoseStamped):
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z)
        # Debounce: skip if target hasn't moved meaningfully.
        if self._last_target is not None:
            lx, ly, lz = self._last_target
            if (x - lx) ** 2 + (y - ly) ** 2 + (z - lz) ** 2 < self.min_jump ** 2:
                return
        if self._inflight >= self._max_inflight:
            return
        if not self._client.service_is_ready():
            return  # cf server not up yet; skip
        req = GoTo.Request()
        req.group_mask = 0
        req.relative = False
        req.goal.x = x
        req.goal.y = y
        req.goal.z = z
        req.yaw = self.yaw
        req.duration.sec = int(self.go_to_duration)
        req.duration.nanosec = int((self.go_to_duration - int(self.go_to_duration)) * 1e9)
        with self._lock:
            self._inflight += 1
        fut = self._client.call_async(req)
        fut.add_done_callback(self._on_resp)
        self._last_target = (x, y, z)

    def _on_resp(self, _fut):
        with self._lock:
            self._inflight = max(0, self._inflight - 1)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryStreamerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
