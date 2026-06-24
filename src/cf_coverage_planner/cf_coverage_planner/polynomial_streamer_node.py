"""Polynomial trajectory streamer.

The deployment problem for the RL coverage planner is that we need to send
smooth motion commands to the firmware that respect the drone's CURRENT
velocity. The available options each had a failure mode:

  - ``cmd_position`` (setpoint_streamer): firmware velocity-PI integrator
    wound up under sustained tracking error → 5+ m/s velocity dumps → crash.
  - ``cmd_full_state``: required ``notify_setpoints_stop`` to transition
    from high-level (takeoff) to low-level (full-state) mode — we didn't
    set that up correctly and drones crashed instantly.
  - ``go_to`` per tick: the firmware's high-level commander generates a
    7th-order polynomial assuming ZERO start velocity. When called on a
    moving drone, the polynomial mismatches the drone's actual state →
    drone overshoots → still crashes.

This node uses ``upload_trajectory + start_trajectory`` with a cubic
polynomial we generate ourselves. The polynomial is constrained by:
    p(0)   = current drone position (from odom)
    p'(0)  = current drone velocity (from odom twist)
    p(T)   = target position (from policy)
    p'(T)  = 0   (drone decelerates to a stop at the target)
4 constraints → 4 coefficients → 3rd-order polynomial in t.

A new trajectory is uploaded every ``replan_period`` seconds (default 0.5).
The firmware tracks the polynomial smoothly via its existing controller
(cascaded PID or Mellinger), without the velocity-PI wind-up failure mode
of cmd_position or the mode-transition failure of cmd_full_state.

The trajectory_id alternates between 0 and 1 each replan so we don't
overwrite the running trajectory's memory while the firmware is reading it.
"""

from __future__ import annotations

import threading
import math

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from crazyflie_interfaces.msg import TrajectoryPolynomialPiece
from crazyflie_interfaces.srv import StartTrajectory, UploadTrajectory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool


def cubic_with_boundary(
    p0: float, v0: float, p1: float, T: float
) -> np.ndarray:
    """Cubic polynomial p(t) = c0 + c1·t + c2·t² + c3·t³ satisfying:
        p(0) = p0;  p'(0) = v0;  p(T) = p1;  p'(T) = 0

    Returns (c0, c1, c2, c3).
    """
    # From p(0)=p0:  c0 = p0
    # From p'(0)=v0: c1 = v0
    # Two equations in c2, c3:
    #   p0 + v0·T + c2·T² + c3·T³ = p1
    #   v0 + 2·c2·T + 3·c3·T² = 0
    # Solve:
    T = float(T)
    dp = p1 - p0 - v0 * T
    # eq1: c2·T² + c3·T³ = dp
    # eq2: 2·c2·T + 3·c3·T² = -v0
    # → solve linear system
    M = np.array([[T * T, T ** 3],
                  [2 * T, 3 * T * T]], dtype=np.float64)
    b = np.array([dp, -v0], dtype=np.float64)
    c2, c3 = np.linalg.solve(M, b)
    return np.array([p0, v0, c2, c3], dtype=np.float64)


def pad_to_8(coeffs: np.ndarray) -> list:
    """Firmware expects 8 polynomial coefficients per axis (7th-order).
    Pad our 4-coeff (3rd-order) result with zeros for the high-order terms."""
    out = list(map(float, coeffs))
    while len(out) < 8:
        out.append(0.0)
    return out


class PolynomialStreamerNode(Node):

    def __init__(self):
        super().__init__('polynomial_streamer')
        self.declare_parameter('drone_name', 'cf1')
        # How long each trajectory segment lasts before being replanned.
        # Should be ≥ several policy ticks so the firmware has time to
        # actually execute most of the segment before replacement.
        self.declare_parameter('segment_duration', 1.0)
        # How often we replan. <= segment_duration, so each new trajectory
        # replaces the previous before it finishes.
        self.declare_parameter('replan_period', 0.5)
        # Hover altitude (z held constant).
        self.declare_parameter('altitude', 0.6)

        self.drone_name = str(self.get_parameter('drone_name').value)
        self.seg_duration = float(self.get_parameter('segment_duration').value)
        self.replan_period = float(self.get_parameter('replan_period').value)
        self.altitude = float(self.get_parameter('altitude').value)

        # Latest drone state (from odom)
        self._lock = threading.Lock()
        self._odom_pos: tuple[float, float, float] | None = None
        self._odom_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
        # Latest policy target
        self._target: tuple[float, float, float] | None = None
        # Toggle between trajectory_id 0 and 1 so we don't overwrite
        # a currently-executing trajectory's memory.
        self._traj_id_toggle = 0

        # Subscriptions
        self.create_subscription(Odometry, f'/{self.drone_name}/odom',
                                 self._on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, f'/{self.drone_name}/policy_target',
                                 self._on_target, 10)
        # /demo/stopped (latched): when True, stop uploading trajectories so the
        # optimizer's land command isn't fought (demo freezes).
        self._demo_stopped = False
        self.create_subscription(
            Bool, '/demo/stopped', self._on_demo_stopped,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        # Service clients
        self._upload_cli = self.create_client(
            UploadTrajectory, f'/{self.drone_name}/upload_trajectory')
        self._start_cli = self.create_client(
            StartTrajectory, f'/{self.drone_name}/start_trajectory')

        # Replan timer
        self.create_timer(self.replan_period, self._replan_tick)

        self._inflight = 0

        self.get_logger().info(
            f'PolynomialStreamer({self.drone_name}): seg={self.seg_duration}s, '
            f'replan={self.replan_period}s, altitude={self.altitude}m')

    def _on_odom(self, msg: Odometry):
        with self._lock:
            self._odom_pos = (msg.pose.pose.position.x,
                              msg.pose.pose.position.y,
                              msg.pose.pose.position.z)
            self._odom_vel = (msg.twist.twist.linear.x,
                              msg.twist.twist.linear.y,
                              msg.twist.twist.linear.z)

    def _on_target(self, msg: PoseStamped):
        with self._lock:
            self._target = (msg.pose.position.x,
                            msg.pose.position.y,
                            msg.pose.position.z)

    def _on_demo_stopped(self, msg: Bool):
        if msg.data and not self._demo_stopped:
            self._demo_stopped = True
            self.get_logger().info(
                '/demo/stopped=True → halting trajectory uploads (demo over)')

    def _replan_tick(self):
        if self._demo_stopped:
            return
        with self._lock:
            if self._odom_pos is None or self._target is None:
                return
            px, py, pz = self._odom_pos
            vx, vy, vz = self._odom_vel
            tx, ty, _ = self._target
            traj_id = self._traj_id_toggle
            self._traj_id_toggle = 1 - self._traj_id_toggle

        # Cap displacement per segment so the polynomial's peak velocity
        # stays bounded. Cubic with v(0)=current_vel and v(T)=0 has peak
        # velocity proportional to displacement. For displacement d over
        # duration T=1s, peak ≈ 1.5*d. Capping at 0.4m → peak ≤ 0.6 m/s,
        # well below the firmware's 1.5 m/s safe limit. This cap is LOAD-BEARING:
        # the de-risk test raised it to 0.85 (1.28 m/s) to match the 2D's ~1.18 m/s
        # sweep, but that let drones chase the bang-bang policy's snapped targets too
        # fast → overshoot → 2 drones crashed (cf1 ran to r=2.04, cf4 dropped) in the
        # demo window. Reverted to the proven 0.4. Faithful tracking of the fast
        # bang-bang sweep needs the receding-horizon streamer, not a bigger cap.
        MAX_DISPLACEMENT = 0.4   # run-1 baseline (reverted 2026-06-04 from 0.25 — slowing didn't stop the drop)
        dx = tx - px; dy = ty - py
        dist = math.sqrt(dx * dx + dy * dy)
        if dist > MAX_DISPLACEMENT:
            scale = MAX_DISPLACEMENT / dist
            tx = px + dx * scale
            ty = py + dy * scale

        if not self._upload_cli.service_is_ready():
            return
        if not self._start_cli.service_is_ready():
            return
        if self._inflight > 2:
            return  # back off if service queue is busy

        # Hold altitude — z target is constant; v_z effectively zero.
        tz = self.altitude

        # Cubic polynomial per axis with boundary conditions.
        cx = cubic_with_boundary(px, vx, tx, self.seg_duration)
        cy = cubic_with_boundary(py, vy, ty, self.seg_duration)
        cz = cubic_with_boundary(pz, vz, tz, self.seg_duration)
        # Yaw stays at 0 — all-zero polynomial.
        cyaw = np.zeros(4, dtype=np.float64)

        piece = TrajectoryPolynomialPiece()
        piece.poly_x = pad_to_8(cx)
        piece.poly_y = pad_to_8(cy)
        piece.poly_z = pad_to_8(cz)
        piece.poly_yaw = pad_to_8(cyaw)
        dur = DurationMsg()
        dur.sec = int(self.seg_duration)
        dur.nanosec = int((self.seg_duration - int(self.seg_duration)) * 1e9)
        piece.duration = dur

        up_req = UploadTrajectory.Request()
        up_req.trajectory_id = int(traj_id)
        up_req.piece_offset = 0
        up_req.pieces = [piece]

        self._inflight += 1
        up_fut = self._upload_cli.call_async(up_req)
        up_fut.add_done_callback(lambda f, tid=traj_id: self._on_uploaded(f, tid))

    def _on_uploaded(self, fut, trajectory_id):
        # Start the trajectory once uploaded.
        start_req = StartTrajectory.Request()
        start_req.group_mask = 0
        start_req.trajectory_id = int(trajectory_id)
        start_req.timescale = 1.0
        start_req.reversed = False
        start_req.relative = False
        st_fut = self._start_cli.call_async(start_req)
        st_fut.add_done_callback(self._on_started)

    def _on_started(self, _fut):
        self._inflight = max(0, self._inflight - 1)


def main(args=None):
    rclpy.init(args=args)
    node = PolynomialStreamerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
