"""Receding-horizon trajectory streamer (RL path).

Drop-in replacement for polynomial_streamer_node that fixes its two failure modes:
  - the single cubic decelerates to v=0 every 0.5 s (saw-tooth velocity -> jitter ->
    dropped single mocap markers on HW), and
  - it interpolates a straight Cartesian CHORD to the target, which bows OUTSIDE the
    leash (radial overshoot -> Sigma_drone > Sigma_leash -> FedDCSA not clean).

Instead, every replan it builds a smooth POLAR arc from the drone's CURRENT odom state
toward the policy target, fit as K quintic-Hermite pieces (C2 -> continuous accel ->
low jerk -> markers hold). The arc:
  - stays in POLAR with r clamped <= leash  => NO radial overshoot by construction;
  - ends with the policy TARGET's velocity (finite-diff of policy_target) => the drone
    flies WITH the moving setpoint (no decelerate-to-0), i.e. it TRACKS the RL policy;
  - has its first knot grafted to the current odom (p,v) => no kink at the replan seam.

The streamer does NOT generate the sweep — the sweep + leash-riding come from the RL
policy via /cfN/policy_target. This only makes the tracking smooth + overshoot-free.
Math lives in receding_horizon_traj.py (unit-tested).
"""
from __future__ import annotations

import math
import threading

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from coverage_optimizer_interfaces.msg import Leash
from crazyflie_interfaces.msg import TrajectoryPolynomialPiece
from crazyflie_interfaces.srv import StartTrajectory, UploadTrajectory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool

from cf_coverage_planner.receding_horizon_traj import (
    build_xy_pieces,
    decompose_velocity,
    hermite_knots_1d,
    polar_hermite_knots,
    quintic_hermite,
)


def _pad8(coeffs) -> list:
    """Firmware wants 8 coeffs/axis (7th order). Our quintics use 6; pad 2 zeros."""
    out = [float(c) for c in coeffs]
    while len(out) < 8:
        out.append(0.0)
    return out


class RecedingHorizonStreamerNode(Node):

    def __init__(self):
        super().__init__('receding_horizon_streamer')
        self.declare_parameter('drone_name', 'cf1')
        self.declare_parameter('altitude', 0.6)
        self.declare_parameter('horizon_sec', 1.5)       # cruise horizon H; arc length = cruise_speed*H
        self.declare_parameter('replan_period', 0.5)     # how often we re-plan (< horizon)
        self.declare_parameter('n_pieces', 5)            # quintic-Hermite pieces per arc
        self.declare_parameter('r_min', 0.3)             # polar floor (avoid omega blow-up near origin)
        self.declare_parameter('target_vel_alpha', 0.4)  # (legacy) low-pass on finite-diff target vel; END tangent no longer uses it
        self.declare_parameter('max_target_speed', 1.4)  # (legacy) clamp on finite-diff target speed (m/s)
        self.declare_parameter('leash_margin', 0.0)      # extra clamp below leash (m); 0 = clamp at leash
        # Constant-speed CRUISE (2D law: rd1=0, phid1=sweep_sign*cruise_speed/r1). The END is
        # PROJECTED one cruise-horizon ahead in the sweep direction (NOT pinned at the policy
        # target), clamped to the target angle so we never sweep past where the policy points.
        self.declare_parameter('cruise_speed', 1.18)     # tangential cruise speed (m/s); arc len = cruise_speed*H
        self.declare_parameter('decel_horizon', 0.15)    # (legacy) seconds-of-cruise ease window
        self.declare_parameter('sign_deadband_deg', 3.0) # (legacy) sweep-sign latch hysteresis
        # 2D-style constant-speed sweep: the streamer drives the ANGLE itself (a smooth
        # edge-to-edge bounce within the sector at cruise_speed), like the 2D policy_tick;
        # the RL policy still drives the RADIUS (rides the leash). phi_mid/phi_half = sector.
        self.declare_parameter('phi_mid_deg', 0.0)       # sector centerline (cardinal: 0/90/180/270)
        self.declare_parameter('phi_half_deg', 45.0)     # sector half-width
        self.declare_parameter('sweep_half_deg', 36.0)   # sweep amplitude (<= phi_half; gap for neighbor sep)

        self.drone_name = str(self.get_parameter('drone_name').value)
        self.altitude = float(self.get_parameter('altitude').value)
        self.H = float(self.get_parameter('horizon_sec').value)
        self.replan_period = float(self.get_parameter('replan_period').value)
        self.n_pieces = int(self.get_parameter('n_pieces').value)
        self.r_min = float(self.get_parameter('r_min').value)
        self.tv_alpha = float(self.get_parameter('target_vel_alpha').value)
        self.max_tv = float(self.get_parameter('max_target_speed').value)
        self.leash_margin = float(self.get_parameter('leash_margin').value)
        self.cruise_speed = float(self.get_parameter('cruise_speed').value)
        self.decel_horizon = float(self.get_parameter('decel_horizon').value)
        self.sign_deadband = math.radians(float(self.get_parameter('sign_deadband_deg').value))
        self.phi_mid = math.radians(float(self.get_parameter('phi_mid_deg').value))
        self.phi_half = math.radians(float(self.get_parameter('phi_half_deg').value))
        self.sweep_half = math.radians(float(self.get_parameter('sweep_half_deg').value))

        self._lock = threading.Lock()
        self._odom_pos: tuple[float, float, float] | None = None
        self._odom_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._target: tuple[float, float] | None = None
        self._target_vel = (0.0, 0.0)            # smoothed finite-diff of policy_target
        self._prev_target: tuple[float, float] | None = None
        self._prev_target_t: float | None = None
        self._leash: float | None = None
        self._sweep_sign = 1                      # (legacy)
        self._sweep_dir = 1                        # 2D arc_direction (+1/-1); flips when drone reaches a sector edge
        self._traj_id_toggle = 0
        self._inflight = 0
        self._demo_stopped = False

        self.create_subscription(Odometry, f'/{self.drone_name}/odom',
                                 self._on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, f'/{self.drone_name}/policy_target',
                                 self._on_target, 10)
        self.create_subscription(Leash, '/coverage/leash', self._on_leash, 10)
        self.create_subscription(
            Bool, '/demo/stopped', self._on_demo_stopped,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self._upload_cli = self.create_client(
            UploadTrajectory, f'/{self.drone_name}/upload_trajectory')
        self._start_cli = self.create_client(
            StartTrajectory, f'/{self.drone_name}/start_trajectory')

        self.create_timer(self.replan_period, self._replan_tick)
        self.get_logger().info(
            f'RecedingHorizonStreamer({self.drone_name}): H={self.H}s, '
            f'replan={self.replan_period}s, {self.n_pieces} pieces, altitude={self.altitude}m')

    # ---- callbacks ----
    def _on_odom(self, msg: Odometry):
        with self._lock:
            self._odom_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                              msg.pose.pose.position.z)
            self._odom_vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                              msg.twist.twist.linear.z)

    def _on_target(self, msg: PoseStamped):
        t = self.get_clock().now().nanoseconds * 1e-9
        tx, ty = msg.pose.position.x, msg.pose.position.y
        with self._lock:
            if self._prev_target is not None and self._prev_target_t is not None:
                dt = t - self._prev_target_t
                if dt > 1e-3:
                    vx = (tx - self._prev_target[0]) / dt
                    vy = (ty - self._prev_target[1]) / dt
                    sp = math.hypot(vx, vy)
                    if sp > self.max_tv:                 # clamp velocity spikes
                        vx *= self.max_tv / sp; vy *= self.max_tv / sp
                    a = self.tv_alpha
                    self._target_vel = (a*vx + (1-a)*self._target_vel[0],
                                        a*vy + (1-a)*self._target_vel[1])
            self._target = (tx, ty)
            self._prev_target = (tx, ty)
            self._prev_target_t = t

    def _on_leash(self, msg: Leash):
        try:
            idx = list(msg.drone_names).index(self.drone_name)
        except ValueError:
            return
        with self._lock:
            self._leash = float(msg.radii[idx])

    def _on_demo_stopped(self, msg: Bool):
        if msg.data and not self._demo_stopped:
            self._demo_stopped = True
            self.get_logger().info('/demo/stopped=True -> halting trajectory uploads')

    # ---- replan ----
    def _replan_tick(self):
        if self._demo_stopped:
            return
        with self._lock:
            if self._odom_pos is None or self._target is None:
                return
            px, py, pz = self._odom_pos
            vx, vy, vz = self._odom_vel
            tx, ty = self._target
            tvx, tvy = self._target_vel
            leash = self._leash if self._leash is not None else math.hypot(tx, ty)
            traj_id = self._traj_id_toggle
            self._traj_id_toggle = 1 - self._traj_id_toggle

        if not self._upload_cli.service_is_ready() or not self._start_cli.service_is_ready():
            return
        if self._inflight > 2:
            return

        # leash clamp (no overshoot); the policy target already encodes edge_target*leash.
        leash_clamped = max(self.r_min + 1e-3, leash - self.leash_margin)
        r0 = max(math.hypot(px, py), 1e-3); phi0 = math.atan2(py, px)
        rd0, phid0 = decompose_velocity(vx, vy, r0, phi0)

        # --- 2D-style constant-speed sweep: the STREAMER drives the angle ---
        # Chasing the policy's jittery deployed angle gave a small, off-center, gets-stuck sweep.
        # Instead mirror the 2D policy_tick: sweep the ANGLE edge-to-edge within the sector at a
        # constant cruise speed, flipping direction when the DRONE reaches a sweep edge (so it
        # never gets stuck). The RL policy still drives the RADIUS (r1 = its target radius clamped
        # to the leash -> rides the leash, no overshoot). The target angle is projected one
        # horizon ahead of the DRONE (not a far fixed point), so the executed speed == cruise
        # regardless of H -> wide, fast, smooth side-to-side motion like the 2D.
        r1 = float(min(max(math.hypot(tx, ty), self.r_min), leash_clamped))
        d_drone = ((phi0 - self.phi_mid + math.pi) % (2 * math.pi)) - math.pi  # drone offset from centerline
        if d_drone >= self.sweep_half:
            self._sweep_dir = -1
        elif d_drone <= -self.sweep_half:
            self._sweep_dir = 1
        omega = self.cruise_speed / max(r1, self.r_min)
        phi1 = phi0 + self._sweep_dir * omega * self.H          # one horizon ahead of the drone
        d_target = max(-self.sweep_half,
                       min(self.sweep_half,
                           ((phi1 - self.phi_mid + math.pi) % (2 * math.pi)) - math.pi))
        phi1 = self.phi_mid + d_target                          # clamp into the sweep band
        phi1 = phi0 + ((phi1 - phi0 + math.pi) % (2 * math.pi) - math.pi)  # short-way unwrap vs phi0
        rd1 = 0.0                                               # ride the leash (no radial drift)
        phid1 = self._sweep_dir * self.cruise_speed / max(r1, self.r_min)   # arrive moving at cruise

        kn = polar_hermite_knots(r0, phi0, rd0, phid0, r1, phi1, rd1, phid1,
                                 leash_clamped, self.H, self.n_pieces, self.r_min)
        xy_pieces = build_xy_pieces(kn, (px, py), (vx, vy))
        # z: gentle Hermite ease to the hold altitude (grafts current pz,vz)
        zk, vzk, azk = hermite_knots_1d(pz, vz, self.altitude, 0.0, self.H, kn["t"])

        pieces_msg = []
        for i, (tau, cx, cy) in enumerate(xy_pieces):
            cz = quintic_hermite(zk[i], vzk[i], azk[i], zk[i+1], vzk[i+1], azk[i+1], tau)
            piece = TrajectoryPolynomialPiece()
            piece.poly_x = _pad8(cx)
            piece.poly_y = _pad8(cy)
            piece.poly_z = _pad8(cz)
            piece.poly_yaw = _pad8([0.0])
            dur = DurationMsg()
            dur.sec = int(tau); dur.nanosec = int((tau - int(tau)) * 1e9)
            piece.duration = dur
            pieces_msg.append(piece)

        up_req = UploadTrajectory.Request()
        up_req.trajectory_id = int(traj_id)
        up_req.piece_offset = 0
        up_req.pieces = pieces_msg
        self._inflight += 1
        fut = self._upload_cli.call_async(up_req)
        fut.add_done_callback(lambda f, tid=traj_id: self._on_uploaded(f, tid))

    def _on_uploaded(self, _fut, trajectory_id):
        req = StartTrajectory.Request()
        req.group_mask = 0
        req.trajectory_id = int(trajectory_id)
        req.timescale = 1.0
        req.reversed = False
        req.relative = False          # first knot already = current odom pos
        st = self._start_cli.call_async(req)
        st.add_done_callback(self._on_started)

    def _on_started(self, _fut):
        self._inflight = max(0, self._inflight - 1)


def main(args=None):
    rclpy.init(args=args)
    node = RecedingHorizonStreamerNode()
    # 2026-06-05: EventsExecutor (event-driven, no per-iteration wait-set rebuild) so the
    # 0.5s replan + the upload/start service callbacks aren't starved in the busy sim DDS
    # graph (same rclpy executor-rebuild bottleneck that throttled the rl_planner).
    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
