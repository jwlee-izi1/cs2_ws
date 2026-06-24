"""Per-drone tangential leash-arc sweep planner (a.k.a. "figure8" planner slot).

WHY THIS SHAPE (2026-06-04 rewrite): the previous lemniscate placed its CENTER on
the sector diagonal at |center| = center_alpha*sqrt(2)*leash = 0.707*leash and
oscillated the lobe RADIALLY around it. Measured: the drone's mean radius landed at
~0.72*leash, so Sigma c*r^2_drone (~7.3) fell far below the optimizer's commanded
Sigma c*r^2_leash (~8.8) -> the Lagrangian budget violation never reached the drones
-> 0 drops. The demo's whole point (Lagrangian drops vs FedDCSA clean) needs
Sigma_drone ~= Sigma_leash.

FIX: pin the radius AT the leash (r = r_frac*leash, mirroring rl_planner_node.py's
hybrid r-pinning) and sweep the drone TANGENTIALLY (sideways) along the leash arc
within its angular sector, driven by a SMOOTH sinusoid. Because r stays ~= leash,
Sigma_drone = r_frac^2 * Sigma_leash by construction -> the contrast appears. The
smooth sinusoid (no bang-bang reversals) keeps the single-marker mocap locked on
real hardware -- the reason we use this scripted planner over the RL policy.

SPATIAL-PARTITION SAFETY: every drone uses the SAME shared wall clock for its sweep
phase, so all drones shift their angle by the identical delta at every instant -> the
90-deg gap between adjacent sector centerlines is preserved for all t -> drones move
in-phase and never converge (adjacent separation ~= 2*r*sin(45deg) >= 0.56 m even at
the min_leash floor). Do NOT phase-offset adjacent drones.

Inputs:  /coverage/leash, /cfN/odom
Outputs: /cfN/policy_target  (same downstream contract as polar_lawnmower / rl_planner)

See docs/hw_multi_drone_verification.md for verification procedure.
"""

import math
import time

import rclpy
from coverage_optimizer_interfaces.msg import Leash
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


class QuadrantFigure8(Node):

    def __init__(self):
        super().__init__('quadrant_figure8')

        self.declare_parameter('drone_name', 'cf1')
        self.declare_parameter('quadrant_sign_x', 1)   # +1 for +x quadrant, -1 for -x
        self.declare_parameter('quadrant_sign_y', 1)   # +1 for +y quadrant, -1 for -y
        # --- Tangential leash-arc sweep geometry ---
        # r pinned at r_frac*leash so Sigma_drone = r_frac^2 * Sigma_leash. 0.99 mirrors
        # the RL hybrid's r_frac (rl_planner_node.py): Sigma_drone = 0.98*Sigma_leash ->
        # Lagrangian 0.98*8.83=8.65 > B_op=8.15 (drops); FedDCSA 0.98*8.0=7.84 < 8.15 (clean).
        self.declare_parameter('r_frac', 0.99)
        self.declare_parameter('r_max', 1.9)            # hard radial cap (matches sectors r_max)
        # Tangential sweep = sweep_amp_frac of the half-wedge (phi_half_deg). 0.8 mirrors
        # the RL SWEEP_CLIP=0.8 -> +-36 deg, strictly inside the +-45 deg sector.
        self.declare_parameter('sweep_amp_frac', 0.8)
        self.declare_parameter('phi_half_deg', 45.0)    # angular half-width of each sector
        # Sector centerline bearing (deg). If set (>-900) it's used directly — e.g. CARDINAL
        # 0/90/180/270 to match the optimizer sectors + a cardinal drone placement. If unset
        # (-999 sentinel), derived from the quadrant signs via atan2 (diagonal 45/-45/225/135).
        self.declare_parameter('phi_mid_deg', -999.0)
        # Full sweep cycle (s). 9.0 -> peak tangential speed at r=1.8 m is
        # r*sweep_amp*(2pi/period) = 1.8*0.628*0.698 = 0.79 m/s, well under the
        # streamer's 1.5 m/s cap (so no setpoint attenuation), and close to the old
        # figure-8's ~0.92 m/s peak (proven gentle for HW marker tracking).
        self.declare_parameter('sweep_period_s', 9.0)
        # Optional tiny INWARD radial wobble to trace a small figure-8 hugging the leash
        # arc (Lissajous, r at 2x the sweep freq). 0.0 = pure arc sweep (default, chosen
        # for max drop-fidelity). Keep <= 0.05 m if used -- larger kills the drop contrast.
        self.declare_parameter('wobble_w', 0.0)
        # Safety floor on the effective leash for the sweep radius. When the optimizer
        # leash drops below this (pre-fire-ignition, q_i ~ 0 -> r_i^k ~ 0), the planner
        # keeps the drone on its sector arc at r_frac*min_leash instead of diving to
        # origin. /coverage/leash still carries the optimizer's true r_i^k (paper claim
        # unaffected). At 0.4: adjacent separation = 2*0.99*0.4*sin(45) = 0.56 m (BVC-safe).
        self.declare_parameter('min_leash', 0.0)
        self.declare_parameter('altitude', 0.6)
        self.declare_parameter('planner_rate_hz', 5.0)

        self.drone_name = str(self.get_parameter('drone_name').value)
        self.sign_x = int(self.get_parameter('quadrant_sign_x').value)
        self.sign_y = int(self.get_parameter('quadrant_sign_y').value)
        self.r_frac = float(self.get_parameter('r_frac').value)
        self.r_max = float(self.get_parameter('r_max').value)
        self.phi_half = math.radians(float(self.get_parameter('phi_half_deg').value))
        self.sweep_amp = float(self.get_parameter('sweep_amp_frac').value) * self.phi_half
        self.sweep_period = float(self.get_parameter('sweep_period_s').value)
        self.wobble_w = float(self.get_parameter('wobble_w').value)
        self.min_leash = float(self.get_parameter('min_leash').value)
        self.altitude = float(self.get_parameter('altitude').value)

        # Sector centerline bearing: explicit phi_mid_deg if given (CARDINAL — matches the optimizer
        # sectors + a cardinal placement), else derived from the quadrant signs (diagonal) via atan2.
        _pm = float(self.get_parameter('phi_mid_deg').value)
        self.phi_mid = math.radians(_pm) if _pm > -900.0 else math.atan2(self.sign_y, self.sign_x)

        self.latest_leash = None
        self.latest_odom = None

        self.create_subscription(Leash, '/coverage/leash', self._leash_cb, 10)
        # Odom not used for the sweep (open-loop on time) but gate first publish on it
        # so the streamer has a live starting setpoint.
        self.create_subscription(
            Odometry, f'/{self.drone_name}/odom', self._odom_cb, 10)
        # /demo/stopped (latched): when True, stop publishing policy_target so nothing fights
        # the optimizer's land@stop_time_s (the streamer halts too).
        self._demo_stopped = False
        self.create_subscription(
            Bool, '/demo/stopped', self._on_demo_stopped,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self.target_pub = self.create_publisher(
            PoseStamped, f'/{self.drone_name}/policy_target', 10)
        # Mirror polar_lawnmower's marker contract so the RViz config picks it up unchanged.
        self.marker_pub = self.create_publisher(
            MarkerArray, f'/{self.drone_name}/coverage_state_markers', 10)

        rate_hz = float(self.get_parameter('planner_rate_hz').value)
        self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f'QuadrantFigure8({self.drone_name}): TANGENTIAL leash-arc sweep | '
            f'phi_mid={math.degrees(self.phi_mid):.0f}deg sweep=+-{math.degrees(self.sweep_amp):.0f}deg '
            f'period={self.sweep_period}s r_frac={self.r_frac} wobble={self.wobble_w} z={self.altitude} '
            f'(r pinned at r_frac*leash -> Sigma_drone = {self.r_frac**2:.2f}*Sigma_leash)')

    def _leash_cb(self, msg: Leash):
        try:
            idx = list(msg.drone_names).index(self.drone_name)
        except ValueError:
            self.get_logger().warn(
                f'{self.drone_name} not in /coverage/leash drone_names; ignoring')
            return
        self.latest_leash = float(msg.radii[idx])

    def _odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def _on_demo_stopped(self, msg: Bool):
        if msg.data and not self._demo_stopped:
            self._demo_stopped = True
            self.get_logger().info(
                '/demo/stopped=True → halting policy_target (demo over, optimizer lands the drones)')

    def _tick(self):
        if self._demo_stopped:
            return   # demo over — stop publishing targets so nothing fights the land
        if self.latest_odom is None or self.latest_leash is None:
            return

        # Pin the radius AT the leash (mean r = r_frac*leash) so Sigma_drone ~= Sigma_leash.
        leash = max(0.0, self.latest_leash)
        effective_leash = max(leash, self.min_leash)
        target_r = min(self.r_frac * effective_leash, self.r_max)

        # Smooth tangential sweep on a SHARED wall clock -> all drones in-phase (the gap
        # between adjacent sector centerlines stays 90 deg for all t -> never converge).
        phase = 2.0 * math.pi * (time.time() / self.sweep_period)
        phi = self.phi_mid + self.sweep_amp * math.sin(phase)
        # Optional inward figure-8 wobble (2x freq); wobble_w=0 -> pure arc (r=target_r).
        r = target_r - self.wobble_w * (1.0 - math.cos(2.0 * phase)) / 2.0

        target_x = r * math.cos(phi)
        target_y = r * math.sin(phi)

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'world'
        out.pose.position.x = float(target_x)
        out.pose.position.y = float(target_y)
        out.pose.position.z = float(self.altitude)
        out.pose.orientation.w = 1.0
        self.target_pub.publish(out)

        self.marker_pub.publish(self._make_markers(leash, target_r))

    def _make_markers(self, leash, target_r):
        """RViz visualization: per-drone 90 deg leash arc at radius = leash, centered at
        ARENA ORIGIN, spanning the drone's quadrant angular range. Matches polar_lawnmower's
        leash-arc visual so the user sees the optimizer's per-drone leash as a radius."""
        arr = MarkerArray()
        # 90-deg wedge centered on the drone's sector centerline (phi_mid +- phi_half),
        # so it's correct for both cardinal (phi_mid_deg) and diagonal (atan2) bearings.
        phi_lo = self.phi_mid - self.phi_half
        phi_hi = self.phi_mid + self.phi_half

        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = f'{self.drone_name}_leash_arc'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.03
        # Color brightness scales with leash so dynamics are visible. Normalize by
        # 1.9 m (the optimizer's r_max ceiling) for the brightness ramp.
        brightness = min(leash / 1.9, 1.0)
        m.color.a = 0.9
        m.color.r = 0.2 * brightness
        m.color.g = 0.3 + 0.6 * brightness
        m.color.b = 0.2 * brightness
        n = 32
        for k in range(n + 1):
            phi = phi_lo + (phi_hi - phi_lo) * k / n
            pt = Point()
            pt.x = float(leash * math.cos(phi))
            pt.y = float(leash * math.sin(phi))
            pt.z = float(self.altitude)
            m.points.append(pt)
        arr.markers.append(m)

        # Text label on the sector centerline at the pinned radius (leash + r values).
        txt = Marker()
        txt.header.frame_id = 'world'
        txt.header.stamp = self.get_clock().now().to_msg()
        txt.ns = f'{self.drone_name}_figure8_label'
        txt.id = 1
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x = float(target_r * math.cos(self.phi_mid))
        txt.pose.position.y = float(target_r * math.sin(self.phi_mid))
        txt.pose.position.z = float(self.altitude + 0.3)
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.12
        txt.color.a = 1.0
        txt.color.r = txt.color.g = txt.color.b = 1.0
        txt.text = self.drone_name   # 2026-06-04: clean label = just "cfN" (dropped leash=/r= text for the video)
        arr.markers.append(txt)
        return arr


def main(args=None):
    rclpy.init(args=args)
    node = QuadrantFigure8()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
