"""Per-drone Level-2 coverage planner (ROS shim around the polar lawnmower).

THROWAWAY when RL lands. Replaced by an RL inference node that subscribes to the same
inputs (/coverage/leash, /cfN/odom) and publishes the same output (/cfN/policy_target).

Inputs:
  - /coverage/leash (coverage_optimizer_interfaces/Leash)
  - /cfN/odom        (nav_msgs/Odometry)
Outputs:
  - /cfN/policy_target           (geometry_msgs/PoseStamped) — consumed by setpoint_streamer
  - /cfN/coverage_state_markers  (visualization_msgs/MarkerArray) — RViz leash ring
"""

import math

import rclpy
from coverage_optimizer_interfaces.msg import Leash
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from .polar_lawnmower import Mode, PolarLawnmower
from .sector_geometry import Sector, polar_to_world, world_to_polar


class CoveragePlanner(Node):

    def __init__(self):
        super().__init__('coverage_planner')

        self.declare_parameter('drone_name', 'cf1')
        self.declare_parameter('center_xy', [0.0, 0.0])
        self.declare_parameter('altitude', 1.0)
        self.declare_parameter('phi_mid_deg', 0.0)
        self.declare_parameter('phi_half_deg', 45.0)
        self.declare_parameter('r_star', 2.0)
        self.declare_parameter('r_max', 2.3)
        self.declare_parameter('q', 1.0)
        self.declare_parameter('c', 1.0)
        self.declare_parameter('footprint_radius', 0.13)
        self.declare_parameter('deadband', 0.10)
        self.declare_parameter('drone_speed', 1.5)
        self.declare_parameter('num_spokes', 5)
        self.declare_parameter('inner_radius', 0.5)
        self.declare_parameter('planner_rate_hz', 5.0)

        self.drone_name = str(self.get_parameter('drone_name').value)
        self.center_xy = tuple(self.get_parameter('center_xy').value)
        self.altitude = float(self.get_parameter('altitude').value)

        sector = Sector(
            phi_mid=math.radians(float(self.get_parameter('phi_mid_deg').value)),
            phi_half=math.radians(float(self.get_parameter('phi_half_deg').value)),
            r_max=float(self.get_parameter('r_max').value),
            r_star=float(self.get_parameter('r_star').value),
            q=float(self.get_parameter('q').value),
            c=float(self.get_parameter('c').value),
        )
        self.sector = sector
        self.lawnmower = PolarLawnmower(
            sector=sector,
            footprint_radius=float(self.get_parameter('footprint_radius').value),
            deadband=float(self.get_parameter('deadband').value),
            drone_speed=float(self.get_parameter('drone_speed').value),
            planner_rate_hz=float(self.get_parameter('planner_rate_hz').value),
            num_spokes=int(self.get_parameter('num_spokes').value),
            inner_radius=float(self.get_parameter('inner_radius').value),
        )

        self.latest_odom = None
        self.latest_leash = None

        self.create_subscription(Leash, '/coverage/leash', self._leash_cb, 10)
        self.create_subscription(
            Odometry, f'/{self.drone_name}/odom', self._odom_cb, 10)

        self.target_pub = self.create_publisher(
            PoseStamped, f'/{self.drone_name}/policy_target', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, f'/{self.drone_name}/coverage_state_markers', 10)

        rate_hz = float(self.get_parameter('planner_rate_hz').value)
        self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f'CoveragePlanner({self.drone_name}): sector phi_mid='
            f'{math.degrees(sector.phi_mid):.0f}deg, q={sector.q}, '
            f'r*={sector.r_star}, r_max={sector.r_max}, '
            f'footprint={self.lawnmower.footprint_radius}, deadband={self.lawnmower.deadband}')

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

    def _tick(self):
        if self.latest_odom is None or self.latest_leash is None:
            return

        p = self.latest_odom.pose.pose.position
        cur_r, cur_phi = world_to_polar(p.x, p.y, self.center_xy)

        target_r, target_phi, mode = self.lawnmower.step(
            cur_r, cur_phi, self.latest_leash)

        tx, ty = polar_to_world(target_r, target_phi, self.center_xy)

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'world'
        out.pose.position.x = float(tx)
        out.pose.position.y = float(ty)
        out.pose.position.z = float(self.altitude)
        out.pose.orientation.w = 1.0
        self.target_pub.publish(out)

        self.marker_pub.publish(self._make_markers(self.latest_leash, mode))

    def _make_markers(self, leash, mode):
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = f'{self.drone_name}_leash_arc'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.02
        m.color.a = 0.8
        # color by mode: red=retracting, green=outbound, blue=inbound, yellow=idle
        if mode is Mode.RETRACTING:
            m.color.r, m.color.g, m.color.b = 1.0, 0.2, 0.2
        elif mode is Mode.OUTBOUND:
            m.color.r, m.color.g, m.color.b = 0.2, 0.9, 0.2
        elif mode is Mode.INBOUND:
            m.color.r, m.color.g, m.color.b = 0.2, 0.4, 0.9
        else:  # IDLE
            m.color.r, m.color.g, m.color.b = 0.9, 0.9, 0.2
        # draw the leash arc inside the sector
        n = 32
        for k in range(n + 1):
            phi = self.sector.phi_low + (self.sector.phi_high - self.sector.phi_low) * k / n
            x, y = polar_to_world(leash, phi, self.center_xy)
            from geometry_msgs.msg import Point as _Pt
            arr_pt = _Pt()
            arr_pt.x = float(x)
            arr_pt.y = float(y)
            arr_pt.z = float(self.altitude)
            m.points.append(arr_pt)
        arr.markers.append(m)
        return arr


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
