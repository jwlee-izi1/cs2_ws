"""Payload optimizer node: FedDCSA for cooperative payload leveling.

Reads initial drone z from odom, runs Algorithm 1 (gate + projected
gradient), and sends absolute go_to corrections at feasible rounds.
"""

import json
import math
import sys
import time

from builtin_interfaces.msg import Duration as DurationMsg
from crazyflie_interfaces.srv import GoTo
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from fed_dcsa.payload_algorithm import DroneState, PayloadFedDCSA

# State machine states
INIT = 0
READ_Z = 1
RUN_ROUNDS = 2
DONE = 3


class PayloadOptimizerNode(Node):

    def __init__(self):
        super().__init__('payload_optimizer')

        # Parameters
        self.declare_parameter('K', 15)
        self.declare_parameter('T', 5)
        self.declare_parameter('q_weights', [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('w_weights', [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('tau_max', 0.0016)
        self.declare_parameter('delta_z_max', 0.15)
        self.declare_parameter('c1', 0.002)
        self.declare_parameter('c2', 0.003)
        self.declare_parameter('settle_time', 4.0)
        self.declare_parameter('drone_names', ['cf1', 'cf2', 'cf3', 'cf4'])
        self.declare_parameter('dry_run', False)
        self.declare_parameter('transport_vx', 0.0)
        self.declare_parameter('transport_vy', 0.0)

        self.K = self.get_parameter('K').value
        self.T = self.get_parameter('T').value
        self.q_weights = list(self.get_parameter('q_weights').value)
        self.w_weights = list(self.get_parameter('w_weights').value)
        self.tau_max = self.get_parameter('tau_max').value
        self.delta_z_max = self.get_parameter('delta_z_max').value
        self.c1 = self.get_parameter('c1').value
        self.c2 = self.get_parameter('c2').value
        self.settle_time = self.get_parameter('settle_time').value
        self.drone_names = list(self.get_parameter('drone_names').value)
        self.dry_run = self.get_parameter('dry_run').value
        self.transport_vx = self.get_parameter('transport_vx').value
        self.transport_vy = self.get_parameter('transport_vy').value

        self.n_drones = len(self.drone_names)

        # Odom storage
        self.latest_odom = {}
        self.odom_received = set()
        cb_group = ReentrantCallbackGroup()
        for name in self.drone_names:
            self.create_subscription(
                Odometry, f'/{name}/odom',
                lambda msg, n=name: self._odom_cb(n, msg),
                10, callback_group=cb_group)

        # GoTo service clients
        self.goto_clients = {}
        for name in self.drone_names:
            self.goto_clients[name] = self.create_client(GoTo, f'/{name}/go_to')

        # Publisher
        self.round_pub = self.create_publisher(String, '/fed_dcsa/round_status', 10)

        # State
        self.state = INIT
        self.current_round = 0
        self.algo = None
        self.drones = []
        self.wait_until = None

        self.timer = self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'PayloadOptimizer: K={self.K}, T={self.T}, '
            f'c1={self.c1}, c2={self.c2}, tau_max={self.tau_max}')

    def _odom_cb(self, name, msg):
        self.latest_odom[name] = msg
        self.odom_received.add(name)

    def _make_duration(self, sec):
        s = int(sec)
        ns = int((sec - s) * 1e9)
        return DurationMsg(sec=s, nanosec=ns)

    def _call_goto(self, drone_name, x, y, z, duration=8.0):
        req = GoTo.Request()
        req.group_mask = 0
        req.relative = False
        req.goal = Point(x=float(x), y=float(y), z=float(z))
        req.yaw = 0.0
        req.duration = self._make_duration(duration)
        future = self.goto_clients[drone_name].call_async(req)
        future.add_done_callback(
            lambda f, n=drone_name: self._on_goto_done(f, n))

    def _on_goto_done(self, future, drone_name):
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f'{drone_name} go_to failed: {e}')

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _tick(self):
        if self.state == INIT:
            self._tick_init()
        elif self.state == READ_Z:
            self._tick_read_z()
        elif self.state == RUN_ROUNDS:
            self._tick_run_rounds()

    def _tick_init(self):
        if len(self.odom_received) >= self.n_drones:
            self.state = READ_Z

    def _tick_read_z(self):
        # Read z̄_i, x̄_i, ȳ_i from current odom
        z_bars = []
        self.drones = []
        for i, name in enumerate(self.drone_names):
            odom = self.latest_odom[name]
            p = odom.pose.pose.position
            z_bars.append(p.z)
            self.drones.append(DroneState(
                drone_id=i,
                z_bar=p.z,
                x_bar=p.x,
                y_bar=p.y,
                dz=0.0,
                q=self.q_weights[i],
                w=self.w_weights[i],
            ))

        # Create algorithm
        self.algo = PayloadFedDCSA(
            K=self.K, T=self.T, c1=self.c1, c2=self.c2,
            tau_max=self.tau_max, delta_z_max=self.delta_z_max,
            z_bars=z_bars, q_weights=self.q_weights, w_weights=self.w_weights,
        )

        # Startup diagnostics
        G_0 = self.algo.compute_G(self.drones)
        eta_0 = self.algo.eta(0)
        r_drift_0 = self.algo.r_drift(0)
        gap = G_0 - self.tau_max

        b_0 = 1 if G_0 <= eta_0 else 0

        self.get_logger().info('=' * 60)
        self.get_logger().info('PAYLOAD OPTIMIZER — STARTUP DIAGNOSTICS')
        self.get_logger().info(f'  z_bars = {[f"{z:.4f}" for z in z_bars]}')
        self.get_logger().info(f'  z*     = {self.algo.z_star:.4f}')
        self.get_logger().info(f'  G_0    = {G_0:.6f}')
        self.get_logger().info(f'  tau_max= {self.tau_max:.6f}')
        self.get_logger().info(f'  G_0 - tau_max = {gap:.6f}')
        self.get_logger().info(f'  c1={self.c1}, c2={self.c2}')
        self.get_logger().info(f'  eta_0  = {eta_0:.6f}')
        self.get_logger().info(f'  r_drift_0 = {r_drift_0:.6f}')
        self.get_logger().info(f'  eta_0 + r_drift_0 = {eta_0 + r_drift_0:.6f}')
        self.get_logger().info(f'  L_G    = {self.algo.L_G:.6f}')
        self.get_logger().info(f'  M_bar  = {self.algo.M_bar:.6f}')
        self.get_logger().info(f'  b_0    = {b_0} ({"INFEASIBLE — correction starts round 0" if b_0 == 0 else "FEASIBLE — consider reducing c2"})')

        if b_0 == 1:
            self.get_logger().warn(
                f'Gate is FEASIBLE at round 0 (G_0={G_0:.6f} <= eta_0={eta_0:.6f}). '
                f'No constraint-correction will happen initially. Consider reducing c2.')

        self.get_logger().info('=' * 60)

        if self.dry_run:
            self.get_logger().info('DRY RUN complete — exiting without sending commands.')
            self.state = DONE
            return

        self.state = RUN_ROUNDS
        self.current_round = 0
        self.transport_t0 = self._now()
        # Track the last z sent to each drone (for transport-only rounds)
        self.last_sent_z = [d.z_bar for d in self.drones]

    def _tick_run_rounds(self):
        # Check if we're waiting for settle
        if self.wait_until is not None:
            if self._now() < self.wait_until:
                return
            self.wait_until = None

        if self.current_round >= self.K:
            self.get_logger().info('All rounds complete.')
            self.state = DONE
            return

        k = self.current_round

        # Run algorithm round
        result = self.algo.run_round(k, self.drones)

        # Read actual odom z for telemetry
        actual_z = []
        for name in self.drone_names:
            odom = self.latest_odom.get(name)
            if odom:
                actual_z.append(odom.pose.pose.position.z)
            else:
                actual_z.append(float('nan'))

        # Compute actual G from odom (empirical)
        G_actual = sum(
            self.w_weights[i] * (actual_z[i] - self.algo.z_star) ** 2
            for i in range(self.n_drones)
        ) - self.tau_max

        # Publish telemetry
        telemetry = {
            'k': k,
            'b_k': result.b_k,
            'G_tilde': result.G_tilde,
            'eta_k': result.eta_k,
            'r_drift_k': result.r_drift_k,
            'eta_plus_rdrift': result.eta_k + result.r_drift_k,
            'G_actual_odom': G_actual,
            'F_value': result.F_value,
            'tau_max': self.tau_max,
            'dz': result.dz_after,
            'z_odom': actual_z,
            'wall_time': time.time(),
        }
        msg = String()
        msg.data = json.dumps(telemetry)
        self.round_pub.publish(msg)

        # Log
        gate_str = 'FEASIBLE' if result.b_k == 1 else 'INFEASIBLE'
        self.get_logger().info(
            f'Round {k:2d} [{gate_str:10s}] '
            f'G̃={result.G_tilde:+.6f} η={result.eta_k:.6f} '
            f'F={result.F_value:.6f} '
            f'δz=[{", ".join(f"{d:.4f}" for d in result.dz_after)}]')

        # Compute transport offset
        t_elapsed = self._now() - self.transport_t0
        x_offset = self.transport_vx * t_elapsed
        y_offset = self.transport_vy * t_elapsed

        if result.b_k == 1:
            # Feasible: send transport + z correction
            for i, name in enumerate(self.drone_names):
                d = self.drones[i]
                z_target = d.z_bar + d.dz
                self._call_goto(name, d.x_bar + x_offset, d.y_bar + y_offset, z_target)
                self.last_sent_z[i] = z_target
            self.get_logger().info(
                f'  → Sent corrections + transport, settling for {self.settle_time}s')
            self.wait_until = self._now() + self.settle_time
        elif self.transport_vx != 0.0 or self.transport_vy != 0.0:
            # Infeasible but transporting: send transport-only (keep last z)
            for i, name in enumerate(self.drone_names):
                d = self.drones[i]
                self._call_goto(name, d.x_bar + x_offset, d.y_bar + y_offset, self.last_sent_z[i])
            self.get_logger().info(
                f'  → Transport only (no z correction), settling for {self.settle_time}s')
            self.wait_until = self._now() + self.settle_time

        self.current_round += 1


def main(args=None):
    rclpy.init(args=args)
    node = PayloadOptimizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
