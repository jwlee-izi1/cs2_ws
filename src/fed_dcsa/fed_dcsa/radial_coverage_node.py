"""ROS node: FedDCSA optimizer for the radial coverage problem.

Reads geometry + step sizes from arena_4drone.yaml; runs the gate-then-project
algorithm at round_rate_hz; publishes /coverage/leash on each round.

This is ONE implementation of the /coverage/leash contract. Hot-swappable with
the mock optimizers in baseline_optimizers and (later) the RL allocator.
"""

import rclpy
from builtin_interfaces.msg import Duration
from coverage_optimizer_interfaces.msg import Leash
from crazyflie_interfaces.srv import Land
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64MultiArray

from fed_dcsa.radial_coverage_algorithm import (
    CoverageDrone,
    RadialCoverageFedDCSA,
)
from fed_dcsa.lagrangian_baseline import LagrangianBaseline


class RadialCoverageNode(Node):

    def __init__(self):
        super().__init__('radial_coverage_node')

        self.declare_parameter('drone_names', ['cf1', 'cf2', 'cf3', 'cf4'])
        self.declare_parameter('q_weights', [3.0, 1.5, 1.0, 0.5])
        self.declare_parameter('c_coeffs', [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('r_star_per_drone', [2.0, 2.0, 2.0, 2.0])
        self.declare_parameter('r_max_per_drone', [2.3, 2.3, 2.3, 2.3])
        self.declare_parameter('budget_B', 10.0)
        self.declare_parameter('K', 100)
        self.declare_parameter('T', 5)
        self.declare_parameter('c1', 0.015)
        self.declare_parameter('c2', 0.18)
        self.declare_parameter('noise_bound', 0.015)
        self.declare_parameter('seed', 0)
        self.declare_parameter('round_rate_hz', 2.0)
        self.declare_parameter('initial_r', 0.0)
        # Pick the optimization algorithm: 'feddcsa' (gate-then-project)
        # or 'lagrangian' (dual ascent baseline). Same I/O contract.
        self.declare_parameter('algorithm', 'feddcsa')
        # Graceful demo stop: at t=stop_time_s seconds the node lands all drones
        # and latches /demo/stopped (planner/streamer halt → demo freezes). 0=off.
        self.declare_parameter('stop_time_s', 0.0)

        self.drone_names = list(self.get_parameter('drone_names').value)
        q = [float(x) for x in self.get_parameter('q_weights').value]
        c = [float(x) for x in self.get_parameter('c_coeffs').value]
        r_star = [float(x) for x in self.get_parameter('r_star_per_drone').value]
        r_max = [float(x) for x in self.get_parameter('r_max_per_drone').value]
        initial_r = float(self.get_parameter('initial_r').value)

        n = len(self.drone_names)
        if not (len(q) == len(c) == len(r_star) == len(r_max) == n):
            raise ValueError('q/c/r_star/r_max length mismatch with drone_names')

        drones = [
            CoverageDrone(name=name, q=qi, c=ci, r_star=rs, r_max=rm, r=initial_r)
            for name, qi, ci, rs, rm in zip(self.drone_names, q, c, r_star, r_max)
        ]
        algorithm = str(self.get_parameter('algorithm').value).lower()
        if algorithm == 'feddcsa':
            self.algo = RadialCoverageFedDCSA(
                drones=drones,
                B=float(self.get_parameter('budget_B').value),
                T=int(self.get_parameter('T').value),
                c1=float(self.get_parameter('c1').value),
                c2=float(self.get_parameter('c2').value),
                noise_bound=float(self.get_parameter('noise_bound').value),
                seed=int(self.get_parameter('seed').value),
            )
        elif algorithm == 'lagrangian':
            self.algo = LagrangianBaseline(
                drones=drones,
                B=float(self.get_parameter('budget_B').value),
                T=int(self.get_parameter('T').value),
                c1=float(self.get_parameter('c1').value),
                c2=float(self.get_parameter('c2').value),
                noise_bound=float(self.get_parameter('noise_bound').value),
                seed=int(self.get_parameter('seed').value),
            )
        else:
            raise ValueError(f'unknown algorithm: {algorithm}')
        self.K = int(self.get_parameter('K').value)
        self.k = 0

        # /coverage/sector_weights overrides the per-drone q from YAML once
        # the first message arrives. Until then the YAML values stand in (so
        # static-q runs without the estimator still behave). The latest array
        # is folded into the algorithm state at the start of every round so
        # the optimizer always sees fresh q_i.
        self._got_sector_weights = False
        self.create_subscription(
            Float64MultiArray,
            '/coverage/sector_weights',
            self._on_sector_weights,
            10,
        )

        self.pub = self.create_publisher(Leash, '/coverage/leash', 10)

        rate_hz = float(self.get_parameter('round_rate_hz').value)
        self._round_rate_hz = rate_hz
        self.create_timer(1.0 / rate_hz, self._tick)

        # ── Graceful demo stop (land + freeze at stop_time_s) ──
        stop_time_s = float(self.get_parameter('stop_time_s').value)
        self.stop_round_k = int(round(stop_time_s * rate_hz)) if stop_time_s > 0.0 else None
        self._stopped = False
        # /demo/stopped is latched (TRANSIENT_LOCAL) so a planner/streamer that
        # subscribes late still receives the one-shot True.
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.stopped_pub = self.create_publisher(Bool, '/demo/stopped', latched)
        self.land_clients = {}
        if self.stop_round_k is not None:
            for name in self.drone_names:
                self.land_clients[name] = self.create_client(Land, f'/{name}/land')
            self.get_logger().info(
                f'demo stop ARMED: land all drones + freeze at k={self.stop_round_k} '
                f'(t={stop_time_s:.0f}s)')

        self.get_logger().info(
            f'RadialCoverageNode: n={n} drones, B={self.algo.B}, '
            f'K={self.K}, T={self.algo.T}, '
            f'c1={self.algo.c1}, c2={self.algo.c2}, '
            f'noise_bound={self.algo.noise_bound}, rate={rate_hz} Hz')

    def _on_sector_weights(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != len(self.algo.drones):
            self.get_logger().warning(
                f'/coverage/sector_weights size {len(msg.data)} != '
                f'{len(self.algo.drones)} drones; ignoring.',
                throttle_duration_sec=5.0,
            )
            return
        for d, q in zip(self.algo.drones, msg.data):
            d.q = float(q)
        if not self._got_sector_weights:
            self._got_sector_weights = True
            self.get_logger().info(
                f'first /coverage/sector_weights received: '
                f'q={["%.3f" % d.q for d in self.algo.drones]}'
            )

    def _tick(self):
        # Graceful demo stop: at stop_round_k, land all drones once + freeze the demo.
        if (self.stop_round_k is not None and not self._stopped
                and self._got_sector_weights and self.k >= self.stop_round_k):
            self._trigger_stop()

        # Always publish — even past K rounds, hold last leash so planners don't time out.
        # Hold off on running rounds until the first /coverage/sector_weights arrives.
        # Otherwise k=0 fires against the static YAML q fallback (large values), pulling
        # r to nonzero in one shot and polluting the chase trajectory shown in the paper
        # figure. With the gate, the optimizer starts cleanly from r=0 against live q.
        # Once stopped, hold the last leash (don't advance rounds) so the demo freezes.
        if self._got_sector_weights and not self._stopped and self.k < self.K:
            result = self.algo.run_round(self.k)
            if self.k % 10 == 0 or self.k == self.K - 1:
                # FedDCSA exposes gate_state + F; Lagrangian doesn't.
                gate = getattr(result, 'gate_state', 1)
                F = getattr(result, 'F', float('nan'))
                lam = getattr(result, 'lambda_k', float('nan'))
                self.get_logger().info(
                    f'k={self.k} gate={gate} G={result.G:+.4f} '
                    f'F={F:.4f} lambda={lam:.3f} '
                    f'r={["%.2f" % r for r in result.radii]}')
            self.k += 1

        msg = Leash()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drone_names = self.drone_names
        msg.radii = [float(d.r) for d in self.algo.drones]
        # gate_state is 1 if the algorithm's current iterate is feasible
        msg.gate_state = 1 if self.algo.G() <= 0.0 else 0
        self.pub.publish(msg)

    def _trigger_stop(self):
        """End the demo: land all drones once and latch /demo/stopped so the
        planner stops publishing targets and the streamer stops uploading
        trajectories (nothing fights the land). The optimizer then holds its
        last leash, so the whole demo freezes at stop_time_s."""
        self._stopped = True
        t_now = self.k / self._round_rate_hz if self._round_rate_hz else 0.0
        self.get_logger().info(
            f'DEMO STOP at k={self.k} (t={t_now:.0f}s): landing '
            f'{len(self.land_clients)} drones + latching /demo/stopped=True')
        self.stopped_pub.publish(Bool(data=True))
        dur = Duration()
        dur.sec = 3
        dur.nanosec = 0
        for cli in self.land_clients.values():
            req = Land.Request()
            req.group_mask = 0
            req.height = 0.0
            req.duration = dur
            cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = RadialCoverageNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
