"""Fed-DCSA Coordinator Node.

ROS2 node that orchestrates the Fed-DCSA experiment on top of the existing
Gazebo/Crazyflie control layer. Manages 4 surveillance stations, each with
an active drone and a standby drone.

State machine:
    INIT -> WAIT_SERVICES -> TAKEOFF_ACTIVE -> GOTO_SURVEY -> RUN_ROUNDS -> DONE
"""

import json

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from crazyflie_interfaces.srv import GoTo, Land, Takeoff
from geometry_msgs.msg import Point
from std_msgs.msg import Int32, String
from std_srvs.srv import Empty

from fed_dcsa.algorithm import FedDCSA, StationState
from fed_dcsa.csv_logger import CSVLogger


# State machine states
INIT = 0
WAIT_SERVICES = 1
TAKEOFF_ACTIVE = 2
GOTO_SURVEY = 3
RUN_ROUNDS = 4
SWAP_GOTO_PAD = 5
SWAP_LAND = 6
SWAP_TAKEOFF = 7
SWAP_GOTO_SURVEY = 8
DONE = 9

STATE_NAMES = {
    INIT: 'INIT', WAIT_SERVICES: 'WAIT_SERVICES',
    TAKEOFF_ACTIVE: 'TAKEOFF_ACTIVE', GOTO_SURVEY: 'GOTO_SURVEY',
    RUN_ROUNDS: 'RUN_ROUNDS',
    SWAP_GOTO_PAD: 'SWAP_GOTO_PAD', SWAP_LAND: 'SWAP_LAND',
    SWAP_TAKEOFF: 'SWAP_TAKEOFF', SWAP_GOTO_SURVEY: 'SWAP_GOTO_SURVEY',
    DONE: 'DONE',
}


class CoordinatorNode(Node):

    def __init__(self):
        super().__init__('fed_dcsa')

        # ── Declare and load parameters ──
        self.declare_parameter('K', 20)
        self.declare_parameter('T', 18)
        self.declare_parameter('tau', 0.7)
        self.declare_parameter('delta', 0.1)
        self.declare_parameter('E_max', 170.0)
        self.declare_parameter('E_thr', 50.0)
        self.declare_parameter('e_rates', [10.0, 12.0, 8.0, 15.0])
        self.declare_parameter('q_weights', [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('station_active', ['cf1', 'cf2', 'cf3', 'cf4'])
        self.declare_parameter('station_standby', ['cf5', 'cf6', 'cf7', 'cf8'])
        self.declare_parameter('charge_pad_x', [-1.5, 1.5, -1.5, 1.5])
        self.declare_parameter('charge_pad_y', [-1.5, -1.5, 1.5, 1.5])
        self.declare_parameter('survey_target_x', [-0.5, 0.5, -0.5, 0.5])
        self.declare_parameter('survey_target_y', [-0.5, -0.5, 0.5, 0.5])
        self.declare_parameter('patrol_height', 0.5)
        self.declare_parameter('round_period', 12.0)
        self.declare_parameter('land_duration', 2.0)
        self.declare_parameter('takeoff_duration', 2.0)
        self.declare_parameter('goto_duration', 4.0)
        self.declare_parameter('log_dir', '/home/rk32226/cs2_ws/fed_dcsa_logs')
        self.declare_parameter('experiment_name', 'gazebo_run')

        self.K = self.get_parameter('K').value
        self.T = self.get_parameter('T').value
        self.tau = self.get_parameter('tau').value
        self.delta = self.get_parameter('delta').value
        self.E_max = self.get_parameter('E_max').value
        self.E_thr = self.get_parameter('E_thr').value
        self.e_rates = list(self.get_parameter('e_rates').value)
        self.q_weights = list(self.get_parameter('q_weights').value)
        self.active_names = list(self.get_parameter('station_active').value)
        self.standby_names = list(self.get_parameter('station_standby').value)
        self.charge_pad_x = list(self.get_parameter('charge_pad_x').value)
        self.charge_pad_y = list(self.get_parameter('charge_pad_y').value)
        self.survey_x = list(self.get_parameter('survey_target_x').value)
        self.survey_y = list(self.get_parameter('survey_target_y').value)
        self.patrol_height = self.get_parameter('patrol_height').value
        self.round_period = self.get_parameter('round_period').value
        self.land_dur = self.get_parameter('land_duration').value
        self.takeoff_dur = self.get_parameter('takeoff_duration').value
        self.goto_dur = self.get_parameter('goto_duration').value
        self.log_dir = self.get_parameter('log_dir').value
        self.experiment_name = self.get_parameter('experiment_name').value

        self.n_stations = len(self.e_rates)
        self.all_drones = list(set(self.active_names + self.standby_names))

        # ── Initialize algorithm ──
        self.stations = []
        for i in range(self.n_stations):
            self.stations.append(StationState(
                station_id=i, x=1.0, battery=self.E_max,
                e_rate=self.e_rates[i], q_weight=self.q_weights[i],
                active_drone=self.active_names[i],
                standby_drone=self.standby_names[i],
            ))

        self.algo = FedDCSA(
            K=self.K, T=self.T, tau=self.tau, delta=self.delta,
            E_max=self.E_max, E_thr=self.E_thr,
            e_rates=self.e_rates, q_weights=self.q_weights,
        )

        self.logger = CSVLogger(self.log_dir, self.experiment_name, self.n_stations)

        # ── Service clients (for all 8 drones) ──
        self.takeoff_clients = {}
        self.land_clients = {}
        self.goto_clients = {}
        self.hover_clients = {}

        for name in self.all_drones:
            self.takeoff_clients[name] = self.create_client(Takeoff, f'/{name}/takeoff')
            self.land_clients[name] = self.create_client(Land, f'/{name}/land')
            self.goto_clients[name] = self.create_client(GoTo, f'/{name}/go_to')
            self.hover_clients[name] = self.create_client(Empty, f'/{name}/hover')

        # ── Status subscribers ──
        self.drone_status = {}  # drone_name -> latest status string
        for name in self.all_drones:
            self.drone_status[name] = ''
            self.create_subscription(
                String, f'/{name}/status',
                lambda msg, n=name: self._status_cb(n, msg), 10)

        # ── Publishers ──
        self.gate_pub = self.create_publisher(Int32, '/fed_dcsa/gate', 10)
        self.round_pub = self.create_publisher(String, '/fed_dcsa/round_status', 10)

        # ── State machine ──
        self.state = INIT
        self.current_round = 0
        self.pending_acks = set()  # drone names we're waiting for
        self.swap_stations = []    # station indices needing swap this round
        self.current_result = None
        self.wait_deadline = None  # sim-time deadline for timed waits

        self.timer = self.create_timer(0.5, self._tick)

        self.get_logger().info(
            f'Fed-DCSA Coordinator: {self.n_stations} stations, '
            f'K={self.K}, T={self.T}, gamma={self.algo.gamma:.6f}')

    # ════════════════════ Status tracking ════════════════════

    def _status_cb(self, drone_name, msg):
        self.drone_status[drone_name] = msg.data

    def _drone_state(self, drone_name):
        """Parse drone state from status string."""
        s = self.drone_status.get(drone_name, '')
        if 'state:' not in s:
            return ''
        parts = dict(p.split(':') for p in s.split())
        return parts.get('state', '')

    def _all_in_state(self, drone_names, target_state):
        """Check if all named drones report the target state."""
        return all(self._drone_state(n) == target_state for n in drone_names)

    def _now_sec(self):
        """Current sim time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def _set_deadline(self, seconds_from_now):
        self.wait_deadline = self._now_sec() + seconds_from_now

    def _deadline_passed(self):
        return self._now_sec() >= self.wait_deadline

    # ════════════════════ Service call helpers ════════════════════

    def _make_duration(self, sec):
        d = Duration()
        d.sec = int(sec)
        d.nanosec = int((sec - d.sec) * 1e9)
        return d

    def _call_takeoff(self, drone_name, height, duration):
        req = Takeoff.Request()
        req.group_mask = 0
        req.height = float(height)
        req.duration = self._make_duration(duration)
        future = self.takeoff_clients[drone_name].call_async(req)
        future.add_done_callback(
            lambda f, n=drone_name: self._on_service_done(f, n, 'takeoff'))

    def _call_land(self, drone_name, duration):
        req = Land.Request()
        req.group_mask = 0
        req.height = 0.0
        req.duration = self._make_duration(duration)
        future = self.land_clients[drone_name].call_async(req)
        future.add_done_callback(
            lambda f, n=drone_name: self._on_service_done(f, n, 'land'))

    def _call_goto(self, drone_name, x, y, z, duration):
        req = GoTo.Request()
        req.group_mask = 0
        req.relative = False
        req.goal = Point(x=float(x), y=float(y), z=float(z))
        req.yaw = 0.0
        req.duration = self._make_duration(duration)
        future = self.goto_clients[drone_name].call_async(req)
        future.add_done_callback(
            lambda f, n=drone_name: self._on_service_done(f, n, 'go_to'))

    def _on_service_done(self, future, drone_name, service_name):
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f'{drone_name} {service_name} failed: {e}')

    # ════════════════════ State machine tick ════════════════════

    def _tick(self):
        if self.state == INIT:
            self._do_init()
        elif self.state == WAIT_SERVICES:
            self._do_wait_services()
        elif self.state == TAKEOFF_ACTIVE:
            self._do_takeoff_active()
        elif self.state == GOTO_SURVEY:
            self._do_goto_survey()
        elif self.state == RUN_ROUNDS:
            self._do_run_round()
        elif self.state == SWAP_GOTO_PAD:
            self._do_swap_goto_pad()
        elif self.state == SWAP_LAND:
            self._do_swap_land()
        elif self.state == SWAP_TAKEOFF:
            self._do_swap_takeoff()
        elif self.state == SWAP_GOTO_SURVEY:
            self._do_swap_goto_survey()
        elif self.state == DONE:
            pass

    def _transition(self, new_state):
        self.get_logger().info(
            f'State: {STATE_NAMES[self.state]} -> {STATE_NAMES[new_state]}')
        self.state = new_state

    # ── INIT ──
    def _do_init(self):
        self.get_logger().info('Checking service availability...')
        self._transition(WAIT_SERVICES)

    # ── WAIT_SERVICES ──
    def _do_wait_services(self):
        # Only check active drones initially (standbys may not have odom yet)
        all_ready = True
        for name in self.active_names:
            if not self.takeoff_clients[name].service_is_ready():
                all_ready = False
                break
            if not self.goto_clients[name].service_is_ready():
                all_ready = False
                break

        if all_ready:
            self.get_logger().info('All services ready. Taking off active drones...')
            for i in range(self.n_stations):
                name = self.stations[i].active_drone
                self._call_takeoff(name, self.patrol_height, self.takeoff_dur)
            self._set_deadline(self.takeoff_dur + 2.0)
            self._transition(TAKEOFF_ACTIVE)

    # ── TAKEOFF_ACTIVE ──
    def _do_takeoff_active(self):
        active_drones = [s.active_drone for s in self.stations]
        if self._all_in_state(active_drones, 'HOVERING') or self._deadline_passed():
            self.get_logger().info('Active drones airborne. Flying to survey positions...')
            for i in range(self.n_stations):
                name = self.stations[i].active_drone
                self._call_goto(name, self.survey_x[i], self.survey_y[i],
                                self.patrol_height, self.goto_dur)
            self._set_deadline(self.goto_dur + 2.0)
            self._transition(GOTO_SURVEY)

    # ── GOTO_SURVEY ──
    def _do_goto_survey(self):
        active_drones = [s.active_drone for s in self.stations]
        if self._all_in_state(active_drones, 'HOVERING') or self._deadline_passed():
            self.get_logger().info('All active drones at survey positions. Starting algorithm.')
            self._transition(RUN_ROUNDS)

    # ── RUN_ROUNDS ──
    def _do_run_round(self):
        if self.current_round >= self.K:
            self._finish_experiment()
            return

        k = self.current_round

        # Run algorithm round (pure computation)
        batteries_before = [s.battery for s in self.stations]
        result = self.algo.run_round(k, self.stations)
        self.current_result = result

        # Log swap events
        for sid in result.swaps:
            self.logger.log_swap(
                k, sid,
                old_active=self.stations[sid].standby_drone,
                new_active=self.stations[sid].active_drone,
                battery_at_swap=batteries_before[sid],
            )
        self.logger.log_round(result, self.stations)

        # Publish gate decision
        gate_msg = Int32()
        gate_msg.data = result.b_k
        self.gate_pub.publish(gate_msg)

        # Publish round status as JSON
        status = {
            'k': k, 'b_k': result.b_k,
            'G_tilde': round(result.G_tilde, 4),
            'eta_k': round(result.eta_k, 4),
            'stations': [
                {'id': i, 'x': round(result.x_after[i], 4),
                 'a': result.actions[i],
                 'battery': round(self.stations[i].battery, 1),
                 'drone': self.stations[i].active_drone}
                for i in range(self.n_stations)
            ],
            'swaps': result.swaps,
        }
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.round_pub.publish(status_msg)

        # Terminal output
        self._print_round_summary(result)

        # Check if swaps are needed
        self.swap_stations = list(result.swaps)
        if self.swap_stations:
            # Start swap sequence: first fly active drones back to pads
            for sid in self.swap_stations:
                # The swap already happened in algorithm (names flipped),
                # so standby_drone is actually the one currently airborne
                airborne = self.stations[sid].standby_drone
                self._call_goto(airborne, self.charge_pad_x[sid],
                                self.charge_pad_y[sid],
                                self.patrol_height, self.goto_dur)
            self._set_deadline(self.goto_dur + 2.0)
            self._transition(SWAP_GOTO_PAD)
        else:
            # No swaps, advance to next round after round_period
            self.current_round += 1
            self._set_deadline(self.round_period)
            # Stay in RUN_ROUNDS but wait for deadline

    # ── SWAP sequence ──
    def _do_swap_goto_pad(self):
        airborne = [self.stations[sid].standby_drone for sid in self.swap_stations]
        if self._all_in_state(airborne, 'HOVERING') or self._deadline_passed():
            # Land the old active drones (now over their pads)
            for sid in self.swap_stations:
                name = self.stations[sid].standby_drone
                self._call_land(name, self.land_dur)
            self._set_deadline(self.land_dur + 2.0)
            self._transition(SWAP_LAND)

    def _do_swap_land(self):
        landing = [self.stations[sid].standby_drone for sid in self.swap_stations]
        if self._all_in_state(landing, 'IDLE') or self._deadline_passed():
            # Takeoff the new active drones (former standbys, now on pads)
            for sid in self.swap_stations:
                name = self.stations[sid].active_drone
                # Check if takeoff service is ready for this drone
                if not self.takeoff_clients[name].service_is_ready():
                    self.get_logger().warn(f'Takeoff service not ready for {name}, waiting...')
                    return
                self._call_takeoff(name, self.patrol_height, self.takeoff_dur)
            self._set_deadline(self.takeoff_dur + 2.0)
            self._transition(SWAP_TAKEOFF)

    def _do_swap_takeoff(self):
        new_actives = [self.stations[sid].active_drone for sid in self.swap_stations]
        if self._all_in_state(new_actives, 'HOVERING') or self._deadline_passed():
            # Fly new actives to survey positions
            for sid in self.swap_stations:
                name = self.stations[sid].active_drone
                self._call_goto(name, self.survey_x[sid], self.survey_y[sid],
                                self.patrol_height, self.goto_dur)
            self._set_deadline(self.goto_dur + 2.0)
            self._transition(SWAP_GOTO_SURVEY)

    def _do_swap_goto_survey(self):
        new_actives = [self.stations[sid].active_drone for sid in self.swap_stations]
        if self._all_in_state(new_actives, 'HOVERING') or self._deadline_passed():
            self.get_logger().info('Swap sequence complete.')
            self.swap_stations = []
            self.current_round += 1
            self._transition(RUN_ROUNDS)

    # ── DONE ──
    def _finish_experiment(self):
        self.get_logger().info('Experiment complete. Landing all airborne drones...')

        # Land all currently active drones
        for s in self.stations:
            name = s.active_drone
            if self._drone_state(name) in ('HOVERING', 'TRAJECTORY'):
                self._call_goto(name, self.charge_pad_x[s.station_id],
                                self.charge_pad_y[s.station_id],
                                self.patrol_height, self.goto_dur)

        self.logger.close()

        x_bar = self.algo.get_weighted_output()
        self.get_logger().info(
            f'Weighted output x_bar: {[f"{x:.4f}" for x in x_bar]}')
        self.get_logger().info(f'CSV logs at: {self.log_dir}/')

        self._transition(DONE)

    # ── Terminal output ──
    def _print_round_summary(self, result):
        gate_str = '\033[92mFEASIBLE\033[0m' if result.b_k == 1 else '\033[91mINFEASIBLE\033[0m'
        self.get_logger().info(
            f'\u2550\u2550\u2550 Round {result.k}/{self.K} \u2550\u2550\u2550  '
            f'Gate: {gate_str} | '
            f'G\u0303={result.G_tilde:.4f}, \u03b7={result.eta_k:.4f}')
        for i, s in enumerate(self.stations):
            action_str = 'ACTIVE' if result.actions[i] == 1 else '\033[93mSWAP\033[0m'
            self.get_logger().info(
                f'  Station {i} ({s.active_drone}): '
                f'x={result.x_before[i]:.3f}\u2192{result.x_after[i]:.3f} '
                f'{action_str} | Batt: {s.battery:.0f}/{self.E_max:.0f} Wh')


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
