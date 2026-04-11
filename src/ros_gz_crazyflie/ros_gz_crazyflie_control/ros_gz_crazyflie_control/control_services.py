"""High-level control services for Crazyflie drones in Gazebo simulation.

Provides Crazyswarm2-compatible ROS 2 services (takeoff, land, go_to, hover)
on top of cmd_vel velocity control for the Gazebo MulticopterVelocityControl
plugin.

Service definitions from crazyflie_interfaces are used for compatibility
with the Crazyswarm2 ecosystem (crazyflie_py, crazyflie_server, etc.).

Trajectory interpolation uses a quintic smoothstep (zero velocity and
acceleration at endpoints) with feedforward velocity for accurate tracking.
"""

import math

from crazyflie_interfaces.srv import (
    GoTo, Land, NotifySetpointsStop, StartTrajectory, Takeoff,
    UploadTrajectory,
)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from std_srvs.srv import Empty

import rclpy
from rclpy.node import Node


class ControlServices(Node):

    STATE_IDLE = 0
    STATE_TRAJECTORY = 1
    STATE_HOVERING = 2

    STATE_NAMES = {0: 'IDLE', 1: 'TRAJECTORY', 2: 'HOVERING'}

    TRAJ_THEN_HOVER = 0
    TRAJ_THEN_IDLE = 1

    TRAJ_MODE_SMOOTHSTEP = 0
    TRAJ_MODE_POLYNOMIAL = 1

    def __init__(self):
        super().__init__('control_services')

        # ── Parameters ──
        self.declare_parameter('hover_height', 0.5)
        self.declare_parameter('robot_prefix', '/crazyflie')
        self.declare_parameter('incoming_twist_topic', '/cmd_vel_teleop')
        self.declare_parameter('max_ang_z_rate', 0.4)
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('kp_xy', 1.0)
        self.declare_parameter('kd_xy', 0.0)
        self.declare_parameter('kp_z', 2.0)
        self.declare_parameter('kd_z', 0.0)
        self.declare_parameter('kp_yaw', 1.0)
        self.declare_parameter('max_vel_xy', 0.3)
        self.declare_parameter('max_vel_z', 2.0)
        self.declare_parameter('bounds_min', [-1.9, -1.9, 0.0])
        self.declare_parameter('bounds_max', [1.9, 1.9, 2.0])

        self.default_hover_height = self.get_parameter('hover_height').value
        robot_prefix = self.get_parameter('robot_prefix').value
        incoming_twist_topic = self.get_parameter('incoming_twist_topic').value
        self.max_ang_z_rate = self.get_parameter('max_ang_z_rate').value
        control_rate = self.get_parameter('control_rate_hz').value

        # Position controller gains (runtime-tunable via ros2 param set)
        self.kp_xy = self.get_parameter('kp_xy').value
        self.kd_xy = self.get_parameter('kd_xy').value
        self.kp_z = self.get_parameter('kp_z').value
        self.kd_z = self.get_parameter('kd_z').value
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.max_vel_xy = self.get_parameter('max_vel_xy').value
        self.max_vel_z = self.get_parameter('max_vel_z').value

        # Geofence bounds (runtime-tunable)
        self.bounds_min = list(self.get_parameter('bounds_min').value)
        self.bounds_max = list(self.get_parameter('bounds_max').value)

        # Runtime parameter update callback
        self.add_on_set_parameters_callback(self._param_cb)

        # ── State ──
        self.state = self.STATE_IDLE
        self.current_pose = Odometry().pose.pose
        self.current_twist = Odometry().twist.twist
        self.teleop_cmd = Twist()

        # Do not publish any commands until the first odom message arrives.
        # Publishing zero Twist before odom is ready can activate the Gazebo
        # MulticopterVelocityControl plugin prematurely (it interprets zero
        # velocity as "hover"), causing drones to fly off at sim startup.
        self.odom_received = False

        # Legacy teleop state (preserved for backward compatibility)
        self.is_flying = False
        self.keep_height = False
        self.desired_height = 0.0

        # Once a service is called, teleop passthrough is disabled.
        self.service_active = False

        # Trajectory interpolation state
        self.traj_start_pos = [0.0, 0.0, 0.0]
        self.traj_end_pos = [0.0, 0.0, 0.0]
        self.traj_start_yaw = 0.0
        self.traj_end_yaw = 0.0
        self.traj_duration = 1.0
        self.traj_start_time = 0.0
        self.traj_end_behavior = self.TRAJ_THEN_HOVER
        self.traj_mode = self.TRAJ_MODE_SMOOTHSTEP

        # Uploaded polynomial trajectories: {trajectory_id: [piece_dict, ...]}
        self.uploaded_trajectories = {}

        # Polynomial trajectory execution state
        self.poly_pieces = []
        self.poly_timescale = 1.0
        self.poly_reversed = False
        self.poly_offset = [0.0, 0.0, 0.0]
        self.poly_yaw_offset = 0.0
        self.poly_total_duration = 0.0

        # Hover setpoint
        self.hover_pos = [0.0, 0.0, 0.0]
        self.hover_yaw = 0.0

        # Active trajectory ID for status reporting
        self.active_traj_id = -1

        # Geofence warning throttle (avoid log spam)
        self._geofence_warn_count = 0

        # ── Publishers ──
        self.cmd_vel_pub = self.create_publisher(
            Twist, robot_prefix + '/cmd_vel', 10)
        self.status_pub = self.create_publisher(
            String, robot_prefix + '/status', 10)

        # ── Subscribers ──
        self.odom_sub = self.create_subscription(
            Odometry, robot_prefix + '/odom', self._odom_cb, 10)
        self.teleop_sub = self.create_subscription(
            Twist, robot_prefix + incoming_twist_topic,
            self._teleop_cb, 10)

        # ── Services (Crazyswarm2-compatible) ──
        self.create_service(
            Takeoff, robot_prefix + '/takeoff', self._takeoff_cb)
        self.create_service(
            Land, robot_prefix + '/land', self._land_cb)
        self.create_service(
            GoTo, robot_prefix + '/go_to', self._go_to_cb)
        self.create_service(
            Empty, robot_prefix + '/hover', self._hover_cb)
        self.create_service(
            Empty, robot_prefix + '/emergency', self._emergency_cb)
        self.create_service(
            UploadTrajectory, robot_prefix + '/upload_trajectory',
            self._upload_trajectory_cb)
        self.create_service(
            StartTrajectory, robot_prefix + '/start_trajectory',
            self._start_trajectory_cb)
        self.create_service(
            NotifySetpointsStop, robot_prefix + '/notify_setpoints_stop',
            self._notify_setpoints_stop_cb)

        # ── Control loop ──
        self.timer = self.create_timer(1.0 / control_rate, self._control_loop)

        self.get_logger().info(
            f'High-level control services ready for {robot_prefix} '
            f'at {control_rate:.0f} Hz')

    # ════════════════════ Parameter callback ════════════════════

    def _param_cb(self, params):
        for p in params:
            if p.name == 'kp_xy':
                self.kp_xy = p.value
            elif p.name == 'kd_xy':
                self.kd_xy = p.value
            elif p.name == 'kp_z':
                self.kp_z = p.value
            elif p.name == 'kd_z':
                self.kd_z = p.value
            elif p.name == 'kp_yaw':
                self.kp_yaw = p.value
            elif p.name == 'max_vel_xy':
                self.max_vel_xy = p.value
            elif p.name == 'max_vel_z':
                self.max_vel_z = p.value
            elif p.name == 'max_ang_z_rate':
                self.max_ang_z_rate = p.value
            elif p.name == 'bounds_min':
                self.bounds_min = list(p.value)
            elif p.name == 'bounds_max':
                self.bounds_max = list(p.value)
        return SetParametersResult(successful=True)

    # ════════════════════ Service callbacks ════════════════════

    def _takeoff_cb(self, request, response):
        height = request.height if request.height > 0 else self.default_hover_height
        duration = self._dur(request.duration)
        if duration <= 0:
            duration = 2.0

        self.service_active = True
        self._start_trajectory(
            start=[self.current_pose.position.x,
                   self.current_pose.position.y,
                   self.current_pose.position.z],
            end=[self.current_pose.position.x,
                 self.current_pose.position.y,
                 height],
            yaw_start=self._get_yaw(),
            yaw_end=self._get_yaw(),
            duration=duration,
            end_behavior=self.TRAJ_THEN_HOVER,
        )
        self.get_logger().info(
            f'Takeoff to {height:.2f}m over {duration:.1f}s')
        return response

    def _land_cb(self, request, response):
        height = request.height
        duration = self._dur(request.duration)
        if duration <= 0:
            duration = 2.0

        self.service_active = True
        self._start_trajectory(
            start=[self.current_pose.position.x,
                   self.current_pose.position.y,
                   self.current_pose.position.z],
            end=[self.current_pose.position.x,
                 self.current_pose.position.y,
                 height],
            yaw_start=self._get_yaw(),
            yaw_end=self._get_yaw(),
            duration=duration,
            end_behavior=self.TRAJ_THEN_IDLE,
        )
        self.get_logger().info(
            f'Landing to {height:.2f}m over {duration:.1f}s')
        return response

    def _go_to_cb(self, request, response):
        duration = self._dur(request.duration)
        if duration <= 0:
            duration = 3.0

        cur = [self.current_pose.position.x,
               self.current_pose.position.y,
               self.current_pose.position.z]
        cur_yaw = self._get_yaw()

        if request.relative:
            goal = [cur[0] + request.goal.x,
                    cur[1] + request.goal.y,
                    cur[2] + request.goal.z]
            goal_yaw = cur_yaw + math.radians(request.yaw)
        else:
            goal = [request.goal.x, request.goal.y, request.goal.z]
            goal_yaw = math.radians(request.yaw)

        # Clamp goal to geofence bounds
        goal = self._clamp_to_bounds(goal)

        if self.state == self.STATE_IDLE:
            self.get_logger().warn(
                'go_to called while idle — consider calling takeoff first')

        self.service_active = True
        self._start_trajectory(
            start=cur, end=goal,
            yaw_start=cur_yaw, yaw_end=goal_yaw,
            duration=duration,
            end_behavior=self.TRAJ_THEN_HOVER,
        )
        self.get_logger().info(
            f'GoTo ({goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f}) '
            f'over {duration:.1f}s')
        return response

    def _hover_cb(self, request, response):
        self.service_active = True
        self.hover_pos = self._clamp_to_bounds([
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.current_pose.position.z,
        ])
        self.hover_yaw = self._get_yaw()
        self.state = self.STATE_HOVERING
        self.active_traj_id = -1
        self.get_logger().info(
            f'Hover at ({self.hover_pos[0]:.2f}, '
            f'{self.hover_pos[1]:.2f}, {self.hover_pos[2]:.2f})')
        return response

    def _emergency_cb(self, request, response):
        self.state = self.STATE_IDLE
        self.service_active = False
        self.is_flying = False
        self.active_traj_id = -1
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().warn('EMERGENCY STOP')
        return response

    def _notify_setpoints_stop_cb(self, request, response):
        """Cancel any active trajectory and hold position."""
        if self.state == self.STATE_TRAJECTORY:
            self.hover_pos = self._clamp_to_bounds([
                self.current_pose.position.x,
                self.current_pose.position.y,
                self.current_pose.position.z,
            ])
            self.hover_yaw = self._get_yaw()
            self.state = self.STATE_HOVERING
            self.active_traj_id = -1
            self.get_logger().info(
                'NotifySetpointsStop: cancelled trajectory, hovering')
        elif self.state == self.STATE_HOVERING:
            self.get_logger().info('NotifySetpointsStop: already hovering')
        else:
            self.get_logger().info('NotifySetpointsStop: idle, no action')
        return response

    def _upload_trajectory_cb(self, request, response):
        traj_id = request.trajectory_id
        offset = request.piece_offset

        pieces = []
        for p in request.pieces:
            dur = float(p.duration.sec) + float(p.duration.nanosec) / 1e9
            pieces.append({
                'poly_x': list(p.poly_x),
                'poly_y': list(p.poly_y),
                'poly_z': list(p.poly_z),
                'poly_yaw': list(p.poly_yaw),
                'duration': dur,
            })

        if traj_id not in self.uploaded_trajectories:
            self.uploaded_trajectories[traj_id] = []

        traj = self.uploaded_trajectories[traj_id]
        required_len = offset + len(pieces)
        if len(traj) < required_len:
            traj.extend([None] * (required_len - len(traj)))

        for i, piece in enumerate(pieces):
            traj[offset + i] = piece

        total_dur = sum(pc['duration'] for pc in traj if pc is not None)
        self.get_logger().info(
            f'Uploaded trajectory {traj_id}: {len(pieces)} pieces '
            f'at offset {offset}, total duration {total_dur:.2f}s')
        return response

    def _start_trajectory_cb(self, request, response):
        traj_id = request.trajectory_id

        if traj_id not in self.uploaded_trajectories:
            self.get_logger().error(
                f'StartTrajectory: trajectory {traj_id} not uploaded')
            return response

        pieces = self.uploaded_trajectories[traj_id]
        if not pieces or any(p is None for p in pieces):
            self.get_logger().error(
                f'StartTrajectory: trajectory {traj_id} is incomplete')
            return response

        self.service_active = True
        self.active_traj_id = traj_id
        self.poly_pieces = pieces
        self.poly_timescale = max(request.timescale, 0.01)
        self.poly_reversed = request.reversed
        self.poly_total_duration = sum(p['duration'] for p in pieces)

        # Compute offset for relative mode
        if request.relative:
            if not request.reversed:
                ref_piece = pieces[0]
                ref_t = 0.0
            else:
                ref_piece = pieces[-1]
                ref_t = ref_piece['duration']

            traj_x = self._eval_poly(ref_piece['poly_x'], ref_t)
            traj_y = self._eval_poly(ref_piece['poly_y'], ref_t)
            traj_z = self._eval_poly(ref_piece['poly_z'], ref_t)
            traj_yaw = self._eval_poly(ref_piece['poly_yaw'], ref_t)

            self.poly_offset = [
                self.current_pose.position.x - traj_x,
                self.current_pose.position.y - traj_y,
                self.current_pose.position.z - traj_z,
            ]
            self.poly_yaw_offset = self._get_yaw() - traj_yaw
        else:
            self.poly_offset = [0.0, 0.0, 0.0]
            self.poly_yaw_offset = 0.0

        self.traj_start_time = self.get_clock().now().nanoseconds / 1e9
        self.traj_duration = self.poly_total_duration * self.poly_timescale
        self.traj_end_behavior = self.TRAJ_THEN_HOVER
        self.traj_mode = self.TRAJ_MODE_POLYNOMIAL
        self.state = self.STATE_TRAJECTORY

        self.get_logger().info(
            f'StartTrajectory {traj_id}: {len(pieces)} pieces, '
            f'duration={self.traj_duration:.2f}s, '
            f'timescale={self.poly_timescale}, '
            f'reversed={self.poly_reversed}, relative={request.relative}')
        return response

    # ════════════════════ Trajectory helpers ════════════════════

    def _start_trajectory(self, start, end, yaw_start, yaw_end,
                          duration, end_behavior):
        self.traj_start_pos = list(start)
        self.traj_end_pos = list(end)
        self.traj_start_yaw = yaw_start
        self.traj_end_yaw = yaw_end
        self.traj_duration = duration
        self.traj_start_time = self.get_clock().now().nanoseconds / 1e9
        self.traj_end_behavior = end_behavior
        self.traj_mode = self.TRAJ_MODE_SMOOTHSTEP
        self.active_traj_id = -1
        self.state = self.STATE_TRAJECTORY

    @staticmethod
    def _smoothstep(t):
        """Quintic smoothstep: s(t) = 6t^5 - 15t^4 + 10t^3.

        Produces zero velocity and zero acceleration at t=0 and t=1.
        """
        t = max(0.0, min(1.0, t))
        return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)

    @staticmethod
    def _smoothstep_deriv(t):
        """Derivative of quintic smoothstep for feedforward velocity."""
        t = max(0.0, min(1.0, t))
        return 30.0 * t * t * (t * (t - 2.0) + 1.0)

    @staticmethod
    def _eval_poly(coeffs, t):
        """Evaluate polynomial via Horner's method.

        Coefficients in ascending power order: coeffs[i] is coeff of t^i.
        """
        result = 0.0
        for i in range(len(coeffs) - 1, -1, -1):
            result = result * t + coeffs[i]
        return result

    @staticmethod
    def _eval_poly_deriv(coeffs, t):
        """Evaluate first derivative of polynomial via Horner's method."""
        result = 0.0
        for i in range(len(coeffs) - 1, 0, -1):
            result = result * t + i * coeffs[i]
        return result

    def _eval_poly_trajectory(self, elapsed):
        """Evaluate piecewise polynomial trajectory at elapsed wall-clock time.

        Returns (target_pos, target_yaw, ff_vel) for _position_controller.
        """
        timescale = self.poly_timescale
        total_dur = self.poly_total_duration

        # Wall-clock to trajectory-local time (undo timescale)
        traj_time = max(0.0, min(elapsed / timescale, total_dur))

        # Mirror time if reversed
        if self.poly_reversed:
            traj_time = total_dur - traj_time

        # Find active piece and local time within it
        local_t = traj_time
        piece = self.poly_pieces[-1]
        for p in self.poly_pieces:
            if local_t <= p['duration']:
                piece = p
                break
            local_t -= p['duration']
        else:
            local_t = piece['duration']

        # Evaluate position and yaw
        pos_x = self._eval_poly(piece['poly_x'], local_t)
        pos_y = self._eval_poly(piece['poly_y'], local_t)
        pos_z = self._eval_poly(piece['poly_z'], local_t)
        yaw = self._eval_poly(piece['poly_yaw'], local_t)

        # Evaluate velocity (derivative) for feedforward
        vel_x = self._eval_poly_deriv(piece['poly_x'], local_t)
        vel_y = self._eval_poly_deriv(piece['poly_y'], local_t)
        vel_z = self._eval_poly_deriv(piece['poly_z'], local_t)

        # Scale velocity: d(pos)/d(wall_time) = d(pos)/d(traj_time) / timescale
        vel_scale = -1.0 / timescale if self.poly_reversed else 1.0 / timescale

        target_pos = [
            pos_x + self.poly_offset[0],
            pos_y + self.poly_offset[1],
            pos_z + self.poly_offset[2],
        ]
        target_yaw = yaw + self.poly_yaw_offset
        ff_vel = [vel_x * vel_scale, vel_y * vel_scale, vel_z * vel_scale]

        return target_pos, target_yaw, ff_vel

    # ════════════════════ Geofence ════════════════════

    def _clamp_to_bounds(self, pos):
        """Clamp position to geofence bounds."""
        clamped = [
            max(self.bounds_min[0], min(self.bounds_max[0], pos[0])),
            max(self.bounds_min[1], min(self.bounds_max[1], pos[1])),
            max(self.bounds_min[2], min(self.bounds_max[2], pos[2])),
        ]
        if clamped != [pos[0], pos[1], pos[2]]:
            self._geofence_warn_count += 1
            if self._geofence_warn_count <= 5 or self._geofence_warn_count % 50 == 0:
                self.get_logger().warn(
                    f'Geofence: clamped ({pos[0]:.2f}, {pos[1]:.2f}, '
                    f'{pos[2]:.2f}) -> ({clamped[0]:.2f}, {clamped[1]:.2f}, '
                    f'{clamped[2]:.2f})')
        return clamped

    # ════════════════════ Status publishing ════════════════════

    def _publish_status(self):
        """Publish current state, time remaining, and active trajectory ID."""
        remaining = 0.0
        if self.state == self.STATE_TRAJECTORY:
            now = self.get_clock().now().nanoseconds / 1e9
            elapsed = now - self.traj_start_time
            remaining = max(0.0, self.traj_duration - elapsed)

        msg = String()
        msg.data = (
            f'state:{self.STATE_NAMES[self.state]} '
            f'remaining:{remaining:.3f} '
            f'traj_id:{self.active_traj_id}'
        )
        self.status_pub.publish(msg)

    # ════════════════════ Control loop ════════════════════

    def _control_loop(self):
        if not self.odom_received:
            return  # Wait for first odom before sending any commands
        if self.service_active:
            self._control_loop_service()
        else:
            self._control_loop_teleop()
        self._publish_status()

    def _control_loop_service(self):
        """Control loop when high-level services are active."""
        if self.state == self.STATE_IDLE:
            self.cmd_vel_pub.publish(Twist())
            return

        if self.state == self.STATE_TRAJECTORY:
            now = self.get_clock().now().nanoseconds / 1e9
            elapsed = now - self.traj_start_time
            t = elapsed / self.traj_duration

            if t >= 1.0:
                # Trajectory complete
                if self.traj_end_behavior == self.TRAJ_THEN_HOVER:
                    if self.traj_mode == self.TRAJ_MODE_SMOOTHSTEP:
                        self.hover_pos = self._clamp_to_bounds(
                            list(self.traj_end_pos))
                        self.hover_yaw = self.traj_end_yaw
                    else:
                        final_pos, final_yaw, _ = self._eval_poly_trajectory(
                            self.traj_duration)
                        self.hover_pos = self._clamp_to_bounds(final_pos)
                        self.hover_yaw = final_yaw
                    self.state = self.STATE_HOVERING
                    self.active_traj_id = -1
                    self.get_logger().info('Trajectory complete -> hovering')
                else:
                    # Landing complete
                    self.state = self.STATE_IDLE
                    self.service_active = False
                    self.is_flying = False
                    self.active_traj_id = -1
                    self.cmd_vel_pub.publish(Twist())
                    self.get_logger().info('Landing complete -> idle')
                    return

        if self.state == self.STATE_TRAJECTORY:
            now = self.get_clock().now().nanoseconds / 1e9
            elapsed = now - self.traj_start_time

            if self.traj_mode == self.TRAJ_MODE_SMOOTHSTEP:
                t = elapsed / self.traj_duration
                s = self._smoothstep(t)
                s_dot = self._smoothstep_deriv(t) / self.traj_duration

                target = [
                    self.traj_start_pos[i]
                    + s * (self.traj_end_pos[i] - self.traj_start_pos[i])
                    for i in range(3)
                ]
                target_yaw = (self.traj_start_yaw
                              + s * (self.traj_end_yaw - self.traj_start_yaw))
                ff = [
                    s_dot * (self.traj_end_pos[i] - self.traj_start_pos[i])
                    for i in range(3)
                ]
            else:
                target, target_yaw, ff = self._eval_poly_trajectory(elapsed)

            cmd = self._position_controller(target, target_yaw, ff)
            self.cmd_vel_pub.publish(cmd)
            return

        if self.state == self.STATE_HOVERING:
            cmd = self._position_controller(
                self.hover_pos, self.hover_yaw, [0.0, 0.0, 0.0])
            self.cmd_vel_pub.publish(cmd)

    def _position_controller(self, target_pos, target_yaw, ff_vel):
        """PD controller with feedforward: position error -> velocity command."""
        # Clamp target to geofence bounds
        target_pos = self._clamp_to_bounds(target_pos)

        cmd = Twist()

        ex = target_pos[0] - self.current_pose.position.x
        ey = target_pos[1] - self.current_pose.position.y
        ez = target_pos[2] - self.current_pose.position.z

        vx = self.kp_xy * ex - self.kd_xy * self.current_twist.linear.x + ff_vel[0]
        vy = self.kp_xy * ey - self.kd_xy * self.current_twist.linear.y + ff_vel[1]

        # Clamp total horizontal velocity magnitude (not per-axis) to
        # prevent excessive tilt that would cause altitude loss.
        vel_xy = math.sqrt(vx * vx + vy * vy)
        if vel_xy > self.max_vel_xy:
            scale = self.max_vel_xy / vel_xy
            vx *= scale
            vy *= scale

        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = self._clamp(
            self.kp_z * ez - self.kd_z * self.current_twist.linear.z + ff_vel[2],
            self.max_vel_z)

        yaw_err = self._wrap_angle(target_yaw - self._get_yaw())
        cmd.angular.z = self._clamp(
            self.kp_yaw * yaw_err, self.max_ang_z_rate)

        return cmd

    # ════════════════════ Legacy teleop loop ════════════════════

    def _control_loop_teleop(self):
        """Original teleop control logic (preserved for backward compat)."""
        msg = self.teleop_cmd
        height_command = msg.linear.z
        new_cmd_msg = Twist()

        if self.is_flying:
            new_cmd_msg.linear.x = msg.linear.x
            new_cmd_msg.linear.y = msg.linear.y
            new_cmd_msg.linear.z = msg.linear.z
            new_cmd_msg.angular.x = msg.angular.x
            new_cmd_msg.angular.y = msg.angular.y
            new_cmd_msg.angular.z = msg.angular.z

        if height_command > 0 and not self.is_flying:
            new_cmd_msg.linear.z = 0.5
            if self.current_pose.position.z > self.default_hover_height:
                new_cmd_msg.linear.z = 0.0
                self.teleop_cmd.linear.z = 0.0
                self.is_flying = True
                self.get_logger().info('Takeoff completed (teleop)')

        if height_command < 0 and self.is_flying:
            if self.current_pose.position.z < 0.1:
                new_cmd_msg.linear.z = 0.0
                self.is_flying = False
                self.keep_height = False
                self.get_logger().info('Landing completed (teleop)')

        if abs(msg.angular.z) > self.max_ang_z_rate:
            new_cmd_msg.angular.z = (
                self.max_ang_z_rate * abs(msg.angular.z) / msg.angular.z)

        tolerance = 1e-7
        if abs(height_command) < tolerance and self.is_flying:
            if not self.keep_height:
                self.desired_height = self.current_pose.position.z
                self.keep_height = True
            else:
                error = self.desired_height - self.current_pose.position.z
                new_cmd_msg.linear.z = error

        if abs(height_command) > tolerance and self.is_flying:
            if self.keep_height:
                self.keep_height = False

        self.cmd_vel_pub.publish(new_cmd_msg)

    # ════════════════════ Subscriber callbacks ════════════════════

    def _odom_cb(self, msg):
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist
        if not self.odom_received:
            self.odom_received = True
            # Auto-hover at spawn position to prevent freefall on startup
            self.service_active = True
            self.hover_pos = [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ]
            self.hover_yaw = self._get_yaw()
            self.state = self.STATE_HOVERING
            self.get_logger().info(
                f'Odometry received, auto-hovering at z={self.hover_pos[2]:.3f}')

    def _teleop_cb(self, msg):
        self.teleop_cmd = msg

    # ════════════════════ Utilities ════════════════════

    def _get_yaw(self):
        q = self.current_pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    @staticmethod
    def _dur(duration):
        return float(duration.sec) + float(duration.nanosec) / 1e9

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    @staticmethod
    def _wrap_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    control_services = ControlServices()
    rclpy.spin(control_services)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
