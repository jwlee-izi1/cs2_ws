#!/usr/bin/env python3
"""Phase A — controller tuning for 1 m/s XY trajectory tracking.

Uploads a synthetic polynomial trajectory to the shared control_services
node (single-agent, cf1) via upload_trajectory + start_trajectory. Sets
controller gains at runtime via ROS2 param API (the shared launch does not
thread gains through, and we do not edit the shared launch). Records odom,
computes tracking error against the commanded polynomial, and saves a CSV
+ matplotlib plot.

Run after `ros2 launch rl_demo tune.launch.py` is up.

Usage:
    ros2 run rl_demo tune_tracking.py \\
        --kp-xy 3.0 --kd-xy 0.3 --max-vel-xy 1.5 \\
        --speed 1.0 --tag kpxy3_kdxy03
"""

import argparse
import csv
import math
import os
import time
from datetime import datetime
from threading import Event, Lock

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from builtin_interfaces.msg import Duration as DurationMsg
from crazyflie_interfaces.msg import TrajectoryPolynomialPiece
from crazyflie_interfaces.srv import (
    GoTo, StartTrajectory, Takeoff, UploadTrajectory,
)
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters


DRONE = 'cf1'
CONTROL_NODE = f'control_services_{DRONE}'
HOVER_Z = 0.5


# ════════════════════ Cubic Hermite fit ════════════════════

def fit_hermite_pieces(xs, ys, ts, z_const=HOVER_Z):
    """Fit piecewise cubic Hermite through waypoints (xs[i], ys[i], ts[i]).

    Per-waypoint velocity from central finite difference (one-sided at ends).
    Per segment [i, i+1], local time u in [0, dt], position
        p(u) = a0 + a1 u + a2 u^2 + a3 u^3
    satisfying p(0)=p_i, p(dt)=p_{i+1}, p'(0)=v_i, p'(dt)=v_{i+1}.

    Closed form:
        a0 = p0
        a1 = v0
        a2 = 3 (p1-p0)/dt^2 - (2 v0 + v1)/dt
        a3 = -2 (p1-p0)/dt^3 + (v0 + v1)/dt^2

    z and yaw: constants (z_const, 0).

    Returns list of TrajectoryPolynomialPiece.
    """
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    ts = np.asarray(ts, dtype=float)
    n = len(ts)
    assert n >= 2

    vxs = np.gradient(xs, ts)
    vys = np.gradient(ys, ts)

    pieces = []
    for i in range(n - 1):
        dt = ts[i + 1] - ts[i]
        if dt <= 0:
            continue

        def cubic(p0, p1, v0, v1, T):
            a0 = p0
            a1 = v0
            a2 = 3.0 * (p1 - p0) / (T * T) - (2.0 * v0 + v1) / T
            a3 = -2.0 * (p1 - p0) / (T ** 3) + (v0 + v1) / (T * T)
            return [float(a0), float(a1), float(a2), float(a3)]

        px = cubic(xs[i], xs[i + 1], vxs[i], vxs[i + 1], dt)
        py = cubic(ys[i], ys[i + 1], vys[i], vys[i + 1], dt)
        pz = [float(z_const), 0.0, 0.0, 0.0]
        pyaw = [0.0, 0.0, 0.0, 0.0]

        piece = TrajectoryPolynomialPiece()
        piece.poly_x = px
        piece.poly_y = py
        piece.poly_z = pz
        piece.poly_yaw = pyaw
        piece.duration = _to_duration_msg(dt)
        pieces.append(piece)

    return pieces


def _to_duration_msg(sec):
    d = DurationMsg()
    d.sec = int(sec)
    d.nanosec = int((sec - int(sec)) * 1e9)
    return d


def eval_hermite_at(pieces_raw, t):
    """Evaluate raw piecewise cubic (list of (px,py,dur)) at trajectory time t.

    Returns (x, y, vx, vy). Used for local error computation against the
    same trajectory the controller is executing.
    """
    total = 0.0
    for px, py, dur in pieces_raw:
        if t <= total + dur:
            u = t - total
            x = px[0] + px[1] * u + px[2] * u * u + px[3] * u ** 3
            y = py[0] + py[1] * u + py[2] * u * u + py[3] * u ** 3
            vx = px[1] + 2 * px[2] * u + 3 * px[3] * u * u
            vy = py[1] + 2 * py[2] * u + 3 * py[3] * u * u
            return x, y, vx, vy
        total += dur
    # past end: hold at last waypoint velocity zero (extrapolate last poly at its end)
    px, py, dur = pieces_raw[-1]
    u = dur
    x = px[0] + px[1] * u + px[2] * u * u + px[3] * u ** 3
    y = py[0] + py[1] * u + py[2] * u * u + py[3] * u ** 3
    return x, y, 0.0, 0.0


# ════════════════════ Synthetic trajectory ════════════════════

def make_synthetic_trajectory(speed=1.0, dt=0.1, start_xy=(-1.5, 0.0)):
    """Build a 2D test trajectory that starts/ends at rest and has a
    rounded corner (safe to fly; exercises steady-state, turning, and
    stop-and-go). Sampled at dt.

    Segments:
      A. Ramp-in: 0 -> speed over 0.5 s along +X (smooth accel).
      B. Straight leg +X at constant speed (steady-state).
      C. Rounded 90-deg turn (quarter-arc, radius 0.4 m, traversed at speed).
      D. Straight leg +Y at constant speed.
      E. Ramp-out to rest over 0.5 s.
      F. Hold at rest for 1 s (stop).
      G. Ramp-in again -> speed along -X over 0.5 s (stop-and-go).
      H. Straight leg -X at constant speed.
      I. Ramp-out to rest.
    """
    x, y = start_xy
    xs, ys, ts = [x], [y], [0.0]

    def append(xn, yn, dur=dt):
        xs.append(xn); ys.append(yn); ts.append(ts[-1] + dur)

    def ramp(dir_x, dir_y, v_start, v_end, ramp_dur):
        """Append ramp from current pos along (dir_x, dir_y) (unit), changing
        speed linearly from v_start to v_end over ramp_dur. Uses dt sampling.
        Returns final speed."""
        n = max(1, int(round(ramp_dur / dt)))
        for k in range(1, n + 1):
            s = k / n
            v = v_start + s * (v_end - v_start)
            # trapezoidal integration along this step
            v_prev = v_start + ((k - 1) / n) * (v_end - v_start)
            step = 0.5 * (v_prev + v) * dt
            append(xs[-1] + dir_x * step, ys[-1] + dir_y * step)

    def straight(dir_x, dir_y, length, v):
        n = max(1, int(round((length / v) / dt)))
        for _ in range(n):
            append(xs[-1] + dir_x * v * dt, ys[-1] + dir_y * v * dt)

    # A. Ramp-in +X
    ramp(1.0, 0.0, 0.0, speed, 0.5)
    # B. Straight +X, 2.0 m at speed
    straight(1.0, 0.0, 2.0, speed)
    # C. Rounded quarter-turn (from +X to +Y), radius R=0.4 m
    R = 0.4
    arc_len = 0.5 * math.pi * R  # ~0.628 m
    arc_dur = arc_len / speed
    n_arc = max(4, int(round(arc_dur / dt)))
    cx = xs[-1]
    cy_center = ys[-1] + R
    theta0 = -math.pi / 2  # start at angle -π/2 on the circle (directly below center)
    for k in range(1, n_arc + 1):
        s = k / n_arc
        theta = theta0 + s * (math.pi / 2)
        append(cx + R * math.cos(theta), cy_center + R * math.sin(theta))
    # D. Straight +Y, 1.0 m at speed
    straight(0.0, 1.0, 1.0, speed)
    # E. Ramp-out to rest
    ramp(0.0, 1.0, speed, 0.0, 0.5)
    # F. Hold at rest
    n_hold = int(round(1.0 / dt))
    for _ in range(n_hold):
        append(xs[-1], ys[-1])
    # G. Ramp-in -X
    ramp(-1.0, 0.0, 0.0, speed, 0.5)
    # H. Straight -X, 1.5 m at speed
    straight(-1.0, 0.0, 1.5, speed)
    # I. Ramp-out to rest
    ramp(-1.0, 0.0, speed, 0.0, 0.5)

    return np.array(xs), np.array(ys), np.array(ts)


# ════════════════════ ROS node ════════════════════

class TuningNode(Node):

    def __init__(self, args):
        super().__init__('tune_tracking')
        self.args = args
        self.cb = ReentrantCallbackGroup()

        # State
        self.odom_lock = Lock()
        self.odom_log = []  # (sim_t_relative, x, y, z, vx, vy)
        self.odom_received = Event()
        self.recording = False
        self.t0_sim = None  # sim time (from odom header) at first recorded sample

        # Subscribers
        self.create_subscription(
            Odometry, f'/{DRONE}/odom', self._odom_cb, 50,
            callback_group=self.cb)

        # Service clients: gain setter + drone commands
        self.set_params_cli = self.create_client(
            SetParameters, f'/{CONTROL_NODE}/set_parameters',
            callback_group=self.cb)
        self.takeoff_cli = self.create_client(
            Takeoff, f'/{DRONE}/takeoff', callback_group=self.cb)
        self.upload_cli = self.create_client(
            UploadTrajectory, f'/{DRONE}/upload_trajectory',
            callback_group=self.cb)
        self.start_cli = self.create_client(
            StartTrajectory, f'/{DRONE}/start_trajectory',
            callback_group=self.cb)
        self.goto_cli = self.create_client(
            GoTo, f'/{DRONE}/go_to', callback_group=self.cb)

    def _odom_cb(self, msg):
        if not self.odom_received.is_set():
            self.odom_received.set()
        if not self.recording:
            return
        t_sim = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        with self.odom_lock:
            if self.t0_sim is None:
                self.t0_sim = t_sim
            self.odom_log.append(
                (t_sim - self.t0_sim, p.x, p.y, p.z, v.x, v.y))

    def wait_services(self, timeout=30.0):
        deadline = time.time() + timeout
        for name, cli in [
            ('set_parameters', self.set_params_cli),
            ('takeoff', self.takeoff_cli),
            ('upload_trajectory', self.upload_cli),
            ('start_trajectory', self.start_cli),
            ('go_to', self.goto_cli),
        ]:
            remaining = deadline - time.time()
            if remaining <= 0:
                raise RuntimeError(f'Timeout waiting for service {name}')
            if not cli.wait_for_service(timeout_sec=remaining):
                raise RuntimeError(f'Service {name} not available')
            self.get_logger().info(f'Service ready: {name}')

        self.get_logger().info('Waiting for first odom message...')
        if not self.odom_received.wait(timeout=15.0):
            raise RuntimeError('No odom received from /cf1/odom')
        self.get_logger().info('Odom received.')

    def _call(self, cli, req, timeout=10.0, name=''):
        """Async call; wait via done-callback so the bg executor handles it
        (never spin_until_future_complete from here — would double-spin and
        starve the odom subscription)."""
        done = Event()
        fut = cli.call_async(req)
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=timeout):
            raise RuntimeError(f'service call timed out: {name}')
        return fut.result()

    def set_gain(self, name, value):
        p = Parameter()
        p.name = name
        p.value = ParameterValue()
        p.value.type = ParameterType.PARAMETER_DOUBLE
        p.value.double_value = float(value)
        req = SetParameters.Request()
        req.parameters = [p]
        result = self._call(self.set_params_cli, req, timeout=5.0,
                            name=f'set_parameters({name})')
        ok = result.results[0].successful
        self.get_logger().info(f'Set {name} = {value}: {"OK" if ok else "FAIL"}')
        if not ok:
            raise RuntimeError(f'set_parameters({name}) returned not successful')

    def set_gains(self, gains):
        for k, v in gains.items():
            self.set_gain(k, v)

    def takeoff(self, height=HOVER_Z, duration=3.0):
        req = Takeoff.Request()
        req.height = float(height)
        req.duration = _to_duration_msg(duration)
        self._call(self.takeoff_cli, req, timeout=5.0, name='takeoff')
        self.get_logger().info(f'Takeoff requested. Waiting {duration + 1.0}s...')
        time.sleep(duration + 1.0)

    def goto_xy(self, x, y, duration=2.0):
        """Reposition to a known start pose before a trajectory run."""
        req = GoTo.Request()
        req.goal.x = float(x)
        req.goal.y = float(y)
        req.goal.z = float(HOVER_Z)
        req.yaw = 0.0
        req.relative = False
        req.duration = _to_duration_msg(duration)
        self._call(self.goto_cli, req, timeout=5.0, name='go_to')
        time.sleep(duration + 1.0)

    def upload_and_start(self, pieces, traj_id=1):
        req_u = UploadTrajectory.Request()
        req_u.trajectory_id = traj_id
        req_u.piece_offset = 0
        req_u.pieces = pieces
        self._call(self.upload_cli, req_u, timeout=10.0, name='upload_trajectory')
        self.get_logger().info(f'Uploaded trajectory {traj_id} ({len(pieces)} pieces)')

        # Begin recording NOW (just before start_trajectory so we capture t=0).
        with self.odom_lock:
            self.odom_log.clear()
        self.t0_sim = None  # will be set to first odom's header.stamp
        self.recording = True

        req_s = StartTrajectory.Request()
        req_s.trajectory_id = traj_id
        req_s.timescale = 1.0
        req_s.reversed = False
        req_s.relative = False
        self._call(self.start_cli, req_s, timeout=5.0, name='start_trajectory')

    def stop_recording(self):
        self.recording = False


# ════════════════════ Main ════════════════════

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--kp-xy', type=float, default=3.0)
    ap.add_argument('--kd-xy', type=float, default=0.3)
    ap.add_argument('--kp-z', type=float, default=2.0)
    ap.add_argument('--kd-z', type=float, default=0.0)
    ap.add_argument('--max-vel-xy', type=float, default=1.5)
    ap.add_argument('--speed', type=float, default=1.0,
                    help='Trajectory speed in m/s')
    ap.add_argument('--dt', type=float, default=0.1,
                    help='Waypoint cadence in seconds (10 Hz = 0.1)')
    ap.add_argument('--tag', type=str, default='run',
                    help='Tag for output filenames')
    ap.add_argument('--outdir', type=str,
                    default=os.path.expanduser('~/cs2_ws/exp1/tuning'),
                    help='Directory to save CSV + plots')
    ap.add_argument('--no-plot', action='store_true')
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    rclpy.init()
    node = TuningNode(args)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.wait_services()

        # 1. Set gains via ROS2 param API.
        node.set_gains({
            'kp_xy': args.kp_xy,
            'kd_xy': args.kd_xy,
            'kp_z': args.kp_z,
            'kd_z': args.kd_z,
            'max_vel_xy': args.max_vel_xy,
        })

        # 2. Takeoff to hover height.
        node.takeoff(height=HOVER_Z, duration=3.0)

        # 3. Reposition to trajectory start (separate from spawn to keep
        #    tracking errors independent of spawn exactness).
        start_xy = (-1.5, 0.0)
        node.goto_xy(start_xy[0], start_xy[1], duration=2.0)

        # 4. Build synthetic trajectory and fit cubic Hermite pieces.
        xs, ys, ts = make_synthetic_trajectory(
            speed=args.speed, dt=args.dt, start_xy=start_xy)
        pieces = fit_hermite_pieces(xs, ys, ts, z_const=HOVER_Z)

        # Keep a raw-coeff copy for local commanded evaluation.
        pieces_raw = []
        for piece in pieces:
            dur = piece.duration.sec + piece.duration.nanosec / 1e9
            pieces_raw.append((list(piece.poly_x), list(piece.poly_y), dur))
        total_dur = sum(d for _, _, d in pieces_raw)

        node.get_logger().info(
            f'Trajectory: {len(pieces)} pieces, total {total_dur:.2f} s, '
            f'speed {args.speed} m/s.')

        # 5. Upload + start. Record odom throughout.
        node.upload_and_start(pieces, traj_id=1)

        # 6. Wait for completion + a short tail. Use sim time via odom log so
        #    a slow Gazebo RTF doesn't truncate the recording in wall time.
        deadline_sim = total_dur + 1.5
        node.get_logger().info(
            f'Recording until sim-t >= {deadline_sim:.1f}s '
            f'(wall timeout {total_dur * 2 + 5.0:.0f}s)...')
        wall_timeout = time.time() + total_dur * 2 + 5.0
        while time.time() < wall_timeout:
            time.sleep(0.2)
            with node.odom_lock:
                if node.odom_log and node.odom_log[-1][0] >= deadline_sim:
                    break
        node.stop_recording()

        # 7. Compute tracking error vs. commanded polynomial.
        with node.odom_lock:
            log = list(node.odom_log)

        rows = []
        errs = []
        errs_straight = []
        for (t, x, y, z, vx, vy) in log:
            cx, cy, cvx, cvy = eval_hermite_at(pieces_raw, t)
            ex = x - cx
            ey = y - cy
            err = math.sqrt(ex * ex + ey * ey)
            rows.append((t, cx, cy, x, y, z, cvx, cvy, vx, vy, err))
            errs.append(err)
            # Steady-state window: leg B (straight +X at constant 1 m/s).
            # Trajectory: A ramp-in 0-0.5s, B straight 0.5-2.5s.
            if 0.6 <= t <= 2.4:
                errs_straight.append(err)

        errs_np = np.array(errs) if errs else np.array([0.0])
        errs_straight_np = (np.array(errs_straight)
                            if errs_straight else np.array([0.0]))
        stats = {
            'max_err_m': float(errs_np.max()),
            'rms_err_m': float(np.sqrt((errs_np ** 2).mean())),
            'mean_err_m': float(errs_np.mean()),
            'steady_state_err_m_leg1': float(errs_straight_np.mean()),
            'n_samples': len(rows),
        }

        node.get_logger().info(
            f'\n=== Tracking error (tag={args.tag}, kp_xy={args.kp_xy}, '
            f'kd_xy={args.kd_xy}, max_vel_xy={args.max_vel_xy}) ===\n'
            f"  max:                {stats['max_err_m']:.4f} m\n"
            f"  RMS:                {stats['rms_err_m']:.4f} m\n"
            f"  mean:               {stats['mean_err_m']:.4f} m\n"
            f"  leg1 steady-state:  {stats['steady_state_err_m_leg1']:.4f} m\n"
            f"  samples:            {stats['n_samples']}")

        # 8. Save CSV.
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        base = f'{stamp}_{args.tag}'
        csv_path = os.path.join(args.outdir, f'{base}.csv')
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'cmd_x', 'cmd_y', 'x', 'y', 'z',
                        'cmd_vx', 'cmd_vy', 'vx', 'vy', 'err'])
            w.writerows(rows)
        node.get_logger().info(f'Saved CSV: {csv_path}')

        # Save summary one-liner for sweep aggregation.
        summary_path = os.path.join(args.outdir, 'summary.csv')
        write_header = not os.path.exists(summary_path)
        with open(summary_path, 'a', newline='') as f:
            w = csv.writer(f)
            if write_header:
                w.writerow(['timestamp', 'tag', 'kp_xy', 'kd_xy', 'kp_z',
                            'kd_z', 'max_vel_xy', 'speed',
                            'max_err_m', 'rms_err_m', 'mean_err_m',
                            'steady_state_err_m_leg1'])
            w.writerow([stamp, args.tag, args.kp_xy, args.kd_xy, args.kp_z,
                        args.kd_z, args.max_vel_xy, args.speed,
                        stats['max_err_m'], stats['rms_err_m'],
                        stats['mean_err_m'],
                        stats['steady_state_err_m_leg1']])

        # 9. Plot.
        if not args.no_plot:
            try:
                import matplotlib
                matplotlib.use('Agg')
                import matplotlib.pyplot as plt

                arr = np.array(rows)
                fig, (ax_xy, ax_err) = plt.subplots(1, 2, figsize=(14, 6))
                ax_xy.plot(arr[:, 1], arr[:, 2], 'b--', label='commanded',
                           linewidth=1)
                ax_xy.plot(arr[:, 3], arr[:, 4], 'r-', label='actual',
                           linewidth=1)
                ax_xy.set_xlabel('x [m]')
                ax_xy.set_ylabel('y [m]')
                ax_xy.set_aspect('equal')
                ax_xy.legend()
                ax_xy.set_title(f'XY trace: {args.tag}')
                ax_xy.grid(True, alpha=0.3)

                ax_err.plot(arr[:, 0], arr[:, 10], 'k-', linewidth=1)
                ax_err.set_xlabel('t [s]')
                ax_err.set_ylabel('position error [m]')
                ax_err.axhline(0.02, color='g', ls=':',
                               label='stretch (2 cm)')
                ax_err.axhline(0.05, color='y', ls=':',
                               label='minimum (5 cm)')
                ax_err.set_title(
                    f"max={stats['max_err_m']*100:.2f} cm, "
                    f"RMS={stats['rms_err_m']*100:.2f} cm, "
                    f"leg1 SS={stats['steady_state_err_m_leg1']*100:.2f} cm")
                ax_err.legend()
                ax_err.grid(True, alpha=0.3)

                plt.suptitle(
                    f'kp_xy={args.kp_xy}, kd_xy={args.kd_xy}, '
                    f'max_vel_xy={args.max_vel_xy}, speed={args.speed} m/s')
                plt.tight_layout()
                png_path = os.path.join(args.outdir, f'{base}.png')
                plt.savefig(png_path, dpi=120)
                node.get_logger().info(f'Saved plot: {png_path}')
            except Exception as e:
                node.get_logger().warn(f'Plot failed: {e}')

        # 10. Return to start for next run.
        node.goto_xy(start_xy[0], start_xy[1], duration=2.0)

    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
