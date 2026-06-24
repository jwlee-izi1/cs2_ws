#!/usr/bin/env python3
"""Play a 2D RL trajectory on the Gazebo Crazyflie via upload_trajectory.

Workflow:
  1. Load NPZ with keys: positions (Nx2), timestamps (N,), optionally
     goal_position, obstacles, arena_bounds, outcome, policy_id.
     (Or use --synthetic for an internal test trajectory.)
  2. Apply time-scale so the policy's 1.0 m/s cadence plays at sim-safe
     speed. Default 3.333x scales 1 m/s -> 0.3 m/s. After the sim run,
     ffmpeg with setpts=0.3*PTS plays back at the policy's native speed.
  3. Load controller gains from config/controller_gains.yaml and apply
     via ROS2 param API on /control_services_cf1 (runtime-tunable).
  4. Takeoff, go to trajectory start, fit cubic Hermite, upload, start,
     log odom (sim time), report tracking error, save CSV + plot.

Yaw is locked at 0 throughout (drone faces +X), per Phase B decisions.

Prereq: `ros2 launch rl_demo rl_demo.launch.py` is running.
"""

import argparse
import csv
import math
import os
import time
from datetime import datetime
from threading import Event, Lock

import numpy as np
import yaml

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

from ament_index_python.packages import get_package_share_directory


DRONE = 'cf1'
CONTROL_NODE = f'control_services_{DRONE}'
HOVER_Z = 0.5


# ════════════════════ Cubic Hermite (same as tune_tracking.py) ═══════════════

def fit_hermite_pieces(xs, ys, ts, z_const=HOVER_Z):
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    ts = np.asarray(ts, dtype=float)
    assert len(ts) >= 2
    vxs = np.gradient(xs, ts)
    vys = np.gradient(ys, ts)
    pieces = []
    for i in range(len(ts) - 1):
        dt = ts[i + 1] - ts[i]
        if dt <= 0:
            continue

        def cubic(p0, p1, v0, v1, T):
            return [
                float(p0),
                float(v0),
                float(3.0 * (p1 - p0) / (T * T) - (2.0 * v0 + v1) / T),
                float(-2.0 * (p1 - p0) / (T ** 3) + (v0 + v1) / (T * T)),
            ]
        piece = TrajectoryPolynomialPiece()
        piece.poly_x = cubic(xs[i], xs[i + 1], vxs[i], vxs[i + 1], dt)
        piece.poly_y = cubic(ys[i], ys[i + 1], vys[i], vys[i + 1], dt)
        piece.poly_z = [float(z_const), 0.0, 0.0, 0.0]
        piece.poly_yaw = [0.0, 0.0, 0.0, 0.0]
        piece.duration = _to_duration_msg(dt)
        pieces.append(piece)
    return pieces


def _to_duration_msg(sec):
    d = DurationMsg()
    d.sec = int(sec)
    d.nanosec = int((sec - int(sec)) * 1e9)
    return d


def eval_hermite_at(raw, t):
    total = 0.0
    for px, py, dur in raw:
        if t <= total + dur:
            u = t - total
            x = px[0] + px[1] * u + px[2] * u * u + px[3] * u ** 3
            y = py[0] + py[1] * u + py[2] * u * u + py[3] * u ** 3
            return x, y
        total += dur
    px, py, dur = raw[-1]
    u = dur
    return (px[0] + px[1] * u + px[2] * u * u + px[3] * u ** 3,
            py[0] + py[1] * u + py[2] * u * u + py[3] * u ** 3)


# ════════════════════ Trajectory sources ════════════════════

def load_npz(path):
    data = np.load(path, allow_pickle=True)
    positions = np.asarray(data['positions'], dtype=float)
    if positions.ndim != 2 or positions.shape[1] != 2:
        raise ValueError(f'positions must be Nx2, got {positions.shape}')
    timestamps = np.asarray(data['timestamps'], dtype=float)
    if timestamps.shape != (positions.shape[0],):
        raise ValueError('timestamps shape mismatch with positions')
    return positions[:, 0], positions[:, 1], timestamps


def synthetic_trajectory(speed=1.0, dt=0.1, start_xy=(-1.5, 0.0)):
    """Same shape as tune_tracking's synthetic test — smooth ramp-in,
    straight leg, rounded corner, straight, ramp-out, hold, ramp-in -X, etc.
    Use when no NPZ is available yet."""
    x, y = start_xy
    xs, ys, ts = [x], [y], [0.0]

    def append(xn, yn):
        xs.append(xn); ys.append(yn); ts.append(ts[-1] + dt)

    def ramp(dx, dy, v0, v1, dur):
        n = max(1, int(round(dur / dt)))
        for k in range(1, n + 1):
            s = k / n
            v = v0 + s * (v1 - v0)
            vp = v0 + ((k - 1) / n) * (v1 - v0)
            step = 0.5 * (vp + v) * dt
            append(xs[-1] + dx * step, ys[-1] + dy * step)

    def straight(dx, dy, L, v):
        n = max(1, int(round((L / v) / dt)))
        for _ in range(n):
            append(xs[-1] + dx * v * dt, ys[-1] + dy * v * dt)

    ramp(1, 0, 0, speed, 0.5)
    straight(1, 0, 2.0, speed)
    R = 0.4
    arc_dur = (0.5 * math.pi * R) / speed
    n_arc = max(4, int(round(arc_dur / dt)))
    cx = xs[-1]; cy = ys[-1] + R
    theta0 = -math.pi / 2
    for k in range(1, n_arc + 1):
        s = k / n_arc
        theta = theta0 + s * (math.pi / 2)
        append(cx + R * math.cos(theta), cy + R * math.sin(theta))
    straight(0, 1, 1.0, speed)
    ramp(0, 1, speed, 0, 0.5)
    for _ in range(int(round(1.0 / dt))):
        append(xs[-1], ys[-1])
    ramp(-1, 0, 0, speed, 0.5)
    straight(-1, 0, 1.5, speed)
    ramp(-1, 0, speed, 0, 0.5)
    return np.array(xs), np.array(ys), np.array(ts)


# ════════════════════ ROS node ════════════════════

class PlayerNode(Node):
    def __init__(self):
        super().__init__('waypoint_player')
        self.cb = ReentrantCallbackGroup()
        self.odom_lock = Lock()
        self.odom_log = []
        self.odom_received = Event()
        self.recording = False
        self.t0_sim = None
        self.create_subscription(
            Odometry, f'/{DRONE}/odom', self._odom_cb, 50,
            callback_group=self.cb)
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
            if not cli.wait_for_service(timeout_sec=max(0.1, deadline - time.time())):
                raise RuntimeError(f'Service {name} not available')
        self.get_logger().info('All services ready; waiting for odom...')
        if not self.odom_received.wait(timeout=15.0):
            raise RuntimeError('No odom on /cf1/odom')

    def _call(self, cli, req, timeout=10.0, name=''):
        done = Event()
        fut = cli.call_async(req)
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=timeout):
            raise RuntimeError(f'Service call timed out: {name}')
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
        if not result.results[0].successful:
            raise RuntimeError(f'set_parameters({name}) failed')

    def takeoff(self, height=HOVER_Z, duration=3.0):
        req = Takeoff.Request()
        req.height = float(height)
        req.duration = _to_duration_msg(duration)
        self._call(self.takeoff_cli, req, timeout=5.0, name='takeoff')
        time.sleep(duration + 1.0)

    def goto_xy(self, x, y, duration=2.0):
        req = GoTo.Request()
        req.goal.x = float(x)
        req.goal.y = float(y)
        req.goal.z = float(HOVER_Z)
        req.yaw = 0.0
        req.relative = False
        req.duration = _to_duration_msg(duration)
        self._call(self.goto_cli, req, timeout=5.0, name='go_to')
        time.sleep(duration + 1.0)

    def play(self, pieces, total_dur):
        req_u = UploadTrajectory.Request()
        req_u.trajectory_id = 1
        req_u.piece_offset = 0
        req_u.pieces = pieces
        self._call(self.upload_cli, req_u, timeout=10.0, name='upload')
        with self.odom_lock:
            self.odom_log.clear()
        self.t0_sim = None
        self.recording = True
        req_s = StartTrajectory.Request()
        req_s.trajectory_id = 1
        req_s.timescale = 1.0
        req_s.reversed = False
        req_s.relative = False
        self._call(self.start_cli, req_s, timeout=5.0, name='start')

        deadline_sim = total_dur + 1.5
        wall_timeout = time.time() + total_dur * 2 + 10.0
        while time.time() < wall_timeout:
            time.sleep(0.2)
            with self.odom_lock:
                if self.odom_log and self.odom_log[-1][0] >= deadline_sim:
                    break
        self.recording = False


# ════════════════════ Main ════════════════════

def load_gains_yaml():
    path = os.path.join(get_package_share_directory('rl_demo'),
                        'config', 'controller_gains.yaml')
    with open(path, 'r') as f:
        cfg = yaml.safe_load(f)
    return cfg.get(f'/{CONTROL_NODE}', {}).get('ros__parameters', {})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--npz', type=str, default=None,
                    help='Path to 2D RL trajectory NPZ')
    ap.add_argument('--synthetic', action='store_true',
                    help='Use internal synthetic trajectory instead of NPZ')
    ap.add_argument('--time-scale', type=float, default=4.8,
                    help='Multiply NPZ timestamps by this (1.0->sim_speed). '
                         'Default 4.8 maps 1.414 m/s diagonal-speed policy '
                         'to ~0.3 m/s sim (safe envelope).')
    ap.add_argument('--wait', action='store_true',
                    help='Pause after repositioning at trajectory start; '
                         'wait for Enter before uploading + playing.')
    ap.add_argument('--outdir', type=str,
                    default=os.path.expanduser('~/cs2_ws/exp1/playback'))
    ap.add_argument('--tag', type=str, default='playback')
    args = ap.parse_args()

    if not args.npz and not args.synthetic:
        raise SystemExit('Provide --npz PATH or --synthetic')
    os.makedirs(args.outdir, exist_ok=True)

    if args.synthetic:
        xs, ys, ts = synthetic_trajectory(
            speed=1.0 / args.time_scale, dt=0.1, start_xy=(-1.5, 0.0))
        source = 'synthetic'
    else:
        xs, ys, ts = load_npz(args.npz)
        ts = ts * args.time_scale
        source = os.path.basename(args.npz)

    rclpy.init()
    node = PlayerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    import threading
    threading.Thread(target=executor.spin, daemon=True).start()

    try:
        node.wait_services()
        gains = load_gains_yaml()
        node.get_logger().info(f'Applying gains: {gains}')
        for k, v in gains.items():
            if isinstance(v, (int, float)):
                node.set_gain(k, v)

        node.takeoff(HOVER_Z, duration=3.0)
        # If drone was spawned at the trajectory start (via prepare_arena.py's
        # per-run YAML), takeoff already leaves us there. Still issue a short
        # go_to to converge XY precisely to positions[0] before upload.
        node.goto_xy(float(xs[0]), float(ys[0]), duration=2.0)

        if args.wait:
            node.get_logger().info(
                'Ready at trajectory start — adjust Gazebo GUI camera, '
                'then press Enter to play...')
            input()

        pieces = fit_hermite_pieces(xs, ys, ts, z_const=HOVER_Z)
        raw = []
        for p in pieces:
            dur = p.duration.sec + p.duration.nanosec / 1e9
            raw.append((list(p.poly_x), list(p.poly_y), dur))
        total_dur = sum(d for _, _, d in raw)
        node.get_logger().info(
            f'Source={source}, pieces={len(pieces)}, total_dur={total_dur:.2f}s, '
            f'time_scale={args.time_scale}')

        node.play(pieces, total_dur)

        # Analysis
        with node.odom_lock:
            log = list(node.odom_log)
        rows, errs = [], []
        for (t, x, y, z, vx, vy) in log:
            cx, cy = eval_hermite_at(raw, t)
            err = math.hypot(x - cx, y - cy)
            rows.append((t, cx, cy, x, y, z, vx, vy, err))
            errs.append(err)
        stats = {
            'max': float(np.max(errs) if errs else 0.0),
            'rms': float(np.sqrt(np.mean(np.square(errs))) if errs else 0.0),
            'mean': float(np.mean(errs) if errs else 0.0),
            'n': len(rows),
        }
        node.get_logger().info(
            f"Tracking: max={stats['max']:.3f}m RMS={stats['rms']:.3f}m "
            f"mean={stats['mean']:.3f}m n={stats['n']}")

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        base = f'{stamp}_{args.tag}'
        csv_path = os.path.join(args.outdir, f'{base}.csv')
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'cmd_x', 'cmd_y', 'x', 'y', 'z', 'vx', 'vy', 'err'])
            w.writerows(rows)
        node.get_logger().info(f'CSV: {csv_path}')

        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            arr = np.array(rows)
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
            ax1.plot(arr[:, 1], arr[:, 2], 'b--', label='commanded')
            ax1.plot(arr[:, 3], arr[:, 4], 'r-', label='actual')
            ax1.set_aspect('equal')
            ax1.legend(); ax1.grid(True, alpha=0.3)
            ax1.set_xlabel('x [m]'); ax1.set_ylabel('y [m]')
            ax2.plot(arr[:, 0], arr[:, 8])
            ax2.set_xlabel('t [s]'); ax2.set_ylabel('err [m]')
            ax2.grid(True, alpha=0.3)
            plt.suptitle(
                f"Playback {args.tag} | max={stats['max']*100:.1f}cm "
                f"RMS={stats['rms']*100:.1f}cm")
            plt.tight_layout()
            plt.savefig(os.path.join(args.outdir, f'{base}.png'), dpi=120)
        except Exception as e:
            node.get_logger().warn(f'Plot failed: {e}')

    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
