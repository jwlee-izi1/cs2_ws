#!/usr/bin/env python3
"""Test payload-coupled hover for 4 Crazyflies.

Calls takeoff + go_to services on cf1..cf4, records odom z traces,
and produces a matplotlib plot at the end.

Usage:
  # Symmetric hover (Step 4):
  python3 test_payload_hover.py --targets 1.25 1.25 1.25 1.25

  # Asymmetric hover (Step 6):
  python3 test_payload_hover.py --targets 1.00 1.10 0.90 1.10

  # Custom durations:
  python3 test_payload_hover.py --targets 1.25 1.25 1.25 1.25 \
      --hold-time 1.0 --goto-duration 5.0 --record-duration 60.0

Prerequisites:
  - Gazebo payload world running (paused or unpaused)
  - ros_gz_bridge running
  - control_services nodes running for cf1..cf4
  - Unpause Gazebo before or after running this script
"""

import argparse
import csv
import os
import sys
import time
from datetime import datetime
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from builtin_interfaces.msg import Duration as DurationMsg
from crazyflie_interfaces.srv import GoTo, Takeoff
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty


DRONES = ['cf1', 'cf2', 'cf3', 'cf4']


class OdomRecorder(Node):
    """Subscribe to odom for all drones, record z traces with wall-clock timestamps."""

    def __init__(self):
        super().__init__('odom_recorder')
        self.lock = Lock()
        # {drone_name: [(wall_time, x, y, z, vx, vy, vz), ...]}
        self.traces = {name: [] for name in DRONES}
        self.latest_pose = {name: None for name in DRONES}
        self.t0 = time.time()

        cb_group = ReentrantCallbackGroup()
        for name in DRONES:
            self.create_subscription(
                Odometry, f'/{name}/odom',
                lambda msg, n=name: self._odom_cb(n, msg),
                10,
                callback_group=cb_group,
            )
        self.get_logger().info('OdomRecorder ready')

    def _odom_cb(self, name, msg):
        t = time.time() - self.t0
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        with self.lock:
            self.traces[name].append((t, p.x, p.y, p.z, v.x, v.y, v.z))
            self.latest_pose[name] = (p.x, p.y, p.z)

    def get_latest_positions(self):
        with self.lock:
            return dict(self.latest_pose)


def make_duration(sec):
    s = int(sec)
    ns = int((sec - s) * 1e9)
    return DurationMsg(sec=s, nanosec=ns)


def call_service(node, srv_type, topic, request, timeout=5.0):
    client = node.create_client(srv_type, topic)
    if not client.wait_for_service(timeout_sec=timeout):
        node.get_logger().error(f'Service {topic} not available')
        return None
    future = client.call_async(request)
    # Wait for result without calling spin (node is already spinning in background)
    t0 = time.time()
    while not future.done() and (time.time() - t0) < timeout:
        time.sleep(0.01)
    if future.done():
        return future.result()
    node.get_logger().error(f'Service {topic} timed out')
    return None


def main():
    parser = argparse.ArgumentParser(description='Test payload-coupled hover')
    parser.add_argument('--targets', nargs=4, type=float, required=True,
                        metavar=('Z1', 'Z2', 'Z3', 'Z4'),
                        help='Target z heights for cf1..cf4')
    parser.add_argument('--hold-time', type=float, default=1.0,
                        help='Seconds to hold current position before sending targets (default: 1.0)')
    parser.add_argument('--goto-duration', type=float, default=5.0,
                        help='Duration for go_to trajectory (default: 5.0)')
    parser.add_argument('--record-duration', type=float, default=60.0,
                        help='Total recording duration in seconds (default: 60.0)')
    parser.add_argument('--output-dir', type=str, default='/tmp/payload_hover',
                        help='Directory for CSV and plot output (default: /tmp/payload_hover)')
    parser.add_argument('--no-plot', action='store_true',
                        help='Skip matplotlib plot')
    args = parser.parse_args()

    rclpy.init()
    recorder = OdomRecorder()

    # Spin in background to collect odom
    from threading import Thread
    spin_thread = Thread(target=lambda: rclpy.spin(recorder), daemon=True)
    spin_thread.start()

    # Wait for odom from all drones
    print('Waiting for odom from all drones...')
    while True:
        positions = recorder.get_latest_positions()
        if all(v is not None for v in positions.values()):
            break
        time.sleep(0.1)
    print('All drones reporting odom.')

    # Phase 1: Hold current position (spin up motors at zero error)
    print(f'Holding current positions for {args.hold_time}s...')
    for name in DRONES:
        pos = positions[name]
        req = GoTo.Request()
        req.group_mask = 0
        req.relative = False
        req.goal = Point(x=pos[0], y=pos[1], z=pos[2])
        req.yaw = 0.0
        req.duration = make_duration(0.5)
        call_service(recorder, GoTo, f'/{name}/go_to', req)
    time.sleep(args.hold_time)

    # Phase 2: Send real targets
    target_z = dict(zip(DRONES, args.targets))
    print(f'Sending targets: {target_z}')
    for name in DRONES:
        pos = positions[name]
        req = GoTo.Request()
        req.group_mask = 0
        req.relative = False
        req.goal = Point(x=pos[0], y=pos[1], z=target_z[name])
        req.yaw = 0.0
        req.duration = make_duration(args.goto_duration)
        call_service(recorder, GoTo, f'/{name}/go_to', req)

    # Phase 3: Record
    print(f'Recording for {args.record_duration}s...')
    t_start = time.time()
    try:
        while time.time() - t_start < args.record_duration:
            time.sleep(0.5)
            # Print live z values
            positions = recorder.get_latest_positions()
            zs = [f'{name}: {positions[name][2]:.3f}' if positions[name] else f'{name}: ---'
                  for name in DRONES]
            elapsed = time.time() - t_start
            print(f'\r  [{elapsed:5.1f}s] ' + '  '.join(zs), end='', flush=True)
    except KeyboardInterrupt:
        print('\nInterrupted.')

    print('\nRecording complete.')

    # Save CSVs
    os.makedirs(args.output_dir, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    for name in DRONES:
        csv_path = os.path.join(args.output_dir, f'{name}_odom_{timestamp}.csv')
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['wall_time_s', 'x', 'y', 'z', 'vx', 'vy', 'vz'])
            writer.writerows(recorder.traces[name])
        print(f'  Saved {csv_path} ({len(recorder.traces[name])} samples)')

    # Plot
    if not args.no_plot:
        try:
            import matplotlib
            matplotlib.use('TkAgg')
            import matplotlib.pyplot as plt

            fig, ax = plt.subplots(figsize=(12, 6))
            for i, name in enumerate(DRONES):
                trace = recorder.traces[name]
                if not trace:
                    continue
                ts = [r[0] for r in trace]
                zs = [r[3] for r in trace]
                ax.plot(ts, zs, label=name)
                # Draw target line
                ax.axhline(y=args.targets[i], color=f'C{i}', linestyle='--',
                           alpha=0.5, linewidth=0.8)

            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Z position (m)')
            ax.set_title('Payload-Coupled Hover: Z Traces')
            ax.legend()
            ax.grid(True, alpha=0.3)

            plot_path = os.path.join(args.output_dir, f'z_traces_{timestamp}.png')
            fig.savefig(plot_path, dpi=150, bbox_inches='tight')
            print(f'  Saved plot: {plot_path}')
            plt.show()
        except ImportError:
            print('  matplotlib not available, skipping plot')
        except Exception as e:
            print(f'  Plot error: {e}')

    recorder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
