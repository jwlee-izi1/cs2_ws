#!/usr/bin/env python3
"""BVC head-on position-exchange test driver (Test 2).

cf1 and cf2 start facing each other on the x-axis (cf1 ~(-1,0), cf2 ~(+1,0)). We take
both off to a common altitude, then stream cmd_position setpoints that command them to
SWAP positions (cf1 -> cf2's start, cf2 -> cf1's start) -> a direct head-on course.

With BVC on (peer_broadcast_hz>0 in the yaml) the onboard Buffered Voronoi Cell must bend
the setpoints sideways (sidestepGoal: cf1 veers -y, cf2 veers +y) so the two never get
closer than the cell geometry allows (horizontal center separation >= 2*0.3 = 0.6 m).

Records, at 50 Hz: both drone positions, the commanded (straight-line) target, horizontal
& 3D separation. Prints min-separation and per-drone max |y| (the sidestep signature) so
"paths happened not to overlap" can be told apart from "BVC actively pushed them apart".

Run AFTER sitl_2drone_headon.sh + the crazyswarm2 server are up and both /cfN/odom live.
Usage:
  python3 scripts/bvc_headon_test.py --label bvc_on  --out /tmp/bvc/bvc_on.csv
  python3 scripts/bvc_headon_test.py --label bvc_off --out /tmp/bvc/bvc_off.csv --exchange 12
"""
import argparse
import csv
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from crazyflie_interfaces.msg import Position
from crazyflie_interfaces.srv import Takeoff, Land
from builtin_interfaces.msg import Duration


class HeadOnTest(Node):
    def __init__(self, args):
        super().__init__('bvc_headon_test')
        self.args = args
        self.z = args.z
        self.pos = {'cf1': None, 'cf2': None}          # latest (x,y,z)
        self.start = {'cf1': None, 'cf2': None}        # recorded pre-takeoff
        self.target = {'cf1': None, 'cf2': None}       # current commanded (x,y)
        self.rows = []
        self.t_stream0 = None                          # wall time streaming began
        self.lock = threading.Lock()

        self.create_subscription(Odometry, '/cf1/odom', lambda m: self._odom('cf1', m), 10)
        self.create_subscription(Odometry, '/cf2/odom', lambda m: self._odom('cf2', m), 10)
        self.pub = {
            'cf1': self.create_publisher(Position, '/cf1/cmd_position', 10),
            'cf2': self.create_publisher(Position, '/cf2/cmd_position', 10),
        }
        self.takeoff_cli = {n: self.create_client(Takeoff, f'/{n}/takeoff') for n in ('cf1', 'cf2')}
        self.land_cli = {n: self.create_client(Land, f'/{n}/land') for n in ('cf1', 'cf2')}

    def _odom(self, name, msg):
        p = msg.pose.pose.position
        with self.lock:
            self.pos[name] = (p.x, p.y, p.z)

    # ---- 50 Hz stream + log timer ----
    def _tick(self):
        with self.lock:
            p1, p2 = self.pos['cf1'], self.pos['cf2']
        if p1 is None or p2 is None or self.t_stream0 is None:
            return
        t = time.monotonic() - self.t_stream0
        settle, exch = self.args.settle, self.args.exchange
        if t < settle:
            phase = 'settle'
            self.target['cf1'] = (self.start['cf1'][0], self.start['cf1'][1])
            self.target['cf2'] = (self.start['cf2'][0], self.start['cf2'][1])
        else:
            phase = 'exchange' if t < settle + exch else 'hold'
            # swap targets: cf1 -> cf2 start, cf2 -> cf1 start
            self.target['cf1'] = (self.start['cf2'][0], self.start['cf2'][1])
            self.target['cf2'] = (self.start['cf1'][0], self.start['cf1'][1])
        for n in ('cf1', 'cf2'):
            tx, ty = self.target[n]
            m = Position()
            m.header.stamp = self.get_clock().now().to_msg()
            m.x, m.y, m.z, m.yaw = float(tx), float(ty), float(self.z), 0.0
            self.pub[n].publish(m)
        dx, dy, dz = p1[0]-p2[0], p1[1]-p2[1], p1[2]-p2[2]
        self.rows.append({
            't': round(t, 3), 'phase': phase,
            'x1': round(p1[0],4), 'y1': round(p1[1],4), 'z1': round(p1[2],4),
            'x2': round(p2[0],4), 'y2': round(p2[1],4), 'z2': round(p2[2],4),
            'tgt1x': round(self.target['cf1'][0],3), 'tgt2x': round(self.target['cf2'][0],3),
            'sep_h': round(math.hypot(dx, dy),4), 'sep_3d': round(math.sqrt(dx*dx+dy*dy+dz*dz),4),
        })

    # ---- service helpers ----
    def _wait_srv(self, cli, name):
        if not cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'service {name} not available')

    def takeoff_both(self):
        for n in ('cf1', 'cf2'):
            self._wait_srv(self.takeoff_cli[n], f'/{n}/takeoff')
        futs = []
        for n in ('cf1', 'cf2'):
            req = Takeoff.Request()
            req.group_mask = 0
            req.height = float(self.z)
            req.duration = Duration(sec=3, nanosec=0)
            futs.append(self.takeoff_cli[n].call_async(req))
        self._await(futs, 5.0)

    def land_both(self):
        futs = []
        for n in ('cf1', 'cf2'):
            if not self.land_cli[n].wait_for_service(timeout_sec=3.0):
                continue
            req = Land.Request()
            req.group_mask = 0
            req.height = 0.05
            req.duration = Duration(sec=3, nanosec=0)
            futs.append(self.land_cli[n].call_async(req))
        self._await(futs, 4.0)

    @staticmethod
    def _await(futs, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if all(f.done() for f in futs):
                return
            time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', default='run')
    ap.add_argument('--out', default='/tmp/bvc/run.csv')
    ap.add_argument('--z', type=float, default=1.0)
    ap.add_argument('--settle', type=float, default=3.0, help='hold-at-start seconds before swap')
    ap.add_argument('--exchange', type=float, default=18.0, help='swap streaming seconds')
    ap.add_argument('--hold', type=float, default=2.0, help='hold-at-target seconds after swap')
    args = ap.parse_args()

    rclpy.init()
    node = HeadOnTest(args)
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()

    log = node.get_logger()
    # 1. wait for both odoms
    log.info('waiting for /cf1/odom and /cf2/odom ...')
    t0 = time.monotonic()
    while node.pos['cf1'] is None or node.pos['cf2'] is None:
        if time.monotonic() - t0 > 30:
            log.error('timed out waiting for odom'); rclpy.shutdown(); return
        time.sleep(0.1)
    with node.lock:
        node.start['cf1'] = node.pos['cf1']
        node.start['cf2'] = node.pos['cf2']
    log.info(f"initial: cf1={tuple(round(v,3) for v in node.start['cf1'])} "
             f"cf2={tuple(round(v,3) for v in node.start['cf2'])}")

    # 2. takeoff
    log.info(f'taking off both to z={args.z} ...')
    node.takeoff_both()
    time.sleep(4.5)

    # 3. stream (settle -> exchange -> hold)
    log.info(f'streaming swap setpoints (settle {args.settle}s, exchange {args.exchange}s) ...')
    node.t_stream0 = time.monotonic()
    timer = node.create_timer(0.02, node._tick)  # 50 Hz
    total = args.settle + args.exchange + args.hold
    time.sleep(total)
    node.destroy_timer(timer)

    # 4. land
    log.info('landing ...')
    node.land_both()
    time.sleep(2.0)

    # 5. write + summarize
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    if node.rows:
        with open(args.out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(node.rows[0].keys()))
            w.writeheader(); w.writerows(node.rows)
    exch_rows = [r for r in node.rows if r['phase'] in ('exchange', 'hold')]
    pool = exch_rows or node.rows
    min_h = min(r['sep_h'] for r in pool)
    min_3d = min(r['sep_3d'] for r in pool)
    max_y1 = max(abs(r['y1']) for r in pool)
    max_y2 = max(abs(r['y2']) for r in pool)
    final = node.rows[-1]
    print('\n========== BVC HEAD-ON SUMMARY (%s) ==========' % args.label)
    print(f'rows logged          : {len(node.rows)}  (csv: {args.out})')
    print(f'initial separation   : {math.hypot(node.start["cf1"][0]-node.start["cf2"][0], node.start["cf1"][1]-node.start["cf2"][1]):.3f} m')
    print(f'MIN horiz separation : {min_h:.3f} m   (BVC cell guarantee ~0.60 m)')
    print(f'MIN 3D separation    : {min_3d:.3f} m')
    print(f'max |y| cf1 / cf2    : {max_y1:.3f} / {max_y2:.3f} m   (sidestep signature; ~0 if no BVC)')
    print(f'final cf1 pos        : ({final["x1"]:.2f},{final["y1"]:.2f},{final["z1"]:.2f})  target swap x={final["tgt1x"]}')
    print(f'final cf2 pos        : ({final["x2"]:.2f},{final["y2"]:.2f},{final["z2"]:.2f})  target swap x={final["tgt2x"]}')
    print('=============================================\n')

    ex.shutdown()          # stop the spin thread before tearing down the context
    spin.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
