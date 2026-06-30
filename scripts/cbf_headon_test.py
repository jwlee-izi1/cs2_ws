#!/usr/bin/env python3
"""CBF asymmetric head-on avoidance driver (first CBF migration test).

Unlike the BVC test (scripts/bvc_headon_test.py, symmetric onboard avoidance), this is an
ASYMMETRIC, ROS/Python-level Control Barrier Function filter:

  * cf2 = OBSTACLE. Streamed a straight, constant-speed setpoint to its goal. It does NOT
    avoid (firmware colAv is a no-op here — run with crazyflies_sitl_2drone_nobvc.yaml,
    peer_broadcast_hz:0). It just flies through.
  * cf1 = EGO. Treated as a 2D single integrator. Its desired velocity (P-control toward
    its goal, capped at --vmax) is passed through a CBF safety filter (a tiny 2-var QP)
    that guarantees it cannot enter a disk of radius --R around cf2. The filtered velocity
    becomes a position setpoint a short lead ahead of cf1's actual odom, streamed to
    /cf1/cmd_position. Firmware is never touched.

CBF (single integrator, xy-plane, altitude held at --z):
    h(p) = ||p_ego - p_obs||^2 - R^2          (h >= 0  <=>  safe)
    enforce  hdot + alpha*h >= 0:
    QP:  min ||v - v_des||^2
         s.t. 2 (p_ego - p_obs) . (v - v_obs) + alpha (||p_ego-p_obs||^2 - R^2) >= 0
    --no-vobs sets v_obs := 0 (ignore obstacle motion, the simplest cut). With v_obs on,
    cf2's velocity is estimated by EMA-filtered finite difference of its odom.

A single linear inequality + strictly-convex quadratic cost => the QP is never infeasible
(v can always grow along the constraint normal). Solved with qpsolvers/quadprog; if that
import fails we fall back to the exact closed-form half-space projection (identical for one
constraint).

DEGENERACY (see docs/sim_validation_log.md): in an EXACT head-on (cf1 goal directly behind
cf2, all on y=0) v_des points straight through the obstacle center, the QP has no lateral
gradient, and the filter can only BRAKE -> cf1 stalls/retreats, never circumnavigates
(BVC's firmware sidestepGoal breaks this symmetry; the basic CBF does not). Two ways to
break it: --lat (offsets cf2's goal in y) makes it geometrically near-head-on, OR the
right-hand BIAS (default ON, see right_bias()) perturbs the *goal velocity* sideways so the
ego always passes on its right even at exact --lat 0. --no-bias disables it (reproduces the
deadlock, for control runs). The bias only fires for a near, frontal obstacle, so ordinary
tracking is untouched.

Run AFTER sitl_2drone_headon.sh + the crazyswarm2 server (nobvc yaml) are up, both odoms live.
Usage:
  python3 scripts/cbf_headon_test.py --selftest                                  # offline bias check, no sim
  python3 scripts/cbf_headon_test.py --label cbf_bias_on  --lat 0.0           --out /tmp/cbf/lat0_on.csv
  python3 scripts/cbf_headon_test.py --label cbf_bias_off --lat 0.0 --no-bias --out /tmp/cbf/lat0_off.csv
  python3 scripts/cbf_headon_test.py --label cbf_regress  --lat 0.2           --out /tmp/cbf/lat02_on.csv
"""
import argparse
import csv
import math
import os
import threading
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from crazyflie_interfaces.msg import Position, Status
from crazyflie_interfaces.srv import Takeoff, Land, NotifySetpointsStop
from builtin_interfaces.msg import Duration

try:
    from qpsolvers import solve_qp
    _HAVE_QP = True
except Exception:                       # no solver installed -> analytic fallback
    _HAVE_QP = False


def _quat_to_rpy(x, y, z, w):
    """Quaternion -> (roll, pitch, yaw) in rad (ZYX). For attitude/flip logging."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (roll, pitch, yaw)


def cbf_filter(p_ego, p_obs, v_des, v_obs, R, alpha):
    """2-var CBF-QP. Returns (v_safe, h, active) where active=True if the filter changed v."""
    d = p_ego - p_obs                              # (2,)
    h = float(d @ d - R * R)
    # constraint:  2 d . (v - v_obs) + alpha h >= 0   <=>   (-2d) . v <= alpha h - 2 d.v_obs
    a = 2.0 * d
    rhs = alpha * h - 2.0 * float(d @ v_obs)       # need a . v >= -rhs ... see derivation below
    # rearrange to a.v + rhs_full >= 0 with rhs_full = alpha h - 2 d.v_obs  (since a.v_obs=2 d.v_obs)
    # i.e.  a . v >= 2 d.v_obs - alpha h
    lower = 2.0 * float(d @ v_obs) - alpha * h     # need a.v >= lower
    if _HAVE_QP:
        P = 2.0 * np.eye(2)
        q = -2.0 * v_des
        G = (-a).reshape(1, 2)                     # -a.v <= -lower
        hh = np.array([-lower])
        v = solve_qp(P, q, G, hh, solver='quadprog')
        if v is None:                              # numerical hiccup -> closed form
            v = _project_halfspace(v_des, a, lower)
    else:
        v = _project_halfspace(v_des, a, lower)
    v = np.asarray(v, dtype=float)
    active = bool(np.linalg.norm(v - v_des) > 1e-4)
    return v, h, active


def _project_halfspace(v_des, a, lower):
    """Exact solution of  min||v-v_des||^2 s.t. a.v >= lower  (single constraint)."""
    av = float(a @ v_des)
    if av >= lower:                                # v_des already safe
        return v_des
    aa = float(a @ a)
    if aa < 1e-12:                                 # ego and obstacle coincident: give up gracefully
        return v_des
    return v_des + ((lower - av) / aa) * a


def right_bias(p_ego, p_obs, v_des, R, bias_gain, bias_margin):
    """Right-hand symmetry-breaking bias added to v_des (NOT to the QP constraint).

    Cures the exact-head-on degeneracy: when v_des points straight through the obstacle
    centre the single-integrator CBF has no lateral gradient (v_y == 0, ego only brakes /
    retreats -> deadlock). We perturb the *goal velocity* sideways so the QP has a defined
    pass side, always to the ego's RIGHT, and only when it matters:

        bias = bias_gain * align * prox * right_hat

      align     = max(0, u_des . u_obs)      heading-into-obstacle cosine; 1 at exact head-on,
                                             0 (or clamped) when v_des is lateral / obstacle behind.
      prox      = clip((R + bias_margin - dist)/bias_margin, 0, 1)   1 at/inside R, ramps to 0
                                             at dist = R + bias_margin (far -> no bias).
      right_hat = (u_obs_y, -u_obs_x)        u_obs rotated -90deg (clockwise) = the ego's right.

    So bias is strong only for a *near, frontal* obstacle and ~0 for a side/far pass, which
    keeps ordinary tracking untouched. Self-limiting: once the ego veers right, u_des rotates
    off u_obs, align falls, bias fades -> no oscillation. Returns (bias_vec(2,), info dict).
    """
    d_vec = p_obs - p_ego
    dist = float(np.linalg.norm(d_vec))
    nv = float(np.linalg.norm(v_des))
    if dist < 1e-6 or nv < 1e-6 or bias_gain <= 0.0:
        return np.zeros(2), {'align': 0.0, 'prox': 0.0, 'mag': 0.0}
    u_obs = d_vec / dist
    u_des = v_des / nv
    align = max(0.0, float(u_des @ u_obs))             # heading-into-obstacle: 1 at head-on
    prox = (R + bias_margin - dist) / max(bias_margin, 1e-6)
    prox = min(1.0, max(0.0, prox))                    # 1 at/inside R, 0 beyond R+margin
    right_hat = np.array([u_obs[1], -u_obs[0]])        # -90deg rotation = ego's right side
    mag = bias_gain * align * prox
    return mag * right_hat, {'align': align, 'prox': prox, 'mag': mag}


class PeerFeed:
    """Models the HARDWARE peer-position feed degradation on top of sim's perfect cf2 state.

    Rationale (docs/hw_localization_path.md §3): on hardware cf1 does NOT see cf2's smooth,
    noise-free 30 Hz ground-truth odom. It sees cf2's own EKF estimate, relayed through the
    server hub and re-broadcast at ~10 Hz, POSITION-ONLY (no velocity), with 2-hop relay
    latency and Vicon+EKF noise. v_obs must be finite-differenced from that degraded position.

    This layer reproduces those three effects WITHOUT touching Gazebo physics or ego-pose
    injection (sim stays perfect-state) — only "the path cf2's position takes into the CBF
    node" is degraded, so sim avoidance stays intact and the rough peer input is isolated:

      * rate downsample  -> latch a new cf2 sample only every 1/hz s; zero-order hold between.
      * latency          -> the latched value is the TRUE position from (t - delay) ago.
      * noise            -> Gaussian added per axis at each latch (held until next latch).
      * (optional) proximity degrade -> noise grows as the drones close in (ghost-marker
        cue, motion_capture.yaml:57); off by default.

    v_obs = EMA-filtered finite difference computed ONLY at latch instants (using the peer
    period dt, not the 50 Hz tick) — differencing the held position every tick would give
    0 between latches and a huge spike at each latch. Lower `ema` = stronger low-pass.

    ⚠️ CONSERVATIVE ASSUMPTIONS, not measured: the repo pins down none of hw's noise sigma,
    delay, or close-range degradation (hw_localization_path.md "Open questions — NO REPO
    BASIS"). These defaults are a single plausible set to answer "does the barrier survive
    rough peer input?"; precise tuning waits for real-flight measurement.
    """

    def __init__(self, hz, delay, noise_std, ema, seed,
                 prox_degrade=False, prox_R=0.5, prox_gain=3.0, prox_margin=0.5):
        self.hz = max(hz, 1e-3)
        self.period = 1.0 / self.hz
        self.delay = max(delay, 0.0)
        self.noise_std = max(noise_std, 0.0)
        self.ema = min(max(ema, 0.0), 1.0)
        self.prox_degrade = prox_degrade
        self.prox_R = prox_R
        self.prox_gain = prox_gain
        self.prox_margin = max(prox_margin, 1e-6)
        self.rng = np.random.default_rng(seed)
        self.hist = deque()              # (t, true_xy) ring buffer for the delay lookup
        self.peer_pos = None             # last latched (held) noisy cf2 position the CBF sees
        self.v_obs = np.zeros(2)
        self._prev_latch = None
        self._last_latch_t = None

    def _sample_at(self, target_t):
        """Linear-interpolate the buffered TRUE position at target_t (for the relay delay)."""
        h = self.hist
        if not h:
            return None
        if target_t <= h[0][0]:
            return h[0][1].copy()
        if target_t >= h[-1][0]:
            return h[-1][1].copy()
        for i in range(len(h) - 1):
            t0, p0 = h[i]
            t1, p1 = h[i + 1]
            if t0 <= target_t <= t1:
                w = (target_t - t0) / max(t1 - t0, 1e-9)
                return p0 + w * (p1 - p0)
        return h[-1][1].copy()

    def update(self, t, true_xy, dist=None):
        """Push the latest true cf2 position; return (peer_pos seen by CBF, v_obs estimate)."""
        true_xy = np.asarray(true_xy, dtype=float)
        self.hist.append((t, true_xy.copy()))
        horizon = max(self.delay * 2.0, 1.0)
        while len(self.hist) > 2 and self.hist[0][0] < t - horizon:
            self.hist.popleft()

        if self._last_latch_t is None or (t - self._last_latch_t) >= self.period - 1e-9:
            delayed = self._sample_at(t - self.delay)
            if delayed is None:
                delayed = true_xy.copy()
            std = self.noise_std
            if self.prox_degrade and dist is not None:
                # ghost-marker analogue: ramp noise up as dist drops below prox_R+margin
                close = (self.prox_R + self.prox_margin - dist) / self.prox_margin
                std = std * (1.0 + self.prox_gain * min(1.0, max(0.0, close)))
            noisy = delayed + self.rng.normal(0.0, std, size=2) if std > 0 else delayed
            if self._prev_latch is not None and self._last_latch_t is not None:
                dt = t - self._last_latch_t
                if dt > 1e-4:
                    raw = (noisy - self._prev_latch) / dt
                    self.v_obs = (1.0 - self.ema) * self.v_obs + self.ema * raw
            self._prev_latch = noisy
            self._last_latch_t = t
            self.peer_pos = noisy
        if self.peer_pos is None:        # before first latch
            self.peer_pos = true_xy.copy()
        return self.peer_pos, self.v_obs


class CBFHeadOn(Node):
    def __init__(self, args):
        super().__init__('cbf_headon_test')
        self.args = args
        self.z = args.z
        # Internal role keys stay 'cf1'=ego(avoider), 'cf2'=obstacle (so all the
        # avoidance/bias/goal math below is unchanged). Only the ROS namespace each
        # role talks to is remapped via --ego-id/--obs-id. Sim default cf1/cf2 =>
        # byte-identical to Tests 3-5; HW uses --ego-id cf2 --obs-id cf4.
        self.ego_id = args.ego_id        # physical drone for the ego role ('cf1')
        self.obs_id = args.obs_id        # physical drone for the obstacle role ('cf2')
        self.pos = {'cf1': None, 'cf2': None}          # latest (x,y,z)
        self.start = {'cf1': None, 'cf2': None}        # recorded pre-takeoff
        self.goal = {'cf1': None, 'cf2': None}         # (x,y) targets
        self.att = {'cf1': None, 'cf2': None}          # latest (roll,pitch,yaw) rad (from odom)
        self.status = {'cf1': None, 'cf2': None}       # latest (vbat_V, tumbled int) from /status
        self.v_obs = np.zeros(2)                       # EMA-filtered cf2 velocity estimate
        # peer-feed degradation layer (see PeerFeed). --ideal-peer => transparent (50 Hz,
        # 0 delay, 0 noise) which reproduces the original smooth-sim path exactly.
        if args.ideal_peer:
            self.peer = PeerFeed(hz=50.0, delay=0.0, noise_std=0.0,
                                 ema=args.vobs_ema, seed=args.peer_seed)
        else:
            self.peer = PeerFeed(hz=args.peer_hz, delay=args.peer_delay,
                                 noise_std=args.peer_noise_std, ema=args.vobs_ema,
                                 seed=args.peer_seed, prox_degrade=args.proximity_degrade,
                                 prox_R=args.R)
        self.rows = []
        self.t_stream0 = None
        self.lock = threading.Lock()

        self.create_subscription(Odometry, f'/{self.ego_id}/odom', lambda m: self._odom('cf1', m), 10)
        self.create_subscription(Odometry, f'/{self.obs_id}/odom', lambda m: self._odom('cf2', m), 10)
        self.pub = {
            'cf1': self.create_publisher(Position, f'/{self.ego_id}/cmd_position', 10),
            'cf2': self.create_publisher(Position, f'/{self.obs_id}/cmd_position', 10),
        }
        _id = {'cf1': self.ego_id, 'cf2': self.obs_id}
        self.takeoff_cli = {n: self.create_client(Takeoff, f'/{_id[n]}/takeoff') for n in ('cf1', 'cf2')}
        self.land_cli = {n: self.create_client(Land, f'/{_id[n]}/land') for n in ('cf1', 'cf2')}
        # low->high handoff: needed before the high-level Land after low-level cmd_position
        # streaming (see land_both — missing this dropped both drones on HW 2026-06-30).
        self.notify_cli = {n: self.create_client(NotifySetpointsStop, f'/{_id[n]}/notify_setpoints_stop')
                           for n in ('cf1', 'cf2')}
        # best-effort battery + tumble feed (present only if the server yaml enables the
        # 'status' log topic; crazyflies_hw_2drone.yaml does). Absent feed -> '' in the CSV.
        self.create_subscription(Status, f'/{self.ego_id}/status', lambda m: self._status('cf1', m), 10)
        self.create_subscription(Status, f'/{self.obs_id}/status', lambda m: self._status('cf2', m), 10)

    def _odom(self, name, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        rpy = _quat_to_rpy(q.x, q.y, q.z, q.w)
        with self.lock:
            self.pos[name] = (p.x, p.y, p.z)
            self.att[name] = rpy

    def _status(self, name, msg):
        tumbled = int(bool(msg.supervisor_info & Status.SUPERVISOR_INFO_IS_TUMBLED))
        with self.lock:
            self.status[name] = (float(msg.battery_voltage), tumbled)

    @staticmethod
    def _vdes(p_xy, goal_xy, kp, vmax):
        """P-controller toward goal, capped at vmax (m/s)."""
        e = goal_xy - p_xy
        v = kp * e
        n = np.linalg.norm(v)
        if n > vmax:
            v = v / n * vmax
        return v

    # ---- 50 Hz stream + log timer ----
    def _tick(self):
        with self.lock:
            p1, p2 = self.pos['cf1'], self.pos['cf2']
            a1, a2 = self.att['cf1'], self.att['cf2']
            s1, s2 = self.status['cf1'], self.status['cf2']
        if p1 is None or p2 is None or self.t_stream0 is None:
            return
        t = time.monotonic() - self.t_stream0
        a = self.args
        p1xy = np.array([p1[0], p1[1]])
        p2xy = np.array([p2[0], p2[1]])

        # --- peer-feed degradation + obstacle-velocity estimate ---
        # cf1 perceives cf2 ONLY through the (optionally degraded) peer feed: p_obs and
        # v_obs below are built from peer_p2, not cf2's true odom. cf2's own obstacle
        # control and the logged separation still use the true position p2xy.
        dist_true = math.hypot(p1xy[0] - p2xy[0], p1xy[1] - p2xy[1])
        peer_p2, self.v_obs = self.peer.update(t, p2xy, dist_true)

        settle = a.settle
        if t < settle:
            phase = 'settle'
            tgt1 = np.array([self.start['cf1'][0], self.start['cf1'][1]])
            tgt2 = np.array([self.start['cf2'][0], self.start['cf2'][1]])
            vdes = np.zeros(2); vsafe = np.zeros(2); h = float('nan'); active = False
            bias_mag = 0.0
        else:
            phase = 'run' if t < settle + a.run else 'hold'
            # cf2 obstacle: straight, constant-speed setpoint toward its (offset) goal
            vdes2 = self._vdes(p2xy, self.goal['cf2'], a.kp, a.obs_speed)
            tgt2 = p2xy + vdes2 * a.lookahead
            # cf1 ego: CBF-filtered velocity toward its goal
            vdes = self._vdes(p1xy, self.goal['cf1'], a.kp, a.vmax)
            v_obs = np.zeros(2) if a.no_vobs else self.v_obs
            bias_mag = 0.0
            # p_obs the CBF/bias act on is the PERCEIVED (peer-feed) cf2 position, not true.
            if a.bypass:
                vsafe, h, active = vdes.copy(), float((p1xy - peer_p2) @ (p1xy - peer_p2) - a.R * a.R), False
            else:
                # right-hand symmetry-breaking bias on the *goal* velocity (cures head-on
                # deadlock); QP constraint unchanged. --no_bias reproduces the old behaviour.
                vdes_q = vdes
                if not a.no_bias:
                    bias, _binfo = right_bias(p1xy, peer_p2, vdes, a.R, a.bias_gain, a.bias_margin)
                    bias_mag = _binfo['mag']
                    vdes_q = vdes + bias
                    nb = float(np.linalg.norm(vdes_q))
                    if nb > a.vmax:                    # keep speed within the same cruise cap
                        vdes_q = vdes_q / nb * a.vmax
                vsafe, h, active = cbf_filter(p1xy, peer_p2, vdes_q, v_obs, a.R, a.alpha)
            tgt1 = p1xy + vsafe * a.lookahead

        for n, tgt in (('cf1', tgt1), ('cf2', tgt2)):
            m = Position()
            m.header.stamp = self.get_clock().now().to_msg()
            m.x, m.y, m.z, m.yaw = float(tgt[0]), float(tgt[1]), float(self.z), 0.0
            self.pub[n].publish(m)

        dx, dy, dz = p1[0]-p2[0], p1[1]-p2[1], p1[2]-p2[2]
        self.rows.append({
            't': round(t, 3), 'phase': phase,
            'x1': round(p1[0],4), 'y1': round(p1[1],4), 'z1': round(p1[2],4),
            'x2': round(p2[0],4), 'y2': round(p2[1],4), 'z2': round(p2[2],4),
            'sep_h': round(math.hypot(dx, dy),4), 'sep_3d': round(math.sqrt(dx*dx+dy*dy+dz*dz),4),
            'h': round(h, 4) if h == h else '',
            'vdes_x': round(float(vdes[0]),3), 'vdes_y': round(float(vdes[1]),3),
            'vsafe_x': round(float(vsafe[0]),3), 'vsafe_y': round(float(vsafe[1]),3),
            'vobs_x': round(float(self.v_obs[0]),3), 'vobs_y': round(float(self.v_obs[1]),3),
            'peer_x': round(float(peer_p2[0]),4), 'peer_y': round(float(peer_p2[1]),4),
            'peer_err': round(float(math.hypot(peer_p2[0]-p2[0], peer_p2[1]-p2[1])),4),
            'cbf_active': int(active),
            'bias_mag': round(float(bias_mag), 3), 'bias_active': int(bias_mag > 1e-3),
            'tgt1x': round(float(tgt1[0]),3), 'tgt1y': round(float(tgt1[1]),3),
            # attitude (flip detection) + battery + supervisor tumble flag
            'roll1': round(a1[0],3) if a1 else '', 'pitch1': round(a1[1],3) if a1 else '', 'yaw1': round(a1[2],3) if a1 else '',
            'roll2': round(a2[0],3) if a2 else '', 'pitch2': round(a2[1],3) if a2 else '', 'yaw2': round(a2[2],3) if a2 else '',
            'vbat1': round(s1[0],3) if s1 else '', 'tumb1': s1[1] if s1 else '',
            'vbat2': round(s2[0],3) if s2 else '', 'tumb2': s2[1] if s2 else '',
        })

    # ---- service helpers (same pattern as bvc_headon_test.py) ----
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
        # ---- low-level -> high-level handoff BEFORE Land (crash fix, HW 2026-06-30) ----
        # The run loop streams low-level /cfN/cmd_position, which latches low-level
        # setpoint priority in firmware. Calling the high-level Land without first
        # notifying setpoints-stop leaves Land IGNORED -> the last low-level setpoint
        # expires with nothing replacing it -> the drone falls (both cf2+cf4 dropped,
        # cf2 flipped). notify_setpoints_stop is crazyswarm2's standard handoff: it
        # relinquishes low-level priority so the high-level Land actually engages.
        nfuts = []
        for n in ('cf1', 'cf2'):
            if self.notify_cli[n].wait_for_service(timeout_sec=2.0):
                req = NotifySetpointsStop.Request()
                req.group_mask = 0
                req.remain_valid_millisecs = 0      # last setpoint invalid now -> high-level takes over
                nfuts.append(self.notify_cli[n].call_async(req))
        self._await(nfuts, 2.0)
        time.sleep(0.15)                            # let the handoff settle before Land
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


def _selftest(args):
    """Offline (no ROS/sim) sanity check that the right-hand bias turns ON for a near,
    frontal obstacle and OFF for a side / far / behind one. Ego at origin heading +x."""
    p_ego = np.array([0.0, 0.0])
    vdes = np.array([args.vmax, 0.0])                  # heading straight +x toward goal
    R, bg, bm = args.R, args.bias_gain, args.bias_margin
    print(f'\n=== right_bias offline self-test  (R={R}, bias_gain={bg}, bias_margin={bm}) ===')
    print(f'ego@(0,0) v_des=(+{args.vmax},0) [heading +x];  bias ON within dist<R+margin={R+bm:.2f} m\n')
    cases = [
        ('exact head-on  (obs dead ahead, close)',     np.array([ 0.55,  0.00]), 'STRONG, -y'),
        ('near head-on   (lat=0.2 analogue)',          np.array([ 0.55,  0.20]), 'strong, -y'),
        ('oblique 45deg',                              np.array([ 0.45,  0.45]), 'moderate'),
        ('side pass      (mostly lateral)',            np.array([ 0.20,  0.80]), '~0'),
        ('far ahead      (beyond R+margin)',           np.array([ 2.00,  0.00]), '0 (far)'),
        ('obstacle behind',                            np.array([-0.55,  0.00]), '0 (behind)'),
    ]
    print(f'{"case":<42}{"dist":>6}{"align":>7}{"prox":>6}{"|bias|":>8}   bias(x,y)        expect')
    for name, p_obs, expect in cases:
        bias, info = right_bias(p_ego, p_obs, vdes, R, bg, bm)
        dist = float(np.linalg.norm(p_obs - p_ego))
        print(f'{name:<42}{dist:>6.2f}{info["align"]:>7.2f}{info["prox"]:>6.2f}'
              f'{info["mag"]:>8.3f}   ({bias[0]:+.3f},{bias[1]:+.3f})   {expect}')
    print('\nExpect: exact/near head-on -> strong bias to -y (ego right); side/far/behind -> ~0.')
    print('Sign check: heading +x, obstacle ahead -> right_hat=(0,-1) -> ego veers to -y '
          '(matches BVC Test 2 cf1->-y).\n')


def _selftest_peer(args):
    """Offline (no ROS/sim) check: push a known constant-velocity cf2 trajectory through the
    PeerFeed at 50 Hz and measure how rough v_obs gets — ideal vs degraded vs strong-filter.

    cf2 flies -x at obs_speed (true v_obs = (-obs_speed, 0)); we compare each feed's v_obs
    against that truth. 'rough' = std of step-to-step v_obs change (jitter)."""
    obs_speed = args.obs_speed
    dt = 0.02                                          # 50 Hz tick (matches the live timer)
    T = 8.0
    n = int(T / dt)
    true_v = np.array([-obs_speed, 0.0])
    x0 = 2.0
    configs = [
        ('ideal   (50Hz, 0 delay, 0 noise, ema=0.3)',
         PeerFeed(hz=50.0, delay=0.0, noise_std=0.0, ema=0.3, seed=args.peer_seed)),
        (f'degraded ({args.peer_hz:.0f}Hz, {args.peer_delay*1e3:.0f}ms, '
         f'{args.peer_noise_std*100:.1f}cm, ema={args.vobs_ema})',
         PeerFeed(hz=args.peer_hz, delay=args.peer_delay, noise_std=args.peer_noise_std,
                  ema=args.vobs_ema, seed=args.peer_seed)),
        (f'degraded + strong LPF (ema={args.vobs_ema/2:.3f})',
         PeerFeed(hz=args.peer_hz, delay=args.peer_delay, noise_std=args.peer_noise_std,
                  ema=args.vobs_ema / 2.0, seed=args.peer_seed)),
    ]
    print('\n=== peer-feed v_obs roughness self-test (offline) ===')
    print(f'cf2 true velocity = (-{obs_speed:.2f}, 0) m/s; settle 1.0s, then move.')
    print('ASSUMED hw params (no repo basis): see PeerFeed docstring / hw_localization_path.md.\n')
    print(f'{"feed":<46}{"v_obs bias":>11}{"v_obs std":>11}{"max|err|":>10}{"jitter":>9}')
    for name, feed in configs:
        vs = []
        prevx = None
        for k in range(n):
            t = k * dt
            x = x0 if t < 1.0 else x0 + true_v[0] * (t - 1.0)   # settle 1s then move
            _, v_obs = feed.update(t, np.array([x, 0.0]))
            if t >= 2.0:                               # let the filter settle past warm-up
                vs.append(v_obs.copy())
        vs = np.array(vs)
        err = vs - true_v
        speed_err = np.linalg.norm(err, axis=1)
        jitter = np.std(np.linalg.norm(np.diff(vs, axis=0), axis=1)) if len(vs) > 1 else 0.0
        bias = np.linalg.norm(err.mean(axis=0))
        print(f'{name:<46}{bias:>11.4f}{np.linalg.norm(err.std(axis=0)):>11.4f}'
              f'{speed_err.max():>10.4f}{jitter:>9.4f}')
    print('\nbias=mean error (delay shows here), std=spread, max|err|=worst spike, '
          'jitter=step-to-step v_obs change (roughness).')
    print('Expect: degraded >> ideal on std/max/jitter; strong LPF cuts std/jitter '
          '(at the cost of a bit more bias/lag).\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', default='run')
    ap.add_argument('--out', default='/tmp/cbf/run.csv')
    # Drone ID mapping (role -> physical drone). Defaults reproduce sim Tests 3-5
    # exactly; HW (cf1/cf3 absent 2026-06-30) uses --ego-id cf2 --obs-id cf4.
    ap.add_argument('--ego-id', dest='ego_id', default='cf1', help='ego/avoider drone id (HW: cf2)')
    ap.add_argument('--obs-id', dest='obs_id', default='cf2', help='obstacle drone id (HW: cf4)')
    ap.add_argument('--z', type=float, default=1.0)
    ap.add_argument('--settle', type=float, default=3.0, help='hold-at-start seconds before run')
    ap.add_argument('--run', type=float, default=14.0, help='CBF streaming seconds')
    ap.add_argument('--hold', type=float, default=2.0, help='hold seconds after run')
    # CBF / scenario params
    ap.add_argument('--R', type=float, default=0.5, help='CBF safety radius (m)')
    ap.add_argument('--alpha', type=float, default=2.0, help='CBF class-K gain')
    ap.add_argument('--vmax', type=float, default=0.4, help='ego cruise speed cap (m/s)')
    ap.add_argument('--obs-speed', dest='obs_speed', type=float, default=0.4, help='obstacle speed (m/s)')
    ap.add_argument('--kp', type=float, default=1.5, help='P-gain of v_des toward goal')
    ap.add_argument('--lookahead', type=float, default=0.6, help='setpoint lead time = v*lookahead (s)')
    ap.add_argument('--lat', type=float, default=0.2, help="cf2 goal y-offset; breaks head-on symmetry (0=exact head-on)")
    ap.add_argument('--goal-beyond', dest='goal_beyond', type=float, default=0.0,
                    help='extend the ego goal this many metres PAST the obstacle along the approach '
                         'line. 0 = goal AT the obstacle (a STATIC obstacle then stalls the ego at the '
                         'barrier tangent, ~90deg); >0 lets the ego complete a half-loop and continue '
                         'straight out the far side. Default 0 reproduces sim Tests 3-5.')
    ap.add_argument('--no-vobs', dest='no_vobs', action='store_true', help='assume obstacle static (v_obs=0)')
    ap.add_argument('--bypass', action='store_true', help='disable CBF (ego flies straight -> collision-course control)')
    # right-hand bias (cures the exact-head-on deadlock; see right_bias())
    ap.add_argument('--bias-gain', dest='bias_gain', type=float, default=0.3,
                    help='peak lateral bias speed (m/s) at full head-on + close range')
    ap.add_argument('--bias-margin', dest='bias_margin', type=float, default=0.7,
                    help='proximity ramp width (m): bias starts at dist=R+margin, full at dist<=R')
    ap.add_argument('--no-bias', dest='no_bias', action='store_true',
                    help='disable the right-hand bias -> reproduces the head-on deadlock (control)')
    ap.add_argument('--selftest', action='store_true',
                    help='run the offline bias unit-check (no ROS/sim) and exit')
    # ---- hardware peer-feed degradation layer (see PeerFeed; docs/hw_localization_path.md §3) ----
    # CONSERVATIVE ASSUMPTIONS — repo has no measured hw noise/delay/rate. Tune after flight.
    ap.add_argument('--ideal-peer', dest='ideal_peer', action='store_true',
                    help='transparent peer feed (50 Hz, 0 delay, 0 noise) = original smooth sim path')
    ap.add_argument('--peer-hz', dest='peer_hz', type=float, default=10.0,
                    help='peer broadcast rate (Hz); zero-order hold between (hw=10, sim=30)')
    ap.add_argument('--peer-delay', dest='peer_delay', type=float, default=0.1,
                    help='2-hop relay delay (s) applied to perceived cf2 position (assumed)')
    ap.add_argument('--peer-noise-std', dest='peer_noise_std', type=float, default=0.015,
                    help='Gaussian position noise std (m) per axis on perceived cf2 (assumed)')
    ap.add_argument('--peer-seed', dest='peer_seed', type=int, default=0,
                    help='RNG seed for reproducible peer noise')
    ap.add_argument('--proximity-degrade', dest='proximity_degrade', action='store_true',
                    help='grow peer noise as drones close in (ghost-marker cue; off by default)')
    ap.add_argument('--vobs-ema', dest='vobs_ema', type=float, default=0.3,
                    help='EMA weight on new finite-diff sample for v_obs (lower = stronger low-pass)')
    ap.add_argument('--selftest-peer', dest='selftest_peer', action='store_true',
                    help='offline v_obs-roughness check across ideal/degraded/filtered feeds; exit')
    args = ap.parse_args()

    if args.selftest:
        _selftest(args); return
    if args.selftest_peer:
        _selftest_peer(args); return

    rclpy.init()
    node = CBFHeadOn(args)
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()

    log = node.get_logger()
    log.info(f"CBF filter: {'qpsolvers/quadprog' if _HAVE_QP else 'analytic half-space (no solver)'}; "
             f"R={args.R} alpha={args.alpha} vmax={args.vmax} lat={args.lat} "
             f"v_obs={'OFF' if args.no_vobs else 'ON'} bypass={args.bypass} "
             f"bias={'OFF' if args.no_bias else f'ON(gain={args.bias_gain},margin={args.bias_margin})'}")
    log.info('waiting for /cf1/odom and /cf2/odom ...')
    t0 = time.monotonic()
    while node.pos['cf1'] is None or node.pos['cf2'] is None:
        if time.monotonic() - t0 > 30:
            log.error('timed out waiting for odom'); rclpy.shutdown(); return
        time.sleep(0.1)
    with node.lock:
        node.start['cf1'] = node.pos['cf1']
        node.start['cf2'] = node.pos['cf2']
    # cf1 ego goal = cf2 start, extended --goal-beyond m PAST it along the approach line
    # (ego_start -> obstacle). goal_beyond=0 => goal at obstacle (swap target, sim default);
    # >0 => goal past the obstacle so a static obstacle no longer stalls the ego at the barrier.
    ego_s = np.array([node.start['cf1'][0], node.start['cf1'][1]])
    obs_s = np.array([node.start['cf2'][0], node.start['cf2'][1]])
    approach = obs_s - ego_s
    d = float(np.linalg.norm(approach))
    u = approach / d if d > 1e-6 else np.array([1.0, 0.0])
    node.goal['cf1'] = obs_s + u * args.goal_beyond
    # cf2 obstacle goal = cf1 start, offset in y by --lat
    node.goal['cf2'] = np.array([node.start['cf1'][0], node.start['cf1'][1] + args.lat])
    log.info(f"initial: cf1={tuple(round(v,3) for v in node.start['cf1'])} "
             f"cf2={tuple(round(v,3) for v in node.start['cf2'])} | "
             f"cf1 goal={tuple(node.goal['cf1'].round(2))} cf2 goal={tuple(node.goal['cf2'].round(2))}")

    log.info(f'taking off both to z={args.z} ...')
    node.takeoff_both()
    time.sleep(4.5)

    log.info(f'streaming (settle {args.settle}s, run {args.run}s) ...')
    node.t_stream0 = time.monotonic()
    timer = node.create_timer(0.02, node._tick)  # 50 Hz
    time.sleep(args.settle + args.run + args.hold)
    node.destroy_timer(timer)

    log.info('landing ...')
    node.land_both()
    time.sleep(2.0)

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    if node.rows:
        with open(args.out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(node.rows[0].keys()))
            w.writeheader(); w.writerows(node.rows)
    run_rows = [r for r in node.rows if r['phase'] in ('run', 'hold')]
    pool = run_rows or node.rows
    min_h = min(r['sep_h'] for r in pool)
    min_3d = min(r['sep_3d'] for r in pool)
    max_y1 = max(abs(r['y1']) for r in pool)
    n_active = sum(r['cbf_active'] for r in pool)
    n_bias = sum(r.get('bias_active', 0) for r in pool)
    max_bias = max((r.get('bias_mag', 0.0) for r in pool), default=0.0)
    final = node.rows[-1]
    reached = math.hypot(final['x1'] - node.goal['cf1'][0], final['y1'] - node.goal['cf1'][1])
    print('\n========== CBF HEAD-ON SUMMARY (%s) ==========' % args.label)
    print(f'rows logged          : {len(node.rows)}  (csv: {args.out})')
    print(f'R / alpha / vmax     : {args.R} / {args.alpha} / {args.vmax}   v_obs={"OFF" if args.no_vobs else "ON"}  bypass={args.bypass}  lat={args.lat}')
    print(f'bias                 : {"OFF" if args.no_bias else f"ON (gain={args.bias_gain}, margin={args.bias_margin})"}   '
          f'active {n_bias}/{len(pool)} ticks ({100*n_bias/max(1,len(pool)):.0f}%)  max|bias|={max_bias:.3f} m/s')
    print(f'MIN horiz separation : {min_h:.3f} m   (safety radius R={args.R}; BVC baseline 0.61 m)')
    print(f'  -> barrier {"HELD" if min_h >= args.R - 0.05 else "VIOLATED"} (R-0.05 tol)')
    print(f'MIN 3D separation    : {min_3d:.3f} m')
    print(f'max |y| cf1          : {max_y1:.3f} m   (circumnavigation; ~0 => braked, no sidestep)')
    print(f'CBF active ticks     : {n_active}/{len(pool)}  ({100*n_active/max(1,len(pool)):.0f}% of run)')
    print(f'final cf1 pos        : ({final["x1"]:.2f},{final["y1"]:.2f},{final["z1"]:.2f})  '
          f'dist-to-goal {reached:.2f} m -> {"REACHED" if reached < 0.25 else "STALLED/DEADLOCK"}')
    print(f'final cf2 pos        : ({final["x2"]:.2f},{final["y2"]:.2f},{final["z2"]:.2f})')
    print('=============================================\n')

    ex.shutdown()
    spin.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
