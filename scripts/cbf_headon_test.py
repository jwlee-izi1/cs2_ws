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


def steer_behind(p_ego, p_obs, v_des, v_obs, R, gain, margin):
    """Crossing-avoidance lateral bias: steer the ego to pass BEHIND a moving obstacle.

    The right_bias() above fixes the HEAD-ON degeneracy by always veering to the ego's
    right — correct when the obstacle closes head-on, but WRONG for a crossing obstacle
    (it can veer the ego INTO the obstacle's travel direction and chase it). This variant
    instead reads the obstacle's motion and biases v_des toward the side the obstacle is
    LEAVING (its tail), so the ego slips behind it and continues — an active circumnavigation
    rather than a brake-and-wait. Used for the circle scenario (--pass-behind).

        d        = p_obs - p_ego,  u = d/|d|      line of sight toward the obstacle
        perp     = (-u_y, u_x)                    +90deg of the LOS
        v_lat    = v_obs . perp                   obstacle's crossing (lateral) speed, signed
        side     = -sign(v_lat)                   dodge OPPOSITE the obstacle's lateral motion
                                                  = toward where it came from = pass behind
        bias     = gain * align * prox * side * perp

      align/prox gate exactly like right_bias (near + frontal only), so tracking is untouched
      when the obstacle is far or off to the side. If the obstacle has ~no lateral motion
      (closing straight along the LOS = true head-on) v_lat->0 and we fall back to the ego's
      right (side=+1), recovering right_bias's head-on behaviour."""
    d = p_obs - p_ego
    dist = float(np.linalg.norm(d))
    nv = float(np.linalg.norm(v_des))
    if dist < 1e-6 or nv < 1e-6 or gain <= 0.0:
        return np.zeros(2), {'align': 0.0, 'prox': 0.0, 'mag': 0.0}
    u = d / dist
    u_des = v_des / nv
    align = max(0.0, float(u_des @ u))
    prox = min(1.0, max(0.0, (R + margin - dist) / max(margin, 1e-6)))
    perp = np.array([-u[1], u[0]])
    v_lat = float(np.asarray(v_obs) @ perp)
    side = -1.0 if v_lat > 1e-3 else (1.0 if v_lat < -1e-3 else 1.0)   # pass behind; head-on -> right
    mag = gain * align * prox
    return mag * side * perp, {'align': align, 'prox': prox, 'mag': mag}


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
        # line-following (problem-1 fix): approach-axis frame (origin=ego_start, along=u,
        # perp=u rotated +90deg) + goal's along-track coord. Set in main() if --line-follow.
        self.axis_origin = None; self.axis_along = None; self.axis_perp = None; self.s_goal = 0.0
        # circle-obstacle scenario (--obs-circle): obstacle drives a circle instead of a
        # straight line. phi0 = its start phase on that circle (derived from spawn in main()
        # so there's no jump at run start); radius/center/omega come from args.
        self.obs_phi0 = 0.0
        self.land_rows = []                            # z/attitude captured DURING the land descent

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

    def _vdes_line(self, p_xy, peer_xy, a):
        """Line-following v_des for the EGO (problem-1 fix). Instead of P-controlling
        straight at the goal POINT (which, from an off-axis sidestep, cuts a diagonal to
        the goal), track the original approach AXIS:

            along-track: P-control toward the goal's axis-position (caps at vmax, decels at goal)
            cross-track: -kct * e_ct  -> pulls the ego back ONTO the axis

        The cross-track term is GATED off near the obstacle (dist < R+bias_margin) so the
        right-hand bias / CBF can still sidestep freely during the encounter; once clear it
        ramps back on, so the ego rejoins the axis and continues straight out (no diagonal).
        Bias+QP downstream are unchanged — this only replaces the ego's base v_des."""
        d = p_xy - self.axis_origin
        s = float(d @ self.axis_along)                 # along-track distance from ego_start
        e_ct = float(d @ self.axis_perp)               # signed cross-track offset from axis
        v_along = float(np.clip(a.kp * (self.s_goal - s), -a.vmax, a.vmax))
        # gate cross-track by ALONG-TRACK progress past the obstacle (not raw distance): the
        # CBF keeps the ego ~R from the obstacle throughout the circumnav, so a distance gate
        # would stay ~0 and never let it return. Turn on once the ego passes the obstacle's
        # along-track position; the CBF still prevents any barrier penetration during the return.
        if a.ct_dist_gate:
            # DISTANCE-gated cross-track return (circle scenario): pull back onto the axis
            # whenever CLEAR of the obstacle, release near it. gate=0 at the barrier (dist=R),
            # ramps to 1 by dist=R+bias_margin. With an ORBITING obstacle the ego meets it
            # twice and must rejoin the axis BETWEEN the two passes — the along-track gate
            # below can't do that (obstacle's along-track coord isn't monotonic), a distance
            # gate does: far from the obstacle -> return; close -> dodge freely.
            dist = float(np.linalg.norm(p_xy - peer_xy))
            gate = min(1.0, max(0.0, (dist - a.R) / max(a.bias_margin, 1e-6)))
        else:
            # ALONG-TRACK gate (head-on default): the CBF keeps the ego ~R from a straight-line
            # obstacle throughout the circumnav, so a distance gate would stay ~0 and never let
            # it return; turn on once the ego passes the obstacle's along-track position instead.
            s_obs = float((peer_xy - self.axis_origin) @ self.axis_along)
            gate = min(1.0, max(0.0, (s - s_obs) / max(a.bias_margin, 1e-6)))
        v_ct = -a.kct * e_ct * gate
        v = v_along * self.axis_along + v_ct * self.axis_perp
        n = np.linalg.norm(v)
        if n > a.vmax:
            v = v / n * a.vmax
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
            if a.obs_circle:
                # cf2 obstacle: drive a circle of radius --obs-radius about --obs-center at
                # angular speed --obs-omega (rad/s, sign = CCW/CW). Command the circle POINT
                # (a lookahead ahead in phase = a tangential lead) rather than a velocity, so
                # firmware tracks the geometric path directly. tc measured from run start
                # (obstacle held at spawn through settle); phi0 set from spawn => no jump.
                tc = t - settle
                cx, cy = a.obs_center
                ang = self.obs_phi0 + a.obs_omega * (tc + a.lookahead)
                tgt2 = np.array([cx + a.obs_radius * math.cos(ang),
                                 cy + a.obs_radius * math.sin(ang)])
            else:
                # cf2 obstacle: straight, constant-speed setpoint toward its (offset) goal
                vdes2 = self._vdes(p2xy, self.goal['cf2'], a.kp, a.obs_speed)
                tgt2 = p2xy + vdes2 * a.lookahead
            # cf1 ego: CBF-filtered velocity toward its goal. --line-follow tracks the
            # approach axis (returns to it after the sidestep); else P-control at the point.
            if a.line_follow and self.axis_along is not None:
                vdes = self._vdes_line(p1xy, peer_p2, a)
            else:
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
                    if a.pass_behind:                  # crossing bias: steer behind the moving obstacle
                        bias, _binfo = steer_behind(p1xy, peer_p2, vdes, v_obs, a.R, a.bias_gain, a.bias_margin)
                    else:                              # head-on bias: fixed right-hand
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
        # remain_valid_millisecs > 0 (problem-2 fix): keep the LAST hover setpoint valid
        # long enough to bridge the notify->Land gap. With 0 the setpoint was invalidated
        # immediately, leaving a ~200 ms UNCOMMANDED window before Land engaged -> on HW the
        # drone dipped, then Land re-descended = "double landing" (sim's idealised dynamics
        # rode through the gap, so it only showed on HW). 400 ms covers the handoff.
        nfuts = []
        for n in ('cf1', 'cf2'):
            if self.notify_cli[n].wait_for_service(timeout_sec=2.0):
                req = NotifySetpointsStop.Request()
                req.group_mask = 0
                req.remain_valid_millisecs = 400
                nfuts.append(self.notify_cli[n].call_async(req))
        self._await(nfuts, 2.0)                     # no sleep: Land immediately, within the 400 ms bridge
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
        # log the land descent (the 50 Hz _tick timer is already stopped, so the main CSV
        # ends at hold; this captures z/attitude THROUGH the land to verify a single smooth ramp).
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.args.land_log:
            with self.lock:
                p1, p2, a1 = self.pos['cf1'], self.pos['cf2'], self.att['cf1']
            self.land_rows.append({
                't': round(time.monotonic() - t0, 3),
                'z1': round(p1[2], 4) if p1 else '', 'x1': round(p1[0], 4) if p1 else '', 'y1': round(p1[1], 4) if p1 else '',
                'z2': round(p2[2], 4) if p2 else '',
                'roll1': round(a1[0], 3) if a1 else '', 'pitch1': round(a1[1], 3) if a1 else '',
            })
            time.sleep(0.05)                        # ~20 Hz

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


def _find_encounters(ts, seps, thresh):
    """Return one (t, sep) per distinct ENCOUNTER: a contiguous excursion where the
    separation dips below `thresh`, reduced to that excursion's minimum. Hysteresis
    (separation must climb back above `thresh` before a new encounter is counted)
    collapses the wiggly real-sim sep trace into the true number of passes — so a clean
    two-encounter run reports exactly two, not one-per-local-wiggle."""
    enc = []
    in_dip = False
    best_t = None; best_s = None
    for t, s in zip(ts, seps):
        if s < thresh:
            if not in_dip or s < best_s:
                best_t, best_s = t, s
            in_dip = True
        else:
            if in_dip:
                enc.append((best_t, best_s))
            in_dip = False
    if in_dip:
        enc.append((best_t, best_s))
    return enc


def _selftest_circle(args):
    """Offline (no ROS/sim) closed-loop kinematic run of the circle-obstacle scenario.

    Ego = 2D single integrator with the SAME line-follow v_des + right-hand bias + CBF-QP
    used live (scripts _vdes_line / right_bias / cbf_filter); obstacle = exact circle point,
    v_obs = analytic circle velocity (the 'v_obs ON, ideal peer' case). Geometry: the ego
    goal is --ego-goal and the ego start is placed diametrically opposite across --obs-center,
    so the ego path is a DIAMETER through the centre and crosses the orbit at two points.

    Prints the two encounter min-seps + whether the ego returns to the axis — the fast loop
    for tuning omega / phase0 so BOTH crossings produce an encounter before a real sim run.
    """
    C = np.array(args.obs_center, dtype=float)
    r = args.obs_radius if args.obs_radius > 0 else 1.0
    omega = args.obs_omega
    phi0 = math.radians(args.obs_phase0_deg) if args.obs_phase0_deg is not None else math.radians(-135.0)
    goal = np.array(args.ego_goal, dtype=float) if args.ego_goal is not None else (C + np.array([0.0, 1.5]))
    ego0 = 2.0 * C - goal                              # diametrically opposite the goal
    step = goal - ego0
    d = float(np.linalg.norm(step))
    u = step / d if d > 1e-6 else np.array([0.0, 1.0])
    perp = np.array([-u[1], u[0]])
    s_goal = d
    dt = 0.02
    T = args.run
    n = int(T / dt)
    p1 = ego0.copy()
    ts, seps, cts, actives = [], [], [], []
    for k in range(n):
        t = k * dt
        ang = phi0 + omega * t
        p2 = C + r * np.array([math.cos(ang), math.sin(ang)])
        v_obs = np.zeros(2) if args.no_vobs else omega * r * np.array([-math.sin(ang), math.cos(ang)])
        # ego line-follow v_des (mirror of _vdes_line)
        dd = p1 - ego0
        s = float(dd @ u); e_ct = float(dd @ perp)
        v_along = float(np.clip(args.kp * (s_goal - s), -args.vmax, args.vmax))
        if args.ct_dist_gate:
            dist = float(np.linalg.norm(p1 - p2))
            gate = min(1.0, max(0.0, (dist - args.R) / max(args.bias_margin, 1e-6)))
        else:
            s_obs = float((p2 - ego0) @ u)
            gate = min(1.0, max(0.0, (s - s_obs) / max(args.bias_margin, 1e-6)))
        v_ct = -args.kct * e_ct * gate
        vdes = v_along * u + v_ct * perp
        nv = float(np.linalg.norm(vdes))
        if nv > args.vmax:
            vdes = vdes / nv * args.vmax
        vdes_q = vdes
        if not args.no_bias:
            if args.pass_behind:
                bias, _ = steer_behind(p1, p2, vdes, v_obs, args.R, args.bias_gain, args.bias_margin)
            else:
                bias, _ = right_bias(p1, p2, vdes, args.R, args.bias_gain, args.bias_margin)
            vdes_q = vdes + bias
            nb = float(np.linalg.norm(vdes_q))
            if nb > args.vmax:
                vdes_q = vdes_q / nb * args.vmax
        vsafe, h, active = cbf_filter(p1, p2, vdes_q, v_obs, args.R, args.alpha)
        p1 = p1 + vsafe * dt
        ts.append(t); seps.append(float(np.linalg.norm(p1 - p2)))
        cts.append(e_ct); actives.append(active)
    enc = _find_encounters(ts, seps, thresh=args.R + 0.4)
    reached = float(np.linalg.norm(p1 - goal))
    final_ct = float((p1 - ego0) @ perp)
    max_ct = max(abs(c) for c in cts)
    print('\n=== circle-scenario offline kinematic self-test ===')
    print(f'center={tuple(C)} radius={r:.2f} omega={omega:.4f} rad/s (period {2*math.pi/max(abs(omega),1e-9):.1f}s) '
          f'phi0={math.degrees(phi0):.1f}deg')
    print(f'ego start={tuple(ego0.round(2))} -> goal={tuple(goal.round(2))}  (diameter len {d:.2f} m)  '
          f'vmax={args.vmax} R={args.R} alpha={args.alpha} bias={"OFF" if args.no_bias else "ON"}')
    print(f'ego expected diameter-transit time (no avoidance) = {d/args.vmax:.1f}s; '
          f'obstacle half-turn = {math.pi/max(abs(omega),1e-9):.1f}s\n')
    print(f'ENCOUNTERS (sep local minima < R+0.4={args.R+0.4:.2f}): {len(enc)} found')
    for i, (te, se) in enumerate(enc, 1):
        print(f'  #{i}  t={te:5.2f}s   min-sep={se:.3f} m   barrier {"HELD" if se >= args.R-0.03 else "VIOLATED"} (R={args.R})')
    if len(enc) < 2:
        print('  -> FEWER than 2 encounters. Timing knobs: if the 2nd is missed, the ego (slowed by')
        print('     the 1st sidestep) reaches the far crossing AFTER the obstacle left it -> LOWER --obs-omega')
        print('     (obstacle lingers) or nudge --obs-phase0-deg. If encounters merge into 1, RAISE omega.')
    print(f'\noverall min-sep = {min(seps):.3f} m (R={args.R})   max |cross-track| = {max_ct:.3f} m (sidestep)')
    print(f'ego final = {tuple(p1.round(2))}  dist-to-goal {reached:.2f} m -> {"REACHED" if reached < 0.25 else "SHORT"}; '
          f'final cross-track {final_ct:+.3f} m -> {"back on axis" if abs(final_ct) < 0.1 else "OFF axis"}\n')
    return enc


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
    # ---- circle-obstacle scenario (obstacle orbits; ego crosses a diameter -> two encounters) ----
    ap.add_argument('--obs-circle', dest='obs_circle', action='store_true',
                    help='obstacle drives a CIRCLE (radius/center/omega below) instead of a straight line. '
                         'Pair with --ego-goal on the opposite side so the ego line is a diameter through '
                         'the centre and crosses the orbit at two points (two avoidance encounters).')
    ap.add_argument('--obs-center', dest='obs_center', type=float, nargs=2, default=[0.0, 0.0],
                    metavar=('CX', 'CY'), help='circle centre (m); ego diameter line should pass through it')
    ap.add_argument('--obs-radius', dest='obs_radius', type=float, default=-1.0,
                    help='circle radius (m); <=0 => derive from the obstacle spawn (spawn lies on the circle)')
    ap.add_argument('--obs-omega', dest='obs_omega', type=float, default=0.3927,
                    help='obstacle angular speed (rad/s; + = CCW). Default pi*0.25/2 = half-turn while an '
                         'ego at vmax=0.25 crosses a r=1 diameter (default two-encounter timing knob).')
    ap.add_argument('--obs-phase0-deg', dest='obs_phase0_deg', type=float, default=None,
                    help='override the obstacle start phase on the circle (deg). Default None => derive '
                         'from spawn (no jump). Set it (and match the spawn) to tune two-encounter timing.')
    ap.add_argument('--ego-goal', dest='ego_goal', type=float, nargs=2, default=None,
                    metavar=('GX', 'GY'), help='set the ego goal EXPLICITLY (overrides the head-on '
                         'obstacle-derived goal). Use for the circle scenario: the far side of the diameter.')
    ap.add_argument('--selftest-circle', dest='selftest_circle', action='store_true',
                    help='offline (no ROS/sim) closed-loop kinematic run of the circle scenario; reports '
                         'the two encounter min-seps + ego axis return, for timing tuning. Exit.')
    ap.add_argument('--goal-beyond', dest='goal_beyond', type=float, default=0.0,
                    help='extend the ego goal this many metres PAST the obstacle along the approach '
                         'line. 0 = goal AT the obstacle (a STATIC obstacle then stalls the ego at the '
                         'barrier tangent, ~90deg); >0 lets the ego complete a half-loop and continue '
                         'straight out the far side. Default 0 reproduces sim Tests 3-5.')
    ap.add_argument('--line-follow', dest='line_follow', action='store_true',
                    help='ego tracks the original approach AXIS (returns to it after the sidestep and '
                         'continues straight) instead of P-controlling at the goal point (which cuts a '
                         'diagonal from an off-axis sidestep). Default off = old point-seeking (sim Tests 3-5).')
    ap.add_argument('--kct', type=float, default=2.0,
                    help='cross-track return gain for --line-follow (higher = snappier axis return; too '
                         'high may oscillate under HW lag). Gated on once the ego passes the obstacle '
                         'along-track. 2.0 returned within ~9 cm with no oscillation offline.')
    ap.add_argument('--land-log', dest='land_log', type=float, default=4.0,
                    help='seconds to log ego/obstacle z + attitude THROUGH the land descent '
                         '(-> <out>_land.csv; verifies a single smooth ramp vs a double landing)')
    ap.add_argument('--no-vobs', dest='no_vobs', action='store_true', help='assume obstacle static (v_obs=0)')
    ap.add_argument('--bypass', action='store_true', help='disable CBF (ego flies straight -> collision-course control)')
    # right-hand bias (cures the exact-head-on deadlock; see right_bias())
    ap.add_argument('--bias-gain', dest='bias_gain', type=float, default=0.3,
                    help='peak lateral bias speed (m/s) at full head-on + close range')
    ap.add_argument('--bias-margin', dest='bias_margin', type=float, default=0.7,
                    help='proximity ramp width (m): bias starts at dist=R+margin, full at dist<=R')
    ap.add_argument('--no-bias', dest='no_bias', action='store_true',
                    help='disable the right-hand bias -> reproduces the head-on deadlock (control)')
    ap.add_argument('--ct-dist-gate', dest='ct_dist_gate', action='store_true',
                    help='distance-gate the line-follow cross-track return (circle scenario): rejoin the '
                         'axis whenever CLEAR of the obstacle, so the ego returns to centre BETWEEN the two '
                         'passes instead of staying pushed to one side. Default = along-track gate (head-on).')
    ap.add_argument('--pass-behind', dest='pass_behind', action='store_true',
                    help='use the crossing-avoidance bias (steer_behind): dodge to the side the moving '
                         'obstacle is LEAVING so the ego actively circumnavigates behind it, instead of '
                         'the fixed right-hand head-on bias. For the circle scenario (obstacle crosses '
                         'the ego path). Falls back to right-hand when the obstacle closes head-on.')
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
    if args.selftest_circle:
        _selftest_circle(args); return

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
    ego_s = np.array([node.start['cf1'][0], node.start['cf1'][1]])
    obs_s = np.array([node.start['cf2'][0], node.start['cf2'][1]])
    # cf1 ego goal. --ego-goal sets it EXPLICITLY (circle scenario: a diameter line through
    # the circle centre, independent of where the obstacle spawns). Otherwise the head-on
    # default: cf2 start extended --goal-beyond m PAST it along the approach line
    # (ego_start -> obstacle). goal_beyond=0 => goal at obstacle (swap target, sim default);
    # >0 => goal past the obstacle so a static obstacle no longer stalls the ego at the barrier.
    if args.ego_goal is not None:
        node.goal['cf1'] = np.array(args.ego_goal, dtype=float)
        step = node.goal['cf1'] - ego_s
        d = float(np.linalg.norm(step))
        u = step / d if d > 1e-6 else np.array([1.0, 0.0])   # ego heading = start -> goal
    else:
        approach = obs_s - ego_s
        d = float(np.linalg.norm(approach))
        u = approach / d if d > 1e-6 else np.array([1.0, 0.0])
        node.goal['cf1'] = obs_s + u * args.goal_beyond
    if args.line_follow:                             # approach-axis frame for _vdes_line
        node.axis_origin = ego_s
        node.axis_along = u
        node.axis_perp = np.array([-u[1], u[0]])     # u rotated +90deg
        node.s_goal = float((node.goal['cf1'] - ego_s) @ u)
    if args.obs_circle:
        # obstacle circles --obs-center; phase0 from its SPAWN so there's no jump at run start.
        # --obs-radius <=0 => derive from spawn (guarantees the spawn is exactly on the circle).
        C = np.array(args.obs_center, dtype=float)
        rel = obs_s - C
        if args.obs_phase0_deg is not None:
            node.obs_phi0 = math.radians(args.obs_phase0_deg)
            spawn_phi = math.degrees(math.atan2(rel[1], rel[0]))
            if abs((spawn_phi - args.obs_phase0_deg + 180) % 360 - 180) > 5.0:
                log.warn(f"--obs-phase0-deg {args.obs_phase0_deg:.1f} != spawn phase {spawn_phi:.1f}deg "
                         f"-> obstacle will JUMP to the circle at run start. Match the spawn to avoid it.")
        else:
            node.obs_phi0 = math.atan2(rel[1], rel[0])
        if args.obs_radius <= 0.0:
            args.obs_radius = float(np.linalg.norm(rel))
        node.goal['cf2'] = obs_s                      # unused in circle mode; keep non-None
        log.info(f"obstacle CIRCLE: center={tuple(C)} radius={args.obs_radius:.3f} "
                 f"omega={args.obs_omega:.4f} rad/s  phi0={math.degrees(node.obs_phi0):.1f}deg "
                 f"(period {2*math.pi/max(abs(args.obs_omega),1e-9):.1f}s)")
    else:
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
    # land-phase z trajectory (problem-2 verification) -> <out>_land.csv + inline summary
    if node.land_rows:
        land_out = args.out.replace('.csv', '_land.csv')
        with open(land_out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(node.land_rows[0].keys()))
            w.writeheader(); w.writerows(node.land_rows)
        zs = [r['z1'] for r in node.land_rows if r['z1'] != '']
        if zs:
            # count local minima that dip then rise >2 cm = a "double landing" bounce
            bounces = sum(1 for i in range(1, len(zs) - 1) if zs[i] < zs[i-1] - 0.005 and zs[i] < zs[i+1] - 0.02)
            print(f'LAND z trajectory     : start {zs[0]:.2f} -> min {min(zs):.2f} -> end {zs[-1]:.2f} m  '
                  f'({len(zs)} samples, {land_out})')
            print(f'  -> {"SINGLE smooth descent" if bounces == 0 else f"NON-MONOTONIC: {bounces} dip/re-descend (double-land?)"}'
                  f'; min z {min(zs):.3f} ({"below floor!" if min(zs) < -0.1 else "ok"})')
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
    if args.obs_circle:                                # two-encounter breakdown (circle scenario)
        enc = _find_encounters([r['t'] for r in pool], [r['sep_h'] for r in pool], thresh=args.R + 0.4)
        print(f'ENCOUNTERS (sep dips < R+0.4={args.R+0.4:.2f}): {len(enc)} '
              f'{"(want 2: lower + upper crossing)" if len(enc) != 2 else "-> two encounters as designed"}')
        for i, (te, se) in enumerate(enc, 1):
            print(f'  #{i}  t={te:6.2f}s   min-sep={se:.3f} m   barrier {"HELD" if se >= args.R-0.05 else "VIOLATED"}')
        # true sidestep = max perpendicular offset from the ego diameter axis (max|y| above is
        # meaningless for a vertical path — it just tracks goal progress along the axis).
        if node.axis_origin is not None:
            max_ct = max(abs(float((np.array([r['x1'], r['y1']]) - node.axis_origin) @ node.axis_perp))
                         for r in pool)
            print(f'max cross-track (sidestep off diameter axis): {max_ct:.3f} m')
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
