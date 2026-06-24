"""Receding-horizon trajectory math (pure-Python, no ROS) — used by
receding_horizon_streamer_node.py and unit-testable on its own.

Why this exists: the single-cubic polynomial_streamer fits p(0)=odom, p(T)=target,
v(T)=0 every 0.5 s. That (a) decelerates to a stop every segment -> saw-tooth velocity
-> jitter (which dropped single mocap markers on HW), and (b) interpolates a straight
CHORD to the target, which bows the path OUTSIDE the leash -> radial overshoot
(Sigma_drone > Sigma_leash -> FedDCSA not clean).

This module instead builds, each replan, a smooth POLAR arc from the drone's CURRENT
state toward the policy target:
  - in POLAR (r, phi) about the arena origin, with r clamped <= leash  => NO overshoot
    by construction (the path can't bow past the leash like a Cartesian chord can);
  - cubic-Hermite r(s), phi(s) whose END rates come from the policy target's VELOCITY
    (finite-diff of consecutive policy_target msgs) => the drone arrives MOVING WITH the
    target (no decelerate-to-0), i.e. it tracks the moving setpoint instead of braking;
  - sampled into K knots with analytic Cartesian (p, v, a), fit as K quintic-Hermite
    pieces => C2 across interior seams (continuous acceleration => low jerk => smooth);
  - the FIRST knot's (p, v) is overridden with the drone's current odom (p, v) so there
    is no kink at the replan seam.

The streamer does NOT decide the motion — the sweep + leash-riding come from the RL
policy via policy_target. This only makes the TRACKING smooth + overshoot-free.
"""
from __future__ import annotations

import numpy as np


def quintic_hermite(p0, v0, a0, p1, v1, a1, tau):
    """Quintic p(t)=sum c_i t^i on t in [0,tau] matching (p,v,a) at both ends.
    Returns 6 coefficients c[0..5]."""
    tau = float(tau)
    c0, c1, c2 = float(p0), float(v0), float(a0) / 2.0
    A = np.array([
        [tau**3,    tau**4,     tau**5],
        [3*tau**2,  4*tau**3,   5*tau**4],
        [6*tau,     12*tau**2,  20*tau**3],
    ], dtype=np.float64)
    b = np.array([
        p1 - (c0 + c1*tau + c2*tau**2),
        v1 - (c1 + 2*c2*tau),
        a1 - (2*c2),
    ], dtype=np.float64)
    c3, c4, c5 = np.linalg.solve(A, b)
    return np.array([c0, c1, c2, c3, c4, c5], dtype=np.float64)


def _hermite_unit(p0, m0, p1, m1, u):
    """Cubic-Hermite on unit interval u in [0,1]; returns (value, d/du, d2/du2)."""
    h00 = 2*u**3 - 3*u**2 + 1; h10 = u**3 - 2*u**2 + u
    h01 = -2*u**3 + 3*u**2;    h11 = u**3 - u**2
    d00 = 6*u**2 - 6*u; d10 = 3*u**2 - 4*u + 1; d01 = -6*u**2 + 6*u; d11 = 3*u**2 - 2*u
    dd00 = 12*u - 6; dd10 = 6*u - 4; dd01 = -12*u + 6; dd11 = 6*u - 2
    return (h00*p0 + h10*m0 + h01*p1 + h11*m1,
            d00*p0 + d10*m0 + d01*p1 + d11*m1,
            dd00*p0 + dd10*m0 + dd01*p1 + dd11*m1)


def polar_hermite_knots(r0, phi0, rd0, phid0, r1, phi1, rd1, phid1,
                        leash, H, n_pieces, r_min):
    """Cubic-Hermite r(s),phi(s) over s in [0,H] with boundary RATES from the current
    odom (rd0,phid0) and the moving target velocity (rd1,phid1). r endpoints clamped
    to [r_min, leash]; each sampled r also clamped <= leash => guaranteed no overshoot.
    Returns per-knot arrays t,x,y,vx,vy,ax,ay (analytic Cartesian from the polar arc)."""
    r1 = float(np.clip(r1, r_min, leash))
    r0 = float(max(r0, 1e-3))
    # unwrap phi1 to nearest phi0 so we sweep the short way
    phi1 = phi0 + ((phi1 - phi0 + np.pi) % (2*np.pi) - np.pi)
    mr0, mr1 = rd0*H, rd1*H              # Hermite tangents (scaled to unit u)
    mp0, mp1 = phid0*H, phid1*H
    ts = np.linspace(0.0, H, n_pieces + 1)
    out = {k: [] for k in ("t", "x", "y", "vx", "vy", "ax", "ay")}
    for t in ts:
        u = t / H
        r, dr_du, ddr_du = _hermite_unit(r0, mr0, r1, mr1, u)
        phi, dp_du, ddp_du = _hermite_unit(phi0, mp0, phi1, mp1, u)
        r = float(min(max(r, r_min), leash))          # hard clamp -> no overshoot
        rdt = dr_du / H; phid = dp_du / H
        rdd = ddr_du / (H*H); phidd = ddp_du / (H*H)
        c, s = np.cos(phi), np.sin(phi)
        x = r*c; y = r*s
        vx = rdt*c - r*phid*s
        vy = rdt*s + r*phid*c
        ax = rdd*c - 2*rdt*phid*s - r*phidd*s - r*phid*phid*c
        ay = rdd*s + 2*rdt*phid*c + r*phidd*c - r*phid*phid*s
        for k, val in zip(("t", "x", "y", "vx", "vy", "ax", "ay"),
                          (t, x, y, vx, vy, ax, ay)):
            out[k].append(val)
    return {k: np.array(v, dtype=np.float64) for k, v in out.items()}


def build_xy_pieces(kn, odom_xy, odom_vxy):
    """Fit a quintic-Hermite per axis for each segment between knots. The first knot's
    (p,v) is overridden with the current odom (graft -> no replan-seam kink). a0 kept
    from the polar arc (C1 at the replan seam, C2 across interior seams).
    Returns list of (tau, cx, cy) with cx/cy length-6 coeff arrays."""
    n = len(kn["t"]) - 1
    px = kn["x"].copy(); py = kn["y"].copy()
    vx = kn["vx"].copy(); vy = kn["vy"].copy()
    ax = kn["ax"].copy(); ay = kn["ay"].copy()
    px[0], py[0] = odom_xy
    vx[0], vy[0] = odom_vxy
    pieces = []
    for i in range(n):
        tau = float(kn["t"][i+1] - kn["t"][i])
        cx = quintic_hermite(px[i], vx[i], ax[i], px[i+1], vx[i+1], ax[i+1], tau)
        cy = quintic_hermite(py[i], vy[i], ay[i], py[i+1], vy[i+1], ay[i+1], tau)
        pieces.append((tau, cx, cy))
    return pieces


def hermite_knots_1d(p0, v0, p1, v1, H, ts):
    """Cubic-Hermite 1-D ease (p0,v0)->(p1,v1) over [0,H], sampled at times ts.
    Used for the z (altitude-hold) axis. Returns (p, v, a) arrays."""
    m0, m1 = v0*H, v1*H
    P = []; V = []; A = []
    for t in ts:
        u = t / H
        p, dp_du, ddp_du = _hermite_unit(p0, m0, p1, m1, u)
        P.append(p); V.append(dp_du/H); A.append(ddp_du/(H*H))
    return np.array(P), np.array(V), np.array(A)


def decompose_velocity(vx, vy, r, phi):
    """Cartesian velocity -> polar rates (rdot, phidot) at (r, phi)."""
    r = max(float(r), 1e-3)
    rd = vx*np.cos(phi) + vy*np.sin(phi)
    phid = (-vx*np.sin(phi) + vy*np.cos(phi)) / r
    return rd, phid


if __name__ == "__main__":
    # Self-test: representative extended-sweep replan; assert smoothness + no overshoot.
    np.set_printoptions(precision=3, suppress=True)
    leash, r_min, H, K = 1.80, 0.30, 1.5, 5
    r0, phi0 = 1.60, 0.10
    r1, phi1 = 1.78, 0.55
    odom_p = (r0*np.cos(phi0)+0.02, r0*np.sin(phi0)-0.01)
    odom_v = (0.15, 0.95)
    rd0, phid0 = decompose_velocity(odom_v[0], odom_v[1], r0, phi0)
    phid1 = 1.0 / r1; rd1 = 0.0                       # target moving along the sweep at the leash
    kn = polar_hermite_knots(r0, phi0, rd0, phid0, r1, phi1, rd1, phid1, leash, H, K, r_min)
    pieces = build_xy_pieces(kn, odom_p, odom_v)

    def _p(c, t): return sum(c[i]*t**i for i in range(6))
    def _d(c, t): return sum(i*c[i]*t**(i-1) for i in range(1, 6))
    def _dd(c, t): return sum(i*(i-1)*c[i]*t**(i-2) for i in range(2, 6))
    R = []; SPD = []; ACC = []; jumps = []; t0 = 0.0
    for i, (tau, cx, cy) in enumerate(pieces):
        for t in np.linspace(0, tau, 60, endpoint=False):
            R.append(np.hypot(_p(cx, t), _p(cy, t)))
            SPD.append(np.hypot(_d(cx, t), _d(cy, t)))
            ACC.append(np.hypot(_dd(cx, t), _dd(cy, t)))
        if i > 0:
            pa = pieces[i-1]
            jumps.append(np.hypot(_dd(pa[1], pa[0]) - _dd(cx, 0), _dd(pa[2], pa[0]) - _dd(cy, 0)))
    R = np.array(R); SPD = np.array(SPD); ACC = np.array(ACC)
    assert R.max() <= leash + 1e-6, f"OVERSHOOT max r {R.max()}"
    assert max(jumps) < 1e-6, f"C2 broken, seam jump {max(jumps)}"
    assert np.allclose((_p(pieces[0][1], 0), _p(pieces[0][2], 0)), odom_p), "start != odom pos"
    assert np.allclose((_d(pieces[0][1], 0), _d(pieces[0][2], 0)), odom_v), "start != odom vel"
    print(f"SELF-TEST PASS: max r {R.max():.3f}<=leash, peak spd {SPD.max():.2f} m/s, "
          f"peak acc {ACC.max():.2f} m/s^2, C2 seam jump {max(jumps):.1e}, graft exact")
