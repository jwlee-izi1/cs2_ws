"""PayloadFedDCSA: Algorithm 1 adapted for payload tilt correction.

Pure Python implementation (no ROS dependencies).

Problem:
    min  F(δz) = Σ_i q_i · δz_i²                    (correction effort)
    s.t. G(δz) = Σ_i w_i · (z̄_i + δz_i - z*)² - τ_max ≤ 0  (tilt budget)

Gate (no r_drift in gate):
    b_k = 1{G̃_k ≤ η_k}
    η_k = c₂ / √(k+1)        (decaying tolerance)
    γ_k = c₁ / √(k+1)        (decaying stepsize)

Feasible rounds: projected gradient on objective (minimize effort).
Infeasible rounds: projected gradient on constraint (reduce tilt).
"""

import math
from dataclasses import dataclass, field


@dataclass
class DroneState:
    """Per-drone optimizer state."""
    drone_id: int
    z_bar: float = 0.0          # nominal z (from odom at startup)
    x_bar: float = 0.0          # nominal x (for absolute go_to)
    y_bar: float = 0.0          # nominal y (for absolute go_to)
    dz: float = 0.0             # decision variable δz_i ∈ [-δ_max, δ_max]
    q: float = 1.0              # objective weight
    w: float = 1.0              # constraint weight


@dataclass
class RoundResult:
    """Summary of one algorithm round."""
    k: int
    b_k: int                    # gate decision
    G_tilde: float              # aggregate constraint at round start
    eta_k: float                # tolerance η_k
    r_drift_k: float            # drift bound (logged, not used in gate)
    gamma_k: float              # stepsize used
    F_value: float              # objective F(δz) after inner steps
    dz_before: list = field(default_factory=list)
    dz_after: list = field(default_factory=list)


class PayloadFedDCSA:
    """FedDCSA for payload tilt correction."""

    def __init__(self, K, T, c1, c2, tau_max, delta_z_max, z_bars, q_weights, w_weights):
        self.K = K
        self.T = T
        self.c1 = c1
        self.c2 = c2
        self.tau_max = tau_max
        self.delta_z_max = delta_z_max
        self.n_drones = len(z_bars)

        self.z_star = sum(z_bars) / len(z_bars)

        # Problem constants for r_drift computation
        # f_i(δz_i) = q_i · δz_i²  →  ∂f_i/∂δz_i = 2·q_i·δz_i
        #   Lipschitz on [-δ_max, δ_max]: L_fi = 2·q_i·δ_max
        # g_i(δz_i) = w_i·(z̄_i + δz_i - z*)²  →  ∂g_i/∂δz_i = 2·w_i·(z̄_i + δz_i - z*)
        #   Lipschitz on [-δ_max, δ_max]: L_gi = 2·w_i·max(|z̄_i - z* ± δ_max|)
        L_fi_list = [2.0 * q * delta_z_max for q in q_weights]
        L_gi_list = [
            2.0 * w * max(abs(zb - self.z_star - delta_z_max),
                          abs(zb - self.z_star + delta_z_max))
            for w, zb in zip(w_weights, z_bars)
        ]

        # M̄ = sqrt(Σ max(L_fi, L_gi)²) — worst-case subgradient norm
        self.M_bar = math.sqrt(
            sum(max(lf, lg) ** 2 for lf, lg in zip(L_fi_list, L_gi_list))
        )

        # L_G = sqrt(Σ L_gi²) — Lipschitz constant of G
        self.L_G = math.sqrt(sum(lg ** 2 for lg in L_gi_list))

        # μ = 1 (Euclidean mirror map)
        self.mu = 1.0

        # D_x = diameter of feasible set = 2·δ_max (per drone)
        # Product set diameter: sqrt(Σ (2·δ_max)²) = 2·δ_max·√m
        self.D_x = 2.0 * delta_z_max * math.sqrt(self.n_drones)

        # Weighted output tracking (paper eq 4)
        self.S_B = 0.0
        self.weighted_dz_sum = [0.0] * self.n_drones

    def gamma(self, k):
        return self.c1 / math.sqrt(k + 1)

    def eta(self, k):
        return self.c2 / math.sqrt(k + 1)

    def r_drift(self, k):
        return (2.0 * self.L_G * self.M_bar / self.mu) * self.T * self.gamma(k)

    def compute_constraint_report(self, drone):
        """Local constraint g_i(δz_i) = w_i·(z̄_i + δz_i - z*)² - τ_max/m."""
        return drone.w * (drone.z_bar + drone.dz - self.z_star) ** 2 - self.tau_max / self.n_drones

    def compute_G(self, drones):
        """Aggregate constraint G̃ = Σ g_i."""
        return sum(self.compute_constraint_report(d) for d in drones)

    def compute_F(self, drones):
        """Objective F(δz) = Σ q_i · δz_i²."""
        return sum(d.q * d.dz ** 2 for d in drones)

    def compute_gate(self, k, drones):
        """Gate: b_k = 1 if G̃_k ≤ η_k."""
        G_tilde = self.compute_G(drones)
        eta_k = self.eta(k)
        b_k = 1 if G_tilde <= eta_k else 0
        return b_k, G_tilde, eta_k

    def inner_steps(self, k, drones, b_k):
        """Run T projected gradient steps."""
        gamma_k = self.gamma(k)

        for t in range(self.T):
            for d in drones:
                if b_k == 1:
                    # Feasible: objective subgradient ∂f_i/∂δz_i = 2·q_i·δz_i
                    h_i = 2.0 * d.q * d.dz
                else:
                    # Infeasible: constraint subgradient ∂g_i/∂δz_i = 2·w_i·(z̄_i + δz_i - z*)
                    h_i = 2.0 * d.w * (d.z_bar + d.dz - self.z_star)

                # Projected gradient step onto [-δ_max, δ_max]
                d.dz = max(-self.delta_z_max,
                           min(self.delta_z_max, d.dz - gamma_k * h_i))

            # Track weighted output for feasible rounds
            if b_k == 1:
                for i, d in enumerate(drones):
                    self.weighted_dz_sum[i] += gamma_k * d.dz
                self.S_B += gamma_k

    def run_round(self, k, drones):
        """Execute one complete algorithm round."""
        dz_before = [d.dz for d in drones]

        # Gate
        b_k, G_tilde, eta_k = self.compute_gate(k, drones)

        # Inner steps
        self.inner_steps(k, drones, b_k)

        dz_after = [d.dz for d in drones]
        F_value = self.compute_F(drones)

        return RoundResult(
            k=k,
            b_k=b_k,
            G_tilde=G_tilde,
            eta_k=eta_k,
            r_drift_k=self.r_drift(k),
            gamma_k=self.gamma(k),
            F_value=F_value,
            dz_before=dz_before,
            dz_after=dz_after,
        )

    def get_weighted_output(self):
        """Return the feasible-phase weighted average δz̄_η (paper eq 4)."""
        if self.S_B <= 0:
            return [0.0] * self.n_drones
        return [w / self.S_B for w in self.weighted_dz_sum]
