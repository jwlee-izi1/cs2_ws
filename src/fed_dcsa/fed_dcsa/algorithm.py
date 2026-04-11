"""Fed-DCSA algorithm: Server-Coordinated Gate with Local Epochs.

Pure Python implementation (no ROS dependencies) of Algorithm 1 from the paper.
Specialized for the multi-drone surveillance scenario (Section VI) where each
station has a scalar decision variable x_i in [0, 1].

Constraint: g_i(x_i) = e_i * x_i - (E_i - E_thr)  [paper eq 17, affine]
Battery:    E_i^{k+1} = E_i^k - e_i * x_i^{k,T}   [paper eq 18, effort-proportional]
"""

import math
from dataclasses import dataclass, field


@dataclass
class StationState:
    """Per-station optimizer and physical state."""
    station_id: int
    x: float = 1.0              # decision variable (activity level) in [0, 1]
    battery: float = 500.0      # current battery Wh
    e_rate: float = 10.0        # energy cost per epoch at full activity
    q_weight: float = 1.0       # surveillance quality weight
    active_drone: str = ''
    standby_drone: str = ''


@dataclass
class RoundResult:
    """Summary of one algorithm round."""
    k: int
    b_k: int                    # gate decision (0 = infeasible, 1 = feasible)
    G_tilde: float              # aggregate constraint estimate
    eta_k: float                # tolerance
    r_drift: float              # drift budget
    gamma: float                # stepsize used
    x_before: list = field(default_factory=list)   # x_i at round start
    x_after: list = field(default_factory=list)     # x_i after T inner steps
    actions: list = field(default_factory=list)      # binary actions a_i
    batteries: list = field(default_factory=list)    # battery levels at round start
    swaps: list = field(default_factory=list)         # list of station_ids that swapped


class FedDCSA:
    """Fed-DCSA algorithm for the multi-drone surveillance experiment."""

    def __init__(self, K, T, tau, delta, E_max, E_thr, e_rates, q_weights,
                 eta_override=None, gamma_override=None, r_drift_override=None):
        """
        Args:
            K: total outer communication rounds
            T: inner local gradient steps per round
            tau: binary rounding threshold
            delta: probability parameter (Corollary 2)
            E_max: full battery capacity (Wh)
            E_thr: safety reserve threshold (Wh)
            e_rates: list of per-station energy costs
            q_weights: list of per-station surveillance weights
            eta_override: if set, use this tolerance instead of Corollary 2
            gamma_override: if set, use this stepsize instead of Corollary 2
            r_drift_override: if set, use this drift budget (0.0 for deterministic)
        """
        self.K = K
        self.T = T
        self.tau = tau
        self.delta = delta
        self.E_max = E_max
        self.E_thr = E_thr
        self.n_stations = len(e_rates)

        # Compute algorithm constants (per paper Section VII)
        # D_x = diameter of [0,1] = 1.0
        self.D_x = 1.0

        # M_bar = sqrt(sum_i max(q_i, e_i)^2) — worst-case a.s. subgradient norm
        self.M_bar = math.sqrt(
            sum(max(q, e) ** 2 for q, e in zip(q_weights, e_rates))
        )

        # L_G = sqrt(sum_i e_i^2) — Lipschitz constant of G
        self.L_G = math.sqrt(sum(e ** 2 for e in e_rates))

        # mu = 1.0 (Euclidean mirror map)
        self.mu = 1.0

        N = K * T

        # Stepsize: use override or Corollary 2 formula
        if gamma_override is not None:
            self.gamma = gamma_override
        else:
            self.gamma = self.D_x / (self.M_bar * math.sqrt(N)) if N > 0 else 0.01

        # Drift budget: use override or Corollary 2 formula
        if r_drift_override is not None:
            self.r_drift = r_drift_override
        else:
            self.r_drift = (2.0 * self.L_G * self.M_bar / self.mu) * T * self.gamma

        # Tolerance: use override or Corollary 2 formula
        if eta_override is not None:
            self.eta = eta_override
        else:
            self.eta = (4.0 * self.M_bar * self.D_x) / (delta * math.sqrt(N)) if N > 0 else 1.0

        # Weighted output tracking (paper equation 4)
        self.S_B = 0.0
        self.weighted_x_sum = [0.0] * self.n_stations

    def compute_constraint_report(self, station):
        """Compute local constraint report s_i = g_i(x_i).

        g_i(x_i) = e_i * x_i - (E_i - E_thr)

        Can be negative when drone has surplus battery, allowing budget
        sharing across the fleet (paper eq 17).
        """
        return station.e_rate * station.x - (station.battery - self.E_thr)

    def compute_gate(self, stations):
        """Server-side gate computation.

        Returns (b_k, G_tilde, eta_k, r_drift).
        """
        # Aggregate constraint reports
        G_tilde = sum(self.compute_constraint_report(s) for s in stations)

        # Gate decision (equation 2): deterministic reports so r_k = 0
        b_k = 1 if G_tilde <= self.eta - self.r_drift else 0

        return b_k, G_tilde, self.eta, self.r_drift

    def inner_steps(self, stations, b_k):
        """Run T projected gradient steps for all stations.

        Args:
            stations: list of StationState
            b_k: gate decision (0 or 1)

        Returns:
            List of x_i values at each inner step (for weighted output tracking).
        """
        inner_x_history = []  # list of lists: [step][station]

        for t in range(self.T):
            inner_x_history.append([s.x for s in stations])

            for s in stations:
                if b_k == 1:
                    # Feasible round: objective subgradient
                    # f_i(x_i) = -q_i * x_i, so df_i/dx_i = -q_i
                    h_i = -s.q_weight
                else:
                    # Infeasible round: constraint subgradient
                    # g_i(x_i) = e_i * x_i - (E_i - E_thr) is affine,
                    # so dg_i/dx_i = e_i unconditionally
                    h_i = s.e_rate

                # Projected gradient step onto [0, 1]
                s.x = max(0.0, min(1.0, s.x - self.gamma * h_i))

            # Track weighted output for feasible rounds
            if b_k == 1:
                for i, s in enumerate(stations):
                    self.weighted_x_sum[i] += self.gamma * s.x
                self.S_B += self.gamma

        return inner_x_history

    def apply_decision(self, stations):
        """Round x_i to binary action.

        Returns list of actions: 1 = stay active, 0 = swap.
        """
        return [1 if s.x >= self.tau else 0 for s in stations]

    def process_round_end(self, stations, actions):
        """Update state after execution (paper eq 18).

        For a_i = 1: E_i -= e_i * x_i^{k,T} (effort-proportional drain).
        For a_i = 0: swap — battery resets to E_max, x resets to 1.0.

        Returns list of station_ids that swapped.
        """
        swapped = []
        for i, (s, a) in enumerate(zip(stations, actions)):
            if a == 1:
                s.battery -= s.e_rate * s.x
            else:
                # Swap: standby takes over with full battery
                s.battery = self.E_max
                s.x = 1.0
                # Swap drone names
                s.active_drone, s.standby_drone = s.standby_drone, s.active_drone
                swapped.append(i)
        return swapped

    def run_round(self, k, stations):
        """Execute one complete algorithm round.

        Returns a RoundResult with all telemetry.
        """
        x_before = [s.x for s in stations]
        batteries = [s.battery for s in stations]

        # Step 1-2: Gate computation
        b_k, G_tilde, eta_k, r_drift = self.compute_gate(stations)

        # Step 3: T inner projected gradient steps
        self.inner_steps(stations, b_k)

        x_after = [s.x for s in stations]

        # Step 4: Binary rounding
        actions = self.apply_decision(stations)

        # Step 5: State update
        swaps = self.process_round_end(stations, actions)

        return RoundResult(
            k=k,
            b_k=b_k,
            G_tilde=G_tilde,
            eta_k=eta_k,
            r_drift=r_drift,
            gamma=self.gamma,
            x_before=x_before,
            x_after=x_after,
            actions=actions,
            batteries=batteries,
            swaps=swaps,
        )

    def get_weighted_output(self):
        """Return the feasible-phase weighted average x̄_η (paper equation 4)."""
        if self.S_B <= 0:
            return [0.0] * self.n_stations
        return [w / self.S_B for w in self.weighted_x_sum]
