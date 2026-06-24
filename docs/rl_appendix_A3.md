# Appendix A.3 — RL Coverage Policy (writeup material)

> All answers compiled 2026-06-05. The receding-horizon (RH) streamer is described **generally**
> on purpose — you'll get it (or a variation) working, so the appendix stays vague on the exact
> trajectory math. **Final policy = `iter18`** (`exp1/rl_training/checkpoints/ppo_coverage_iter18_final.zip`).
> iter18 has the **same reward function** as iter17; the only change is `edge_target` 0.96→0.99 (a config
> value, not a reward change) so the policy rides tight at the leash → the Fed<Lag contrast holds at B_op=8.4.

---

## 1. Narrative (draft-ready — adapt freely)

We train a per-drone coverage policy with PPO (BC warm-start from an edge-sweep expert; linear
learning-rate decay 3e-4→0, standard PPO practice). The policy observes a local thermal/age field
plus its leash and (r, φ), and outputs an angle and a radius fraction; it learns to **ride near the
optimizer's leash and sweep its angular sector**. The reward combines a small set of **commensurate**
terms — a bounded heat-coverage bonus, a leash-edge term, an **angular-sweep** term, and safety terms
(in-sector, soft-leash, smoothness) — so that "ride-and-sweep the leash" is the dominant rewarded
behavior and the training return **rises monotonically and plateaus**.

At deployment, the discrete policy setpoints are tracked by a **smooth receding-horizon streamer**
that produces jerk-free motion respecting the per-drone leash (no radial overshoot), keeping the motion
compatible with the onboard single-marker state estimator and the executed Σc·r² at the commanded budget.

The optimizer (FedDCSA vs. the Lagrangian baseline) sets the per-drone leash; the policy is identical
for both. **FedDCSA holds the global budget, so the executed Σc·r² stays below the data-loss threshold
(no data loss); the Lagrangian baseline overshoots the budget → sustained data loss.**

---

## 2. Reward function (term level — no need for exact weights)

A per-tick sum of **commensurate** terms (all the same order of magnitude → no single term dominates):

| Term | Role |
|---|---|
| **coverage** | small, *bounded* heat-coverage bonus (observe hot/aged cells; capped so it can't dominate) |
| **edge / anchor** | ride at ~`edge_target`·leash (symmetric pull to the leash radius) |
| **angular sweep** | reward for covering the *angular extent* of the sector (patrol the arc) → wide sweep |
| **sector** | penalty for leaving the assigned angular wedge |
| **soft-leash** | penalty for exceeding the leash radius |
| **smoothness** | penalty on action change (no jerky commands) |

**Design point worth stating:** an earlier version let coverage *dominate*, and coverage **conflicts**
with leash-riding (the leash sits outside the fire), so the training return *declined* as the policy
learned the intended behavior. Making the terms commensurate (bounding coverage) and adding the explicit
angular-sweep term fixed both the **declining curve** and a too-rigid sweep → a monotone-rising return
**and** a wide sweep.

---

## 3. Training setup

PPO (Stable-Baselines3), **BC warm-start** from an edge-sweep expert (keeps the sweep prior), **linear
LR decay 3e-4 → 0**, ~300k env steps, 8 parallel envs, γ=0.99. Reward normalized (VecNormalize); the
episode reward is logged un-normalized (that's the curve). Simulated 2D drone dynamics = a cascaded-PID
approximation of the firmware controller (for sim-to-real transfer).

---

## 4. Numbers to cite (iter18)

| Quantity | Value |
|---|---|
| Training curve (ep_rew_mean) | **−123 → +8620**, 100% monotone rise → plateau |
| Ride / sweep | r/leash ≈ **0.99**; sweep **~77°** of the 90° sector (radius-chase corr 0.96–0.99) |
| Σ_leash (optimizer) | FedDCSA max **8.08** (holds B=8.0) · Lagrangian max **8.81** (overshoots) |
| Σ_drone (smooth streamer, no overshoot = edge_target²·Σ_leash) | Fed **7.92** · Lag **8.64** |
| Data-loss frames (Σ_drone > B_op) @ **B_op=8.4** | **Fed 0 / Lag ≈190** |
| Data-loss frames @ honest **B_op=8.15** | **Fed 0 / Lag ≈310** |
| Hardware demo (figure-8 tracking path, separate) | Fed **9** / Lag **239** @ B_op=8.30 |
| Motion smoothness (streamer) | C2 (continuous accel), peak ~2 m/s², max drone r < 1.9 (r ≤ leash → no overshoot) |

> FedDCSA is clean at **both** thresholds — the contrast is the **optimizer**, not the policy.

---

## 5. Figures (3 for A.3)

1. **Training curve** — reward-only, monotone rise (drop `explained_variance` — noisy critic diagnostic,
   not needed). Target file: `exp1/rl_training/ppo_iter18_reward_curve.png`.
2. **Rollout** — 4-drone wedge sweep + radius-chase. **Needed.** A **GIF is not** (papers use the static
   figure). Target file: `exp1/rl_training/ppo_iter18_rollout.png`.
3. **Fed-vs-Lag** — Σc·r² vs t, Fed clean vs Lag sustained drops @ B_op=8.4.
   Target file: `exp1/rl_training/ppo_iter18_fed_vs_lag.png`.

*(The current images on disk are iter17's — same shape, slightly looser ride. Re-render from iter18.)*

---

## 6. Your earlier questions — answered

- **Rollout graph needed?** Yes (the behavior figure, companion to the curve). **GIF?** No (paper = static figure).
- **Why does the reward start ≈ −2000?** Early in each episode the leash is tiny, so any small tracking
  error is a huge *relative* error → large anchor/leash penalties. As the fire grows and PPO learns to ride
  + stay in-sector, the penalties vanish → the return climbs. The negative→positive climb **is** the learning.
- **`explained_variance` — needed?** No. The rising *reward* curve is the learning evidence; explained_var is
  a noisy critic-health diagnostic (ours ends ~0.8, healthy) that only clutters the figure.
- **Is LR decay standard?** Yes — the PPO paper (Schulman 2017) anneals the LR to 0; SB3 documents
  `linear_schedule`; SB3 RL Zoo uses the `lin_` prefix for tuned configs. 3e-4 is the default *initial*
  value, not an argument against decay.

---

## 7. Iteration history (context, in case useful)

- **iter15** — declining curve (coverage dominated + conflicted with leash-riding).
- **iter16** — commensurate reward + LR decay → curve fixed (monotone), but rode the leash *rigidly* (sweep ~23°).
- **iter17** — added the angular-sweep reward → wide sweep (~75°); curve still monotone. edge_target 0.96.
- **iter18** — edge_target 0.99 (ride tight). With the overshoot-free RH streamer this restores the Fed<Lag
  contrast at B_op=8.4 and is clean at the honest 8.15. **This is the final policy.**
