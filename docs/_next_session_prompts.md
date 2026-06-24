
# Next-session opening prompts

> Throwaway anchor. Delete once both sessions have been opened — they live in
> their own chats from then on.

Written 2026-05-27 at the end of the Phase B sim-verification chat. Two
remaining work streams can run **in parallel**:

- **A. RL policy (3-step chain across chats: design+impl → server training → drop-into-Gazebo eval)** — this prompt is for step 1 only.
- **C. Phase B-HW verification** — existing multi-drone HW chat picks this up.

The full multi-chat strategy is in
`~/.claude/plans/in-an-earlier-chat-snuggly-planet.md`. This file is just
the two opening prompts for the chats you're about to open.

---

## Prompt — RL policy work, step 1: design + local impl + handoff (NEW chat)

I want to start the RL policy work for the federated coverage project. The
Phase A offline substrate and Phase B live ROS plumbing are now locked — see
`docs/federated_coverage.md` (especially "Dynamic fire substrate" and "Offline
previewer + tooling") and the locked YAMLs at
`src/thermal_mapping/config/thermal_field.yaml` +
`src/cf_coverage_planner/config/arena_4drone.yaml`. The 2D source-of-truth
design lives at
`/home/rk32226/drone-rl-2d/multi_drone/experimental_setup_plan.md` — note:
the 2D version did NOT have the thermal sensor + shared-map staleness inputs
that 3D has; that's a real new design surface here.

The policy is a drop-in replacement for `coverage_planner_node.py` /
`polar_lawnmower.py` / `quadrant_figure8_node.py`. It consumes
`(local thermal sensor obs from /cfN/thermal/raw, r_i^k from /coverage/leash,
staleness from /thermal_map age_seconds layer)` and publishes
`/cfN/policy_target`. All surviving topic contracts in
`federated_coverage.md` are invariants.

**Open design questions for this chat to resolve before any code:**

1. **Training environment** — wrap the existing ROS+Gazebo stack (faithful,
   slow, ~30 steps/s) vs port the 2D pure-Python framework to 3D and wrap
   that (fast, vectorizable, sim2real gap to discover later) vs hybrid. The
   3D-specific sensor/map staleness inputs don't exist in the 2D framework,
   so either path needs new env code. (and other questions like just random thought like: "this is an extension of the training env but how are we making a env for this how do we confine the space or assigned region as part of the policy how do we generalise to any region you see what i am saying alot of un answered questions, so i want you to go over research/github/ similar coverage rl based policies or whatever and help me come up with something that works for my case and for my purposes like the specfic leash chasing behaviour and exploring and tracking balanced behaviour and staying close to the leash but not going over alot atleast you know what i am saying? ")
2. **Reward shape** — coverage of hot regions, leash compliance (`r_i^k`
   cap), staleness reduction, anti-collision, smooth motion. The figure-8
   baseline is the bar to beat in `r_i^k` chase fidelity.
3. **Action space** — direct setpoint `/cfN/policy_target` (continuous xy +
   altitude), or higher-level (target angle within sector + radial fraction
   of leash), or something else.
4. **Episode boundaries** — 6-min fire run = one episode? Mid-run resets?
   Single drone vs joint-action across all 4?

Discuss those four before deciding anything and discuss in depth and wait for my signal/permission before decdidng anything. Then design the env, write the
wrapper + training script in `src/rl_demo/`, do a SMALL in-chat sanity
training run to confirm plumbing.

The big training run happens on a GPU server (no chat). This chat's
deliverable is a **handoff doc** `docs/rl_policy_handoff.md` capturing:
chosen training env, env wrapper version + observation/action contract,
server training config (network, hparams, episode setup), checkpoint
location + scp-back procedure, and the eval invocation a separate eval chat
will use. Treat the handoff doc as load-bearing — it's how the eval chat
avoids mis-loading a stale checkpoint or running a wrapper that doesn't
match what training saw.

The 3rd link of the chain (separate chat, after server training) is: drop
the trained policy into `coverage_restart.sh` with `planner=RL` instead of
`figure8` and verify the chase behavior in ROS+Gazebo+RViz matches Phase A
predictions and performs better with much better and faster and accurate coverage with focus on the heat spots and chasing the leash and other behaviour just as expected to be better than the figure 8 we currenbtly have..

Discuss before deciding anything non-obvious. Don't burn cycles on long
training runs in this chat — those happen on the server, post-handoff.

---

## Prompt — Phase B-HW verification (EXISTING multi-drone HW chat)

The federated coverage sim now has the full dynamic-fire pipeline wired
(qi_estimator + thermal_ground_truth + last-write-wins thermal_mapper +
gated radial_coverage_optimizer + figure-8 planner with min_leash=0.4 and
figure8_speed=2.0). See `docs/federated_coverage.md` for the latest. Sim
trajectory reproduces the offline Phase A previewer to 2 decimal places.

Goal: add a **Phase B-HW** verification row to
`docs/hw_multi_drone_verification.md §7`. Re-run the Stage 4 procedure on
real drones with the new launch (`~/cs2_ws/scripts/coverage_restart.sh
fed_dcsa` already defaults to `planner_type:=figure8` and uses the new
qi_estimator + ground_truth nodes). Pre-flight checks unchanged
(single-marker yaw≈0 placement, channel 100, BVC enabled). Confirm the
dynamic-q chase produces the same KKT-faithful behavior on HW that it
does in sim (cf1 dominant downwind, cf3/cf4 squeezed below budget, gate
holding at 1 most of run, drones never leaving their quadrants).
