# coverage_optimizer_interfaces

Topic contract between optimizers (federated, baseline, future RL) and per-drone coverage planners for the CDC 2026 radial coverage demo.

## Contract

**Topic:** `/coverage/leash`
**Type:** `coverage_optimizer_interfaces/msg/Leash`
**QoS:** default (reliable, depth 10). Latest-only semantics: each planner cares only about the most recent leash.

## Message fields

```
std_msgs/Header header
string[]        drone_names    # order matches arena_4drone.yaml
float32[]       radii          # r_i^k in meters, index-aligned with drone_names
uint8           gate_state     # 0 = constraint-correcting, 1 = feasible
```

## Indexing

`drone_names[i]` ↔ `radii[i]`. Optimizers must populate both arrays in the order declared in `arena_4drone.yaml`'s `drone_names` list. Planners look up their own name by string match — they do not depend on positional ordering.

## gate_state semantics

- `0` (constraint-correcting): the optimizer is in a round where it detected $g(r^k) > 0$ (infeasible) and is correcting back into the feasible set. Planners may use this as a hint to expect retraction.
- `1` (feasible): the optimizer iterate satisfies $\sum c_i r_i^2 \le B$. Planners can sweep freely up to the leash.

The planner does not need to act differently on `gate_state` — it's informational. The leash radius is authoritative.

## Stamping

`header.stamp` = the wall-clock time the round was published. Planners may use this for timeout detection (e.g., declare optimizer dead after N seconds without a fresh leash).

## Implementations of this contract

- `fed_dcsa/radial_coverage_node.py` — FedDCSA optimizer (primary).
- `baseline_optimizers/constant_leash_node.py` — debug: fixed radii.
- `baseline_optimizers/sinusoidal_leash_node.py` — debug: sinusoidal radii for planner testing.
- (future) `rl_demo/...` — learned allocator.
