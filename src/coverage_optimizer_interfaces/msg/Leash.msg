# Per-round leash output of any optimizer for the radial coverage demo.
# Published by an optimizer (fed_dcsa, baseline_optimizers, future RL allocator).
# Consumed by per-drone coverage planners.

std_msgs/Header header
string[]        drone_names    # Index into radii[]. Order matches arena_4drone.yaml.
float32[]       radii          # r_i^k in meters. Same length as drone_names.
uint8           gate_state     # 0 = constraint-correcting, 1 = feasible
