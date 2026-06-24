#!/usr/bin/env bash
# coverage_restart.sh — one-command teardown + restart of the radial coverage demo.
#
# Kills any running sim / crazyswarm / coverage / rviz processes, brings up the
# CrazySim stack with sector-midpoint spawn (COVERAGE_4DRONE=1), and launches the
# coverage demo with the chosen optimizer.
#
# Usage:
#   ./scripts/coverage_restart.sh                       # default: fed_dcsa optimizer, figure8 planner
#   ./scripts/coverage_restart.sh constant              # mock constant-leash optimizer, figure8 planner
#   ./scripts/coverage_restart.sh fed_dcsa rl           # RL planner with fed_dcsa optimizer
#   ./scripts/coverage_restart.sh lagrangian rl         # RL planner with Lagrangian baseline (demo violation)
#   ./scripts/coverage_restart.sh fed_dcsa polar_lawnmower  # original polar lawnmower planner

set -e
OPTIMIZER="${1:-fed_dcsa}"
PLANNER_TYPE="${2:-figure8}"
WS="$(cd "$(dirname "$0")/.." && pwd)"
# Default RL checkpoint (locked iter 9 — slack=1.0, w_edge=100k). Override with
# 3rd positional arg, e.g.:
#   ./scripts/coverage_restart.sh fed_dcsa rl /abs/path/to/other_ckpt.zip
RL_CHECKPOINT="${3:-$WS/exp1/rl_training/checkpoints/ppo_coverage_iter11_final.zip}"

echo "[coverage_restart] === tearing down any prior run ==="
pkill -9 -f "ros2 launch|coverage_planner_node|setpoint_streamer_node|polynomial_streamer_node|radial_coverage_node|thermal_sensor_node|thermal_mapper_node|thermal_ground_truth_node|qi_estimator_node|constant_leash|sinusoidal_leash|crazyflie_server|gz sim|gz server|gz gui|rviz2|teleop|rl_planner_node|quadrant_figure8_node" 2>/dev/null || true
sleep 2
docker ps -a --filter "name=cf2-" -q | xargs -r docker rm -f 2>/dev/null || true
sleep 2

# Confirm clean
remaining=$(pgrep -af "coverage_planner|setpoint_streamer|radial_coverage|gz sim|crazyflie_server" 2>/dev/null | grep -v "shell-snapshot\|kworker\|thermald\|coverage_restart" | wc -l)
if [ "$remaining" -gt 0 ]; then
  echo "[coverage_restart] WARNING: $remaining processes still alive after pkill"
  pgrep -af "coverage_planner|setpoint_streamer|radial_coverage|gz sim|crazyflie_server" 2>/dev/null | grep -v "shell-snapshot\|kworker\|thermald\|coverage_restart" | head -5
fi

echo "[coverage_restart] === bringing up sim + crazyswarm2 (sector spawn) ==="
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}
rm -f /tmp/thermal_demo_up.log
# Spawn-radius selection by planner:
#   - RL: spawn near origin (0.05m) to match training (env spawns drones at (0,0)).
#     The trained policy walks them outward toward sector centerlines from origin.
#   - figure-8 / polar lawnmower: keep the figure-8-quadrant default of 0.42m.
if [ "$PLANNER_TYPE" = "rl" ]; then
  # RL planner expects sectors at cardinal directions (E/N/W/S, phi_mid =
  # 0/90/180/270°). Spawn each drone on its own sector centerline so it
  # doesn't have to traverse a neighbor's wedge before the policy starts.
  # 0.3 m gives inter-drone distance R*sqrt(2)=0.42m — safe from cf2-sitl
  # collision physics.
  export COVERAGE_SPAWN_R="${COVERAGE_SPAWN_R:-0.3}"
  export COVERAGE_SPAWN_LAYOUT="${COVERAGE_SPAWN_LAYOUT:-cardinal}"
fi
CFLIES_YAML="${CFLIES_YAML:-$WS/config/crazyflies_sitl_multi.yaml}" \
  COVERAGE_4DRONE=1 "$WS/scripts/thermal_demo.sh" up --no-thermal 2>&1 | tee /tmp/thermal_demo_up.log &
SIM_PID=$!

# Wait until sim is ready (or fails)
echo "[coverage_restart] waiting for sim to come up ..."
for i in $(seq 1 30); do
  if grep -q "stack is up" /tmp/thermal_demo_up.log 2>/dev/null; then
    echo "[coverage_restart] sim ready"
    break
  fi
  if grep -q "err:" /tmp/thermal_demo_up.log 2>/dev/null; then
    echo "[coverage_restart] sim bringup failed; check /tmp/thermal_demo_up.log"
    exit 1
  fi
  sleep 5
done

if ! grep -q "stack is up" /tmp/thermal_demo_up.log 2>/dev/null; then
  echo "[coverage_restart] sim didn't come up within 150s; aborting"
  exit 1
fi

echo "[coverage_restart] === launching coverage demo (optimizer=$OPTIMIZER, planner=$PLANNER_TYPE) ==="
# thermal_demo.sh up already issued /all/takeoff to 1.0 m → tell the launch
# file to SKIP its own takeoff (would otherwise issue a competing one to 0.6 m
# at a different time, which lets the streamer fight the takeoff command).
LAUNCH_ARGS=(optimizer:="$OPTIMIZER" planner_type:="$PLANNER_TYPE" enable_takeoff:=false)
if [ "$PLANNER_TYPE" = "rl" ]; then
  if [ ! -f "$RL_CHECKPOINT" ]; then
    echo "[coverage_restart] ERROR: RL checkpoint not found: $RL_CHECKPOINT"
    exit 1
  fi
  echo "[coverage_restart]   rl_checkpoint=$RL_CHECKPOINT"
  # Give takeoff at least 5 s of headroom before the policy starts publishing.
  LAUNCH_ARGS+=(rl_checkpoint:="$RL_CHECKPOINT" policy_warmup_sec:=5.0)
  rm -f /tmp/rl_planner_cf?.log
fi
exec ros2 launch cf_coverage_planner coverage_demo.launch.py "${LAUNCH_ARGS[@]}"
