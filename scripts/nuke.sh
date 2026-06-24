#!/usr/bin/env bash
# nuke.sh — aggressive cleanup of ALL crazyswarm2/sim/thermal/coverage processes.
#
# When to use:
#   - After a crash (PC slowed down, processes left over)
#   - Before starting a new run, to guarantee a clean slate
#   - Whenever you're confused about what's running
#
# What it kills (best-effort, ignores errors):
#   - ros2 launch parents + their children
#   - crazyflie_server, motion_capture_tracking, teleop, joy_node
#   - thermal_sensor_node, thermal_mapper_node
#   - fed_dcsa nodes (coverage_planner, setpoint_streamer, radial_coverage, constant_leash, sinusoidal_leash)
#   - rviz2
#   - Gazebo (gz sim, gz server, gz gui)
#   - cf2-* docker containers
# Then:
#   - Restarts ros2 daemon (clears cached state)
#   - Verifies no survivors

set +e   # never abort on errors — this is best-effort cleanup

# ── ROS launches + nodes ──
# NOTE: pkill -f uses POSIX ERE; do NOT escape '|' for alternation (use '|' not '\|')
pkill -9 -f "ros2 launch"                                       2>/dev/null
pkill -9 -f "crazyflie_server"                                  2>/dev/null
pkill -9 -f "motion_capture_tracking"                           2>/dev/null
pkill -9 -f "thermal_sensor_node|thermal_mapper_node"           2>/dev/null
pkill -9 -f "teleop|joy_node"                                   2>/dev/null
pkill -9 -f "coverage_planner_node|setpoint_streamer_node|radial_coverage_node" 2>/dev/null
pkill -9 -f "constant_leash_node|sinusoidal_leash_node"         2>/dev/null
pkill -9 -f "rviz2"                                             2>/dev/null

# ── Gazebo ──
pkill -9 -f "gz sim|gz server|gz gui"                           2>/dev/null
pkill -9 -f "ruby.*gz.*sim"                                     2>/dev/null   # ruby launcher for gz

# ── Docker cf2 containers ──
cf2_containers=$(docker ps -a --filter "name=cf2-" -q 2>/dev/null)
if [[ -n "$cf2_containers" ]]; then
    docker rm -f $cf2_containers >/dev/null 2>&1
fi

# ── ROS daemon cache flush (clears stale node/topic state) ──
ros2 daemon stop  >/dev/null 2>&1
sleep 0.5
ros2 daemon start >/dev/null 2>&1

# ── Wait a beat for OS to finalize SIGKILL'd processes ──
sleep 1

# ── Survivor check (include teleop/joy_node — was missed in v1) ──
survivors=$(pgrep -af "crazyflie_server|motion_capture_tracking|thermal_sensor|thermal_mapper|coverage_planner|setpoint_streamer|radial_coverage|gz sim|gz server|gz gui|teleop|joy_node" 2>/dev/null \
    | grep -v "pgrep\|grep\|nuke\|shell-snapshot" | wc -l)
docker_survivors=$(docker ps -q --filter "name=cf2-" 2>/dev/null | wc -l)

if [[ "$survivors" -eq 0 && "$docker_survivors" -eq 0 ]]; then
    echo "[nuke] ✓ clean — no survivors"
    exit 0
else
    echo "[nuke] ⚠ $survivors process survivors + $docker_survivors docker survivors"
    pgrep -af "crazyflie_server|motion_capture_tracking|thermal_sensor|thermal_mapper|coverage_planner|setpoint_streamer|radial_coverage|gz sim|gz server|gz gui" 2>/dev/null \
        | grep -v "pgrep\|grep\|nuke\|shell-snapshot" | head -10
    docker ps --filter "name=cf2-" --format "  docker: {{.Names}}" 2>/dev/null
    exit 1
fi
