#!/usr/bin/env bash
# thermal_demo.sh — bring up / tear down the multi-drone thermal mapping demo.
#
# Usage:
#   thermal_demo.sh up        Start the full stack (T1 sim + T2 server + T3 takeoff + T4 mapping).
#   thermal_demo.sh down      Stop everything cleanly.
#   thermal_demo.sh status    Report what's running.
#   thermal_demo.sh restart   down + up.
#   thermal_demo.sh map       Test: subscribe to /thermal_map, print finite cells + range.
#   thermal_demo.sh check     Pre-flight: verify deps, env, file paths.
#
# Flags (for `up`):
#   --no-takeoff      Don't auto-takeoff after server is up.
#   --no-rviz         Don't launch rviz2 (headless).
#   --no-thermal      Just bring up sim + server, skip thermal nodes.
#   --num-drones N    Number of drones (default 4).
#
# Adapted from CRAZYSIM_MIGRATION.md §"How to run" — bundles the four-terminal
# sequence into one command with explicit waits and pre-flight checks.

# Don't use 'set -u' here: ROS's setup.bash references many optional env vars
# (AMENT_PREFIX_PATH, ROS_PACKAGE_PATH, etc.) and will exit silently under -u.
# Don't use 'set -e' either: we manage exit codes explicitly via `return` inside subroutines.

# ── paths ────────────────────────────────────────────────────────────────────
WS="${WS:-$HOME/cs2_ws}"
CONDA_PY="${CONDA_PY:-$HOME/miniconda3/envs/crazyflie/bin/python3}"
CFLIB_SRC="${CFLIB_SRC:-$WS/cflib-src}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
WS_SETUP="${WS_SETUP:-$WS/install/setup.bash}"
SITL_SCRIPT="$WS/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh"
CFLIES_YAML="${CFLIES_YAML:-$WS/config/crazyflies_sitl_multi.yaml}"
LOG_DIR="${LOG_DIR:-/tmp/thermal_demo}"

mkdir -p "$LOG_DIR"

# ── argument parsing ─────────────────────────────────────────────────────────
CMD="${1:-help}"; shift || true
NO_TAKEOFF=0; NO_RVIZ=0; NO_THERMAL=0; NUM_DRONES=4
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-takeoff) NO_TAKEOFF=1 ;;
    --no-rviz)    NO_RVIZ=1 ;;
    --no-thermal) NO_THERMAL=1 ;;
    --num-drones) NUM_DRONES="$2"; shift ;;
    *) echo "unknown flag: $1" >&2; exit 2 ;;
  esac
  shift
done

# ── helpers ──────────────────────────────────────────────────────────────────
log()  { echo "[thermal_demo] $*"; }
err()  { echo "[thermal_demo] ERROR: $*" >&2; }
note() { echo "[thermal_demo] note: $*"; }

source_ros() {
  if [[ ! -r "$ROS_SETUP" ]]; then
    err "ROS not found at $ROS_SETUP — is ros-jazzy-desktop installed?"; return 1
  fi
  # shellcheck source=/dev/null
  source "$ROS_SETUP"
  if [[ -f "$WS_SETUP" ]]; then
    # shellcheck source=/dev/null
    source "$WS_SETUP"
  else
    note "workspace not built yet ($WS_SETUP missing) — run 'colcon build' from $WS"
  fi
}

count_pids() { pgrep -f "$1" 2>/dev/null | wc -l; }

# ── pre-flight checks ────────────────────────────────────────────────────────
preflight() {
  local fail=0
  # 1. ROS2 reachable
  if [[ ! -f "$ROS_SETUP" ]]; then
    err "ROS2 setup.bash not at $ROS_SETUP"; fail=1
  fi
  # 2. Workspace built
  if [[ ! -f "$WS_SETUP" ]]; then
    note "workspace not built — '$WS' install/ missing"
  fi
  # 3. cflib import works in conda env
  if ! "$CONDA_PY" -c "import cflib" 2>/dev/null; then
    err "cflib not importable from $CONDA_PY"
    err "  fix:  cd $WS && git clone --depth=1 https://github.com/bitcraze/crazyflie-lib-python.git cflib-src"
    err "        cd cflib-src && $CONDA_PY -m pip uninstall -y cflib"
    err "        SETUPTOOLS_SCM_PRETEND_VERSION=0.1.31 $CONDA_PY -m pip install -e ."
    fail=1
  fi
  # 4. Docker accessible
  if ! docker ps >/dev/null 2>&1; then
    err "docker daemon unreachable for $USER"
    err "  fix:  sudo systemctl start docker"
    err "        sudo setfacl -m u:\$USER:rw /var/run/docker.sock   # non-persistent across docker restarts"
    fail=1
  fi
  # 5. SITL script + YAML present (we invoke via 'bash <script>', so +x is not required)
  [[ -r "$SITL_SCRIPT" ]] || { err "missing/unreadable: $SITL_SCRIPT"; fail=1; }
  [[ -f "$CFLIES_YAML" ]] || { err "missing: $CFLIES_YAML"; fail=1; }
  # 6. firmware_logging.enabled in the YAML (we need /cfN/odom)
  if ! grep -qE '^\s*enabled:\s*true' "$CFLIES_YAML" 2>/dev/null; then
    note "firmware_logging may be disabled in $CFLIES_YAML — /cfN/odom won't publish"
    note "  required block (under 'all:'):"
    note "    firmware_logging:"
    note "      enabled: true"
    note "      default_topics:"
    note "        odom:"
    note "          frequency: 20"
  fi
  # 7. cf2-sitl docker image present
  if ! docker image inspect cf2-sitl:22.04 >/dev/null 2>&1; then
    err "docker image cf2-sitl:22.04 not built (see CRAZYSIM_MIGRATION.md §'One-time setup' step 3)"
    fail=1
  fi
  return "$fail"
}

# ── status reporting ─────────────────────────────────────────────────────────
status() {
  echo "=== docker ==="
  docker ps --filter 'name=cf2-' --format '  {{.Names}} | {{.Status}}' 2>&1 | head -5 || echo "  (docker not accessible)"
  echo "=== gazebo + sitl scripts ==="
  pgrep -af "gz sim|sitl_multiagent" 2>/dev/null | grep -v grep | head -3 | sed 's/^/  /' || echo "  (none)"
  echo "=== crazyswarm2 ==="
  pgrep -af "crazyflie_server\.py|ros2 launch crazyflie " 2>/dev/null | grep -v grep | head -3 | sed 's/^/  /' || echo "  (none)"
  echo "=== thermal nodes ==="
  pgrep -af "thermal_(sensor|mapper)_node|ros2 launch thermal" 2>/dev/null | grep -v grep | head -8 | sed 's/^/  /' || echo "  (none)"
  echo "=== rviz ==="
  pgrep -af "/rviz2/rviz2.*thermal" 2>/dev/null | grep -v grep | head -3 | sed 's/^/  /' || echo "  (none)"
}

# ── teardown ─────────────────────────────────────────────────────────────────
down() {
  log "stopping thermal nodes + rviz..."
  pgrep -f 'thermal_(sensor|mapper)_node' | xargs -r kill -9 2>/dev/null
  pgrep -f 'ros2 launch thermal_mapping' | xargs -r kill -9 2>/dev/null
  pgrep -f '/rviz2/rviz2.*thermal' | xargs -r kill -9 2>/dev/null

  log "stopping crazyswarm2..."
  pgrep -f 'crazyflie_server\.py' | xargs -r kill -9 2>/dev/null
  pgrep -f 'ros2 launch crazyflie launch.py' | xargs -r kill -9 2>/dev/null
  pgrep -f '/lib/crazyflie/teleop' | xargs -r kill -9 2>/dev/null
  pgrep -f 'motion_capture_tracking_node' | xargs -r kill -9 2>/dev/null
  pgrep -f '/lib/joy/joy_node' | xargs -r kill -9 2>/dev/null

  log "stopping gazebo + cf2 containers..."
  pgrep -f 'gz sim' | xargs -r kill -9 2>/dev/null
  pgrep -f 'sitl_multiagent_square' | xargs -r kill -9 2>/dev/null
  for i in 0 1 2 3 4 5 6 7; do docker stop "cf2-$i" 2>/dev/null; done >/dev/null

  sleep 2
  log "down complete"
}

# ── waiters ──────────────────────────────────────────────────────────────────
wait_until() {  # wait_until DESC TIMEOUT_S CMD...
  local desc="$1" timeout="$2"; shift 2
  local end=$(( $(date +%s) + timeout ))
  while ! eval "$@" >/dev/null 2>&1; do
    if [[ $(date +%s) -ge $end ]]; then
      err "timed out waiting for: $desc (after ${timeout}s)"
      return 1
    fi
    sleep 1
  done
  log "ok: $desc"
}

# ── bringup ──────────────────────────────────────────────────────────────────
up() {
  log "preflight checks..."
  preflight || { err "preflight failed — fix errors above and retry"; return 1; }

  # Always nuke first — guarantees clean slate regardless of how previous
  # processes were started (script-managed OR direct ros2 launch).
  note "running nuke.sh for clean slate..."
  "$WS/scripts/nuke.sh" || true
  sleep 1

  source_ros || return 1

  # ── T1: Gazebo + cf2 containers ──
  log "[T1] starting Gazebo + $NUM_DRONES cf2 containers"
  ( cd "$WS/crazyflie-firmware" && bash "$SITL_SCRIPT" -n "$NUM_DRONES" -m crazyflie ) \
    >"$LOG_DIR/T1_sim.log" 2>&1 &
  T1_PID=$!
  wait_until "$NUM_DRONES cf2 containers up" 60 \
    "[ \"\$(docker ps --filter 'name=cf2-' -q | wc -l)\" = $NUM_DRONES ]" || return 1
  wait_until "$((NUM_DRONES * 4)) gz topics published" 60 \
    "[ \"\$(gz topic -l 2>/dev/null | grep -c '^/cf_')\" -ge $((NUM_DRONES * 4)) ]" || return 1

  # ── T2: crazyswarm2 ──
  log "[T2] starting crazyswarm2 server"
  ros2 launch crazyflie launch.py backend:=cflib \
      crazyflies_yaml_file:="$CFLIES_YAML" gui:=false \
      >"$LOG_DIR/T2_server.log" 2>&1 &
  T2_PID=$!
  wait_until "crazyswarm2 fully connected" 60 \
    "grep -q 'All Crazyflies are fully connected' '$LOG_DIR/T2_server.log'" || return 1

  # ── T3: takeoff ──
  if [[ "$NO_TAKEOFF" -eq 0 ]]; then
    log "[T3] sending /all/takeoff to height 1.0 m"
    timeout 10 ros2 service call /all/takeoff crazyflie_interfaces/srv/Takeoff \
        "{group_mask: 0, height: 1.0, duration: {sec: 3, nanosec: 0}}" \
        >"$LOG_DIR/T3_takeoff.log" 2>&1 || note "takeoff service call returned non-zero (drones may still have lifted)"
    sleep 4
  else
    note "[T3] skipped (--no-takeoff)"
  fi

  # ── T4: thermal mapping ──
  if [[ "$NO_THERMAL" -eq 0 ]]; then
    log "[T4] starting thermal mapping ($NUM_DRONES sensors + 1 mapper)"
    local launch_args="num_drones:=$NUM_DRONES"
    if [[ "$NO_RVIZ" -eq 1 ]]; then
      # rviz2 is unconditional in the launch file; spawn without DISPLAY to skip GUI gracefully
      DISPLAY="" ros2 launch thermal_mapping thermal_mapping_demo.launch.py $launch_args \
          >"$LOG_DIR/T4_thermal.log" 2>&1 &
    else
      DISPLAY="${DISPLAY:-:1}" ros2 launch thermal_mapping thermal_mapping_demo.launch.py $launch_args \
          >"$LOG_DIR/T4_thermal.log" 2>&1 &
    fi
    T4_PID=$!
    wait_until "thermal_mapper up" 30 \
      "grep -q 'thermal_mapper up:' '$LOG_DIR/T4_thermal.log'" || return 1
  else
    note "[T4] skipped (--no-thermal)"
  fi

  echo
  log "stack is up. logs in $LOG_DIR/"
  log "  T1: $LOG_DIR/T1_sim.log"
  log "  T2: $LOG_DIR/T2_server.log"
  [[ "$NO_TAKEOFF" -eq 0 ]] && log "  T3: $LOG_DIR/T3_takeoff.log"
  [[ "$NO_THERMAL" -eq 0 ]] && log "  T4: $LOG_DIR/T4_thermal.log"
  echo
  log "next steps — fly drones over hot spots:"
  cat <<'  EOF'
    source /opt/ros/jazzy/setup.bash && source ~/cs2_ws/install/setup.bash
    ros2 service call /cf1/go_to crazyflie_interfaces/srv/GoTo \
      "{group_mask: 0, relative: false, goal: {x: -1.0, y: -1.0, z: 1.0}, yaw: 0, duration: {sec: 4, nanosec: 0}}"
    ros2 service call /cf2/go_to crazyflie_interfaces/srv/GoTo \
      "{group_mask: 0, relative: false, goal: {x:  1.5, y: -0.5, z: 1.0}, yaw: 0, duration: {sec: 4, nanosec: 0}}"
    # check map convergence:
    thermal_demo.sh map
  EOF
}

# ── map probe ────────────────────────────────────────────────────────────────
map_probe() {
  source_ros || return 1
  "$CONDA_PY" -c "
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from grid_map_msgs.msg import GridMap

class P(Node):
    def __init__(s):
        super().__init__('map_probe')
        s.last = None
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        s.create_subscription(GridMap, '/thermal_map', s._cb, qos)
    def _cb(s, m): s.last = m

rclpy.init(); n = P()
end = n.get_clock().now().nanoseconds + int(4e9)
while n.get_clock().now().nanoseconds < end and n.last is None:
    rclpy.spin_once(n, timeout_sec=0.2)
if n.last is None:
    print('FAIL: no /thermal_map message'); raise SystemExit(1)
gm = n.last
nrows = round(gm.info.length_x / gm.info.resolution)
ncols = round(gm.info.length_y / gm.info.resolution)
arr = np.array(gm.data[gm.layers.index('thermal')].data, dtype=np.float32).reshape((nrows, ncols), order='F')
finite = int(np.isfinite(arr).sum())
nan = int(np.isnan(arr).sum())
print(f'/thermal_map: {finite} finite, {nan} NaN cells out of {nrows*ncols}')
if finite > 0:
    v = arr[np.isfinite(arr)]
    print(f'  range [{v.min():.2f}, {v.max():.2f}] degC, mean {v.mean():.2f} degC')
rclpy.shutdown()
"
}

# ── dispatch ─────────────────────────────────────────────────────────────────
case "$CMD" in
  up)       up ;;
  down)     down ;;
  restart)  down; up ;;
  status)   status ;;
  map)      map_probe ;;
  check)    source_ros || true; preflight && log "preflight ok" ;;
  help|"-h"|"--help"|*)
    sed -n '2,40p' "$0" | sed 's/^# \?//'
    ;;
esac
