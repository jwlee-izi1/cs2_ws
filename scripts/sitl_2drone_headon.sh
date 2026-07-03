#!/bin/bash
# Headless 2-drone CrazySim launcher for the BVC head-on / position-exchange test (Test 2).
#
# Spawns cf_id 0 -> cf1 at (-1, 0) and cf_id 1 -> cf2 at (+1, 0), facing each other along
# the x-axis, then starts the Gazebo server (`gz sim -s`). Each cf2 firmware runs in its own
# cf2-sitl:22.04 docker container, exactly like sitl_singleagent.sh / sitl_multiagent_square.sh.
#
#   (default)  headless: server only, no render window, no GPU. For automated/quantitative runs.
#   --gui      also opens the `gz sim -g` GUI client in THIS shell (so it inherits the GZ
#              resource/plugin paths and the models actually render). On this Optimus box the
#              GUI renders on the default Intel iGPU — no gz-gpu / RTX offload, no global GPU
#              env (the forbidden kind). Fine for 2 drones; keeps heat/fan low.
#
# Prereq: `source $CS2_WS/env.sh` first (puts `gz` on PATH). Run from any CWD.
# Teardown: Ctrl-C this script / close the GUI window (its EXIT trap stops the cf2-*
#           containers and gz server), or run scripts/sitl_2drone_headon.sh --down.
# (no `set -u`: setup_gz.bash references unbound GZ_SIM_* vars, like the upstream scripts.)

CS2_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FW_DIR="$CS2_WS/crazyflie-firmware"
GZ_LAUNCH="$FW_DIR/tools/crazyflie-simulation/simulator_files/gazebo"
build_path="$FW_DIR/sitl_make/build"
world="${WORLD:-crazysim_default}"
vehicle_model="${VEHICLE_MODEL:-crazyflie}"
export CF2_SIM_MODEL=gz_${vehicle_model}

# cf_id -> spawn (x, y).  index 0 = cf1, index 1 = cf2.
# Overridable via env (defaults reproduce the head-on Tests 2-5 layout unchanged). For the
# circle-obstacle scenario the ego (cf1) sits below the circle and the obstacle (cf2) starts
# on the orbit, e.g.:  CF1_X=0 CF1_Y=-1.5 CF2_X=-0.707 CF2_Y=-0.707 scripts/sitl_2drone_headon.sh --gui
SPAWN_X=(${CF1_X:--1.0}  ${CF2_X:-1.0})
SPAWN_Y=(${CF1_Y:-0.0}   ${CF2_Y:-0.0})
NUM=2

function cleanup() {
	echo "[sitl_2drone] cleanup: stopping cf2 containers + gz server"
	docker ps -q --filter "name=cf2-" 2>/dev/null | xargs -r docker stop 2>/dev/null
	pkill -x cf2 2>/dev/null
	pkill -9 ruby 2>/dev/null   # gz server is a ruby process
}

if [ "${1:-}" == "--down" ]; then
	cleanup
	exit 0
fi

GUI=0
[ "${1:-}" == "--gui" ] && GUI=1

if ! command -v gz >/dev/null 2>&1; then
	echo "[sitl_2drone] ERROR: 'gz' not on PATH — run 'source $CS2_WS/env.sh' first." >&2
	exit 1
fi

function spawn_model() {
	local N=$1 X=$2 Y=$3
	local working_dir="$build_path/$N"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"
	pushd "$working_dir" &>/dev/null

	python3 "$GZ_LAUNCH/launch/jinja_gen.py" \
		"$GZ_LAUNCH/models/${vehicle_model}/model.sdf.jinja" \
		"$GZ_LAUNCH" \
		--cffirm_udp_port $((19950+N)) \
		--cflib_udp_port $((19850+N)) \
		--cf_id $((N)) \
		--cf_name cf \
		--output-file /tmp/${vehicle_model}_${N}.sdf

	echo "[sitl_2drone] spawning ${vehicle_model}_${N} (cf$((N+1))) at ${X} ${Y}"
	gz service -s /world/${world}/create \
		--reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 300 \
		--req 'sdf_filename: "/tmp/'${vehicle_model}_${N}'.sdf", pose: {position: {x:'${X}', y:'${Y}', z: 0.5}}, name: "'${vehicle_model}_${N}'", allow_renaming: 1'

	docker rm -f cf2-${N} 2>/dev/null
	docker run -d --rm --network=host --name cf2-${N} \
		cf2-sitl:22.04 $((19950+N)) 127.0.0.1 \
		> "$working_dir/out.log" 2> "$working_dir/error.log"
	popd &>/dev/null
}

echo "[sitl_2drone] killing any stale cf2 firmware instances"
pkill -x cf2 2>/dev/null || true
sleep 1

source "$GZ_LAUNCH/launch/setup_gz.bash" "$FW_DIR" "$build_path"

echo "[sitl_2drone] starting Gazebo server (gz sim -s)$([ "$GUI" == "1" ] && echo ' + GUI client to follow' || echo ', headless')"
gz sim -s -r "$GZ_LAUNCH/worlds/${world}.sdf" -v 3 &
sleep 3

for ((n=0; n<NUM; n++)); do
	spawn_model "$n" "${SPAWN_X[$n]}" "${SPAWN_Y[$n]}"
done

trap "cleanup" SIGINT SIGTERM EXIT

if [ "$GUI" == "1" ]; then
	# GZ_GUI_WRAP lets the GUI client be wrapped (e.g. `gz-gpu` for RTX PRIME offload on this
	# Optimus box) WITHOUT a global GPU env. Default empty -> plain `gz sim -g` on the Intel
	# iGPU, unchanged from the BVC test. For a clean recording run:  GZ_GUI_WRAP=gz-gpu --gui.
	_REND="Intel iGPU"; [ -n "${GZ_GUI_WRAP:-}" ] && _REND="via '${GZ_GUI_WRAP}' (RTX offload)"
	echo "[sitl_2drone] up. cf1@(-1,0) cf2@(+1,0). Opening GUI client (gz sim -g, ${_REND})..."
	echo "[sitl_2drone] close the window (or Ctrl-C) to tear everything down."
	${GZ_GUI_WRAP:-} gz sim -g
else
	echo "[sitl_2drone] up. cf1@(-1,0) cf2@(+1,0), headless. Ctrl-C to tear down."
	echo "[sitl_2drone] (server PID(ruby) running; this script now waits.)"
	wait
fi
