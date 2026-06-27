#!/usr/bin/env bash
# cs2_ws environment — source this once per run terminal:  source <cs2_ws>/env.sh
# (self-locating: works from any CWD and regardless of where the workspace is cloned)
#
# Bundles the two sources every run needs:
#   1. ROS 2 Jazzy underlay  (/opt/ros/jazzy)
#   2. this workspace overlay (install/setup.bash, built by colcon)
#
# DO NOT add this to ~/.bashrc or any global auto-source. Auto-sourcing ROS in every
# shell collides with the conda env installed later, and global env edits have broken
# this box before (the GPU-env / GNOME-login incident). Source it explicitly per terminal.
# GPU is never set globally either — only narrowly via the gz-gpu wrapper at sim-run time.

# Resolve workspace root from this file's own location (works regardless of CWD /
# symlinks). No hardcoded paths, so this stays portable across machines / clone dirs.
_CS2_WS="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"

# 1. ROS 2 Jazzy underlay
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "env.sh: WARNING /opt/ros/jazzy/setup.bash not found — is ROS 2 Jazzy installed?" >&2
fi

# 2. cs2_ws overlay
if [ -f "$_CS2_WS/install/setup.bash" ]; then
    source "$_CS2_WS/install/setup.bash"
else
    echo "env.sh: WARNING $_CS2_WS/install/setup.bash not found — run 'colcon build --symlink-install' first." >&2
fi

unset _CS2_WS
