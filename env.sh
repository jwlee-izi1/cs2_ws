#!/usr/bin/env bash
# cs2_ws environment — source this once per run terminal:  source <cs2_ws>/env.sh
# (self-locating: works from any CWD and regardless of where the workspace is cloned)
#
# Works in BOTH bash and zsh. Each terminal's interactive shell may differ (this box's
# login shell is zsh / oh-my-zsh), so we pick the matching ROS/overlay setup files at
# runtime: setup.bash under bash, setup.zsh under zsh. Sourcing setup.bash *into* zsh
# breaks ROS's self-location (${BASH_SOURCE[0]} is empty in zsh -> ROS thinks its prefix
# is your CWD and tries to source ./setup.sh), which is exactly the failure this avoids.
#
# Bundles the three things every run needs:
#   1. ROS 2 Jazzy underlay  (/opt/ros/jazzy)
#   2. this workspace overlay (install/setup.*, built by colcon)
#   3. CS2_WS exported to the workspace root, so the doc commands ($CS2_WS/...) just work
#
# DO NOT add this to ~/.bashrc / ~/.zshrc or any global auto-source. Auto-sourcing ROS in
# every shell collides with the conda env installed later, and global env edits have broken
# this box before (the GPU-env / GNOME-login incident). Source it explicitly per terminal.
# GPU is never set globally either — only narrowly via the gz-gpu wrapper at sim-run time.

# --- pick shell + locate this file (zsh and bash differ) ---
if [ -n "${ZSH_VERSION:-}" ]; then
    _cs2_ext=zsh
    # zsh has no BASH_SOURCE; %x = path of the file currently being sourced.
    # eval hides the zsh-only syntax from bash's parser entirely.
    eval '_cs2_self="${(%):-%x}"'
else
    _cs2_ext=bash
    _cs2_self="${BASH_SOURCE[0]:-$0}"
fi

# Resolve workspace root from this file's own location (works regardless of CWD /
# symlinks). No hardcoded paths, so this stays portable across machines / clone dirs.
_CS2_WS="$(cd "$(dirname "$_cs2_self")" && pwd)"

# 1. ROS 2 Jazzy underlay
if [ -f "/opt/ros/jazzy/setup.$_cs2_ext" ]; then
    source "/opt/ros/jazzy/setup.$_cs2_ext"
else
    echo "env.sh: WARNING /opt/ros/jazzy/setup.$_cs2_ext not found — is ROS 2 Jazzy installed?" >&2
fi

# 2. cs2_ws overlay
if [ -f "$_CS2_WS/install/setup.$_cs2_ext" ]; then
    source "$_CS2_WS/install/setup.$_cs2_ext"
else
    echo "env.sh: WARNING $_CS2_WS/install/setup.$_cs2_ext not found — run 'colcon build --symlink-install' first." >&2
fi

# 3. Export CS2_WS so '$CS2_WS/...' in the docs resolves after a single 'source env.sh'.
export CS2_WS="$_CS2_WS"

unset _CS2_WS _cs2_self _cs2_ext
