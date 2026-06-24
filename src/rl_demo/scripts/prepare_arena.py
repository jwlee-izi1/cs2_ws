#!/usr/bin/env python3
"""Rewrite rl_arena.sdf so obstacle and goal positions match an NPZ.

Usage:
    python3 prepare_arena.py /path/to/rollout.npz
        [--arena-sdf /path/to/rl_arena.sdf]  (default: installed rl_demo arena)
        [--out /tmp/rl_arena_current.sdf]

Effect:
- Reads NPZ's `obstacles` (5x2) and `goal_position` (2,).
- Parses the arena SDF.
- Sets <pose> of obstacle_1..obstacle_5 and goal_marker to match.
- Writes the result to --out (default: /tmp/rl_arena_current.sdf).

After this, set `GZ_SIM_RESOURCE_PATH`'s arena path via launch argument
or just point `rl_demo.launch.py` at the new world. See README.
"""

import argparse
import os
import re
import sys

import numpy as np

from ament_index_python.packages import get_package_share_directory


def replace_pose(sdf, model_name, x, y, z):
    """Replace <pose> inside a <model name="..."> block."""
    pattern = re.compile(
        r'(<model\s+name="' + re.escape(model_name) + r'">[\s\S]*?<pose>)'
        r'[^<]+'
        r'(</pose>)',
        re.MULTILINE,
    )
    replacement = rf'\g<1>{x:.4f} {y:.4f} {z:.4f} 0 0 0\g<2>'
    new_sdf, n = pattern.subn(replacement, sdf, count=1)
    if n != 1:
        raise RuntimeError(f'Could not locate <model name="{model_name}"> <pose>')
    return new_sdf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('npz', help='NPZ rollout file')
    ap.add_argument('--arena-sdf', default=None)
    ap.add_argument('--out', default='/tmp/rl_arena_current.sdf')
    ap.add_argument('--drone-yaml-out', default='/tmp/rl_drone_current.yaml',
                    help='Per-run single-drone YAML with spawn at NPZ start.')
    args = ap.parse_args()

    if args.arena_sdf is None:
        args.arena_sdf = os.path.join(
            get_package_share_directory('rl_demo'),
            'worlds', 'rl_arena.sdf')

    data = np.load(args.npz, allow_pickle=True)
    obstacles = np.asarray(data['obstacles'], dtype=float)
    goal = np.asarray(data['goal_position'], dtype=float)

    if obstacles.shape != (5, 2):
        print(f'WARNING: obstacles shape {obstacles.shape} != (5, 2). '
              f'Will place up to 5, leave rest at defaults.', file=sys.stderr)

    with open(args.arena_sdf, 'r') as f:
        sdf = f.read()

    # Obstacles: height 0.8 m, centered at z=0.4.
    for i, (x, y) in enumerate(obstacles[:5], start=1):
        sdf = replace_pose(sdf, f'obstacle_{i}', x, y, 0.4)

    # Goal marker: flat disc sitting on the ground (z=0.005).
    sdf = replace_pose(sdf, 'goal_marker', goal[0], goal[1], 0.005)

    # GUI camera pose: if user has saved one via save_camera.py, inject it.
    saved_cam = os.path.expanduser('~/cs2_ws/.gui_camera_pose.txt')
    if os.path.exists(saved_cam):
        with open(saved_cam, 'r') as f:
            pose_str = f.read().strip()
        if pose_str:
            # Replace the <gui><camera><pose>...</pose> block
            pattern = re.compile(
                r'(<gui\b[^>]*>\s*<camera\s+name="user_camera">\s*<pose>)'
                r'[^<]+(</pose>)',
                re.MULTILINE,
            )
            new_sdf, n = pattern.subn(rf'\g<1>{pose_str}\g<2>', sdf, count=1)
            if n == 1:
                sdf = new_sdf

    with open(args.out, 'w') as f:
        f.write(sdf)

    # Per-run drone YAML: spawn cf1 at the NPZ's trajectory start.
    start = data['positions'][0]
    drone_yaml = f"""fileversion: 3

robots:
  cf1:
    enabled: true
    type: cf21
    initial_position: [{float(start[0]):.4f}, {float(start[1]):.4f}, 0.03]

robot_types:
  cf21:
    motion_capture:
      tracking: "librigidbodytracker"
      marker: default_single_marker
      dynamics: default
    big_quad: false
    battery:
      voltage_warning: 3.8
      voltage_critical: 3.7

all:
  firmware_params:
    commander: {{ enHighLevel: 1 }}
    stabilizer: {{ estimator: 2, controller: 2 }}
"""
    with open(args.drone_yaml_out, 'w') as f:
        f.write(drone_yaml)

    print(f'Wrote {args.out}')
    print(f'Wrote {args.drone_yaml_out}')
    print(f'  obstacles: {obstacles.tolist()}')
    print(f'  goal:      {goal.tolist()}')
    print(f'  start:     {data["positions"][0].tolist()}')
    print(f'  outcome:   {data["outcome"]}')


if __name__ == '__main__':
    main()
