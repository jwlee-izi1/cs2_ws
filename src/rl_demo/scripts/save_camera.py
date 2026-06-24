#!/usr/bin/env python3
"""Capture current Gazebo GUI camera pose and save it for future runs.

Subscribes briefly to the gz topic /gui/camera/pose, reads one message,
converts the orientation quaternion to roll-pitch-yaw, and writes an SDF
<pose> string to the persisted file used by prepare_arena.py.

Run this while sim is up and the GUI camera is where you want it:
    ros2 run rl_demo save_camera.py
    # or
    python3 src/rl_demo/scripts/save_camera.py
"""

import math
import os
import subprocess
import sys
import re


SAVE_PATH = os.path.expanduser('~/cs2_ws/.gui_camera_pose.txt')


def parse_gz_pose_output(text):
    """Extract first complete pose (position + orientation) from `gz topic -e`
    output. Returns (px, py, pz, qx, qy, qz, qw) or None."""
    # Extract first matching position block
    pos_match = re.search(
        r'position\s*\{\s*x:\s*(-?[\d.e+-]+)\s*y:\s*(-?[\d.e+-]+)\s*z:\s*(-?[\d.e+-]+)\s*\}',
        text)
    ori_match = re.search(
        r'orientation\s*\{\s*x:\s*(-?[\d.e+-]+)\s*y:\s*(-?[\d.e+-]+)\s*z:\s*(-?[\d.e+-]+)\s*w:\s*(-?[\d.e+-]+)\s*\}',
        text)
    if not pos_match or not ori_match:
        return None
    return (float(pos_match.group(1)), float(pos_match.group(2)), float(pos_match.group(3)),
            float(ori_match.group(1)), float(ori_match.group(2)),
            float(ori_match.group(3)), float(ori_match.group(4)))


def quat_to_rpy(qx, qy, qz, qw):
    """Standard ZYX (roll-pitch-yaw) conversion."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def main():
    try:
        out = subprocess.check_output(
            ['gz', 'topic', '-e', '-t', '/gui/camera/pose', '--duration', '2'],
            stderr=subprocess.STDOUT, timeout=6,
        ).decode('utf-8', errors='replace')
    except subprocess.TimeoutExpired as e:
        out = (e.output or b'').decode('utf-8', errors='replace')
    except Exception as e:
        print(f'ERROR: gz topic failed: {e}', file=sys.stderr)
        sys.exit(1)

    parsed = parse_gz_pose_output(out)
    if not parsed:
        print('ERROR: could not parse any pose from /gui/camera/pose output. '
              'Is sim running?', file=sys.stderr)
        sys.exit(2)

    px, py, pz, qx, qy, qz, qw = parsed
    r, p, y = quat_to_rpy(qx, qy, qz, qw)
    pose_str = f'{px:.4f} {py:.4f} {pz:.4f} {r:.4f} {p:.4f} {y:.4f}'
    with open(SAVE_PATH, 'w') as f:
        f.write(pose_str + '\n')
    print(f'Saved GUI camera pose -> {SAVE_PATH}')
    print(f'  pose: {pose_str}')


if __name__ == '__main__':
    main()
