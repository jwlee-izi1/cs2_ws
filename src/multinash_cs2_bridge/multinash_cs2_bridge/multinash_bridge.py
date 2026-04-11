#!/usr/bin/env python3
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import json
import subprocess
import os
import tempfile

from tf2_ros import Buffer, TransformListener  # TF listener

class MultiNashBridge(Node):
    def __init__(self):
        super().__init__('multinash_bridge')

        # --- Backend parameter (so future you can swap modes) ---
        self.backend = self.declare_parameter('backend', 'cs2_sim').get_parameter_value().string_value
        self.get_logger().info(f"MultiNashBridge backend: {self.backend}")

        # --- TF2 buffer + listener (used in SIM and can be reused for HW/Gazebo) ---
        # Buffer() in Jazzy takes optional cache_time and node; here we create it plain
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Publishers for planned paths (for RViz) ---
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.cf1_path_pub = self.create_publisher(Path, '/cf1/planned_path', qos)
        self.cf2_path_pub = self.create_publisher(Path, '/cf2/planned_path', qos)

        self.planned = False
        self.timer = self.create_timer(1.0, self.timer_cb)

        # --- Paths to planner script and venv python ---
        self.planner_script = '/home/rk32226/DronePotentialGame/run_multinash_once.py'
        self.venv_python   = '/home/rk32226/cs2_ws/.venv_multinash/bin/python'

    # ---------- Backend-agnostic "position fetcher" ----------
    def get_positions_world(self):
        """
        Return positions as numpy array shape (2,3) for cf1 and cf2 in 'world' frame.
        For now, we only implement the cs2_sim backend using TF.
        Later you can add 'gazebo' or 'cs2_hw' branches.
        """
        if self.backend == 'cs2_sim':
            now = rclpy.time.Time()  # latest
            # lookup_transform(target_frame, source_frame, time)
            tf_cf1 = self.tf_buffer.lookup_transform('world', 'cf1', now)
            tf_cf2 = self.tf_buffer.lookup_transform('world', 'cf2', now)
            p1 = tf_cf1.transform.translation
            p2 = tf_cf2.transform.translation
            pos = np.array([[p1.x, p1.y, p1.z],
                            [p2.x, p2.y, p2.z]])
            # sanity check: log positions
            self.get_logger().info(f"Positions (world): cf1={pos[0]}, cf2={pos[1]}")
            return pos

        # Future backends:
        # elif self.backend == 'gazebo':
        #   use odom or TF: world->crazyflie
        # elif self.backend == 'cs2_hw':
        #   use TF from mocap or /cfX/pose logging

        raise RuntimeError(f"Unsupported backend '{self.backend}' in get_positions_world()")

    # ---------- Main timer callback ----------
    def timer_cb(self):
        if self.planned:
            return

        # Try to get positions from the backend-specific function
        try:
            positions_world = self.get_positions_world()  # shape (2,3)
        except Exception as e:
            self.get_logger().info(f"Waiting for positions: {e}")
            return

        self.planned = True
        self.get_logger().info("Got positions; running external MultiNash planner...")

        # Build x0_all in planner's [p,q,r,theta,phi,v,omega_theta,omega_phi] format
        x0_all = []
        for i, pos in enumerate(positions_world):
            yaw_init = 0.0 if i == 0 else np.pi  # just a guess; planner uses its own dynamics
            x0 = [0.0] * 8
            x0[0] = float(pos[0])
            x0[1] = float(pos[1])
            x0[2] = float(pos[2])
            x0[3] = float(yaw_init)
            x0[4] = 0.0
            x0[5] = 0.5
            # omega_theta, omega_phi = 0
            x0_all.append(x0)

        # Naive goals: swap positions
        cf1_p = x0_all[0][0:3]
        cf2_p = x0_all[1][0:3]
        cf1_goal = list(cf2_p)
        cf1_goal[2] = 2.5
        cf2_goal = list(cf1_p)
        cf2_goal[2] = 2.5
        goals_all = [cf1_goal, cf2_goal]

        tau = 30
        dt  = 0.1

        # Write input.json, call planner, read output.json
        with tempfile.TemporaryDirectory() as tmpdir:
            input_path  = os.path.join(tmpdir, 'input.json')
            output_path = os.path.join(tmpdir, 'output.json')

            data_in = {
                'tau': tau,
                'dt': dt,
                'x0_all': x0_all,
                'goals_all': goals_all,
            }
            with open(input_path, 'w') as f:
                json.dump(data_in, f)

            cmd = [self.venv_python, self.planner_script, input_path, output_path]
            self.get_logger().info(f"Calling planner: {' '.join(cmd)}")
            res = subprocess.run(cmd, capture_output=True, text=True)
            if res.returncode != 0:
                self.get_logger().error(f"Planner failed: {res.stderr}")
                return

            if not os.path.exists(output_path):
                self.get_logger().error("Planner did not produce output.json")
                return

            with open(output_path, 'r') as f:
                data_out = json.load(f)

        if not data_out.get('success', True):
            self.get_logger().warn(
                f"Planner reported non-success: {data_out.get('message','')}"
            )
        else:
            self.get_logger().info(f"Planner success, cost={data_out.get('cost', 0.0)}")

        positions = np.array(data_out['positions'])  # shape (2, tau, 3)
        self.publish_paths(positions)

    def publish_paths(self, positions):
        """Publish planned paths for cf1 and cf2 as nav_msgs/Path in 'world' frame."""
        frame = 'world'

        path1 = Path()
        path2 = Path()
        path1.header.frame_id = frame
        path2.header.frame_id = frame

        tau = positions.shape[1]
        for k in range(tau):
            # cf1
            ps1 = PoseStamped()
            ps1.header.frame_id = frame
            ps1.pose.position.x = float(positions[0, k, 0])
            ps1.pose.position.y = float(positions[0, k, 1])
            ps1.pose.position.z = float(positions[0, k, 2])
            path1.poses.append(ps1)

            # cf2
            ps2 = PoseStamped()
            ps2.header.frame_id = frame
            ps2.pose.position.x = float(positions[1, k, 0])
            ps2.pose.position.y = float(positions[1, k, 1])
            ps2.pose.position.z = float(positions[1, k, 2])
            path2.poses.append(ps2)

        self.cf1_path_pub.publish(path1)
        self.cf2_path_pub.publish(path2)
        self.get_logger().info("Published planned paths for cf1 and cf2.")

def main(args=None):
    rclpy.init(args=args)
    node = MultiNashBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

