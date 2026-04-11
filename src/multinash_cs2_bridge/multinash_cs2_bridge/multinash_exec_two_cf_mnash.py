#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
import os
import json
import subprocess
import numpy as np

from builtin_interfaces.msg import Duration
from crazyflie_interfaces.srv import UploadTrajectory, StartTrajectory, Takeoff
from crazyflie_interfaces.msg import TrajectoryPolynomialPiece
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener

sys.path.append('/home/rk32226/DronePotentialGame')
from poly_helper import waypoints_to_trajectory

PYTHON_VENV = '/home/rk32226/cs2_ws/.venv_multinash/bin/python'
MULTINASH_SCRIPT = '/home/rk32226/DronePotentialGame/run_multinash_once.py'


class MultiNashExecTwoCF(Node):
    def __init__(self):
        super().__init__('multinash_exec_two_cf_mnash')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Service clients for cf1 and cf2
        # Service clients for cf1 and cf2
        self.cf1_takeoff_cli = self.create_client(Takeoff, '/cf1/takeoff')
        self.cf2_takeoff_cli = self.create_client(Takeoff, '/cf2/takeoff')
        self.cf1_upload_cli = self.create_client(UploadTrajectory, '/cf1/upload_trajectory')
        self.cf2_upload_cli = self.create_client(UploadTrajectory, '/cf2/upload_trajectory')
        self.cf1_start_cli  = self.create_client(StartTrajectory,  '/cf1/start_trajectory')
        self.cf2_start_cli  = self.create_client(StartTrajectory,  '/cf2/start_trajectory')

        self.get_logger().info("Waiting for /cf1/takeoff...")
        self.cf1_takeoff_cli.wait_for_service()
        self.get_logger().info("Waiting for /cf2/takeoff...")
        self.cf2_takeoff_cli.wait_for_service()
        self.get_logger().info("Waiting for /cf1/upload_trajectory...")
        self.cf1_upload_cli.wait_for_service()
        self.get_logger().info("Waiting for /cf2/upload_trajectory...")
        self.cf2_upload_cli.wait_for_service()
        self.get_logger().info("Waiting for /cf1/start_trajectory...")
        self.cf1_start_cli.wait_for_service()
        self.get_logger().info("Waiting for /cf2/start_trajectory...")
        self.cf2_start_cli.wait_for_service()
        self.get_logger().info("All services are available.")

        self.started = False
        self.ascent_complete = False
        self.cf1_uploaded = False
        self.cf2_uploaded = False
        self.timer = self.create_timer(1.0, self.timer_cb)

    def get_positions_world(self):
        now = rclpy.time.Time()
        tf_cf1 = self.tf_buffer.lookup_transform('world', 'cf1', now)
        tf_cf2 = self.tf_buffer.lookup_transform('world', 'cf2', now)
        p1 = tf_cf1.transform.translation
        p2 = tf_cf2.transform.translation
        pos = np.array([[p1.x, p1.y, p1.z],
                        [p2.x, p2.y, p2.z]], dtype=float)
        self.get_logger().info(f"Initial positions (world): cf1={pos[0]}, cf2={pos[1]}")
        return pos

    def timer_cb(self):
        if self.started:
            return
        self.started = True

        # 1) Get initial positions
        try:
            p = self.get_positions_world()  # shape (2,3)
        except Exception as e:
            self.get_logger().error(f"Failed to get TF positions: {e}")
            self.started = False
            return

        p1_0, p2_0 = p[0], p[1]

        # 2) Command both drones to ascend to z=1.0 before planning
        self.get_logger().info("Commanding drones to ascend to z=1.0...")
        self.ascend_to_height(p1_0, p2_0, target_z=1.0, duration_sec=3.0)

    def ascend_to_height(self, p1_0, p2_0, target_z=1.0, duration_sec=3.0):
        """Command both drones to ascend to target_z height."""
        # Store positions for later use in planning
        self.p1_0 = p1_0.copy()
        self.p2_0 = p2_0.copy()
        self.p1_0[2] = target_z
        self.p2_0[2] = target_z
        
        # Create Takeoff requests
        req1 = Takeoff.Request()
        req1.group_mask = 0
        req1.height = float(target_z)
        d1 = Duration()
        d1.sec = int(duration_sec)
        d1.nanosec = int((duration_sec - d1.sec) * 1e9)
        req1.duration = d1

        req2 = Takeoff.Request()
        req2.group_mask = 0
        req2.height = float(target_z)
        d2 = Duration()
        d2.sec = int(duration_sec)
        d2.nanosec = int((duration_sec - d2.sec) * 1e9)
        req2.duration = d2

        self.get_logger().info(f"Sending cf1 takeoff to height {target_z:.2f}")
        self.get_logger().info(f"Sending cf2 takeoff to height {target_z:.2f}")

        # Call services asynchronously
        self.cf1_takeoff_done = False
        self.cf2_takeoff_done = False
        self.cf1_takeoff_future = self.cf1_takeoff_cli.call_async(req1)
        self.cf1_takeoff_future.add_done_callback(self.on_cf1_takeoff_done)
        self.cf2_takeoff_future = self.cf2_takeoff_cli.call_async(req2)
        self.cf2_takeoff_future.add_done_callback(self.on_cf2_takeoff_done)

    def on_cf1_takeoff_done(self, future):
        try:
            result = future.result()
            self.get_logger().info("cf1 Takeoff command accepted.")
        except Exception as e:
            self.get_logger().error(f"cf1 Takeoff failed: {e}")
            return
        self.cf1_takeoff_done = True
        self.maybe_continue_after_ascent()

    def on_cf2_takeoff_done(self, future):
        try:
            result = future.result()
            self.get_logger().info("cf2 Takeoff command accepted.")
        except Exception as e:
            self.get_logger().error(f"cf2 Takeoff failed: {e}")
            return
        self.cf2_takeoff_done = True
        self.maybe_continue_after_ascent()

    def maybe_continue_after_ascent(self):
        """Continue with planning once both drones have reached target height."""
        if self.cf1_takeoff_done and self.cf2_takeoff_done and not self.ascent_complete:
            self.ascent_complete = True
            self.get_logger().info("Both drones have been commanded to takeoff. Waiting 4 seconds for ascent...")
            # Wait a bit for the drones to actually reach the target
            self.ascent_timer = self.create_timer(4.0, self.continue_after_ascent)

    def continue_after_ascent(self):
        """Continue with MultiNash planning after ascent is complete."""
        self.ascent_timer.cancel()  # Cancel the one-shot timer
        self.get_logger().info("Ascent complete. Proceeding with MultiNash planning...")
        p1_0 = self.p1_0
        p2_0 = self.p2_0

        # Build x0_all and goals_all
        x0_1 = np.zeros(8, dtype=float)
        x0_2 = np.zeros(8, dtype=float)
        # Start both cf1 and cf2 at z=1.0 (they should already be there)
        x0_1[0:3] = p1_0
        x0_2[0:3] = p2_0
        x0_1[3] = 0.0
        x0_1[4] = 0.0
        x0_1[5] = 0.5
        x0_2[3] = np.pi
        x0_2[4] = 0.0
        x0_2[5] = 0.5

        x0_all = np.stack([x0_1, x0_2], axis=0)
        # Goals: swap x,y positions but both at z=1.0
        goal_1 = p2_0.copy()
        goal_1[2] = 1.0  # cf1 goes to cf2's x,y position at z=1.0
        goal_2 = p1_0.copy()
        goal_2[2] = 1.0  # cf2 goes to cf1's x,y position at z=1.0
        goals_all = np.stack([goal_1, goal_2], axis=0)

        tau = 30
        dt  = 0.1

        self.get_logger().info("Calling MultiNash planner for both CFs...")
        try:
            positions_all, tau_eff, dt_eff = self.run_multinash_and_get_positions(
                x0_all, goals_all, tau=tau, dt=dt
            )
        except Exception as e:
            self.get_logger().error(f"MultiNash planning failed: {e}")
            return

        self.get_logger().info(f"MultiNash returned trajectories with tau={tau_eff}, dt={dt_eff}")

        positions_cf1 = positions_all[0, :, :]  # (K,3)
        positions_cf2 = positions_all[1, :, :]  # (K,3)
        K = positions_cf1.shape[0]
        t_array = np.arange(K, dtype=float) * dt_eff
        yaw_array = np.zeros(K, dtype=float)

        txyz_yaw_cf1 = np.column_stack([t_array,
                                        positions_cf1[:,0],
                                        positions_cf1[:,1],
                                        positions_cf1[:,2],
                                        yaw_array])
        txyz_yaw_cf2 = np.column_stack([t_array,
                                        positions_cf2[:,0],
                                        positions_cf2[:,1],
                                        positions_cf2[:,2],
                                        yaw_array])

        # Convert both sets of waypoints to Trajectory objects
        self.get_logger().info("Converting cf1 waypoints to polynomial trajectory...")
        traj1 = waypoints_to_trajectory(txyz_yaw_cf1, pieces=min(8, K-1))
        self.get_logger().info("Converting cf2 waypoints to polynomial trajectory...")
        traj2 = waypoints_to_trajectory(txyz_yaw_cf2, pieces=min(8, K-1))

        # Build TrajectoryPolynomialPiece lists
        pieces1 = self.traj_to_pieces(traj1)
        pieces2 = self.traj_to_pieces(traj2)

        # Upload to both CFs (async)
        upload1 = UploadTrajectory.Request()
        upload1.trajectory_id = 1
        upload1.piece_offset = 0
        upload1.pieces = pieces1

        upload2 = UploadTrajectory.Request()
        upload2.trajectory_id = 1
        upload2.piece_offset = 0
        upload2.pieces = pieces2

        self.get_logger().info("Uploading MultiNash-based trajectory to cf1 (async)...")
        self.cf1_uploaded = False
        self.cf2_uploaded = False
        self.cf1_upload_future = self.cf1_upload_cli.call_async(upload1)
        self.cf1_upload_future.add_done_callback(self.on_cf1_upload_done)

        self.get_logger().info("Uploading MultiNash-based trajectory to cf2 (async)...")
        self.cf2_upload_future = self.cf2_upload_cli.call_async(upload2)
        self.cf2_upload_future.add_done_callback(self.on_cf2_upload_done)

    def traj_to_pieces(self, traj):
        pieces = []
        for poly in traj.polynomials:
            piece = TrajectoryPolynomialPiece()
            d = Duration()
            d_sec = float(poly.duration)
            d.sec = int(d_sec)
            d.nanosec = int((d_sec - d.sec) * 1e9)
            piece.duration = d
            piece.poly_x = [float(c) for c in poly.px.p]
            piece.poly_y = [float(c) for c in poly.py.p]
            piece.poly_z = [float(c) for c in poly.pz.p]
            piece.poly_yaw = [float(c) for c in poly.pyaw.p]
            pieces.append(piece)
        return pieces

    def run_multinash_and_get_positions(self, x0_all, goals_all, tau, dt):
        import tempfile

        x0_all = np.asarray(x0_all, dtype=float)
        goals_all = np.asarray(goals_all, dtype=float)

        with tempfile.TemporaryDirectory() as tmpdir:
            input_path = os.path.join(tmpdir, 'input.json')
            output_path = os.path.join(tmpdir, 'output.json')

            data_in = {
                'tau': int(tau),
                'dt': float(dt),
                'x0_all': x0_all.tolist(),
                'goals_all': goals_all.tolist(),
            }
            with open(input_path, 'w') as f:
                json.dump(data_in, f)

            cmd = [PYTHON_VENV, MULTINASH_SCRIPT, input_path, output_path]
            self.get_logger().info(f"Running MultiNash planner: {' '.join(cmd)}")
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode != 0:
                raise RuntimeError(
                    f"run_multinash_once.py failed:\n"
                    f"  CMD: {' '.join(cmd)}\n"
                    f"  STDOUT:\n{result.stdout}\n"
                    f"  STDERR:\n{result.stderr}"
                )

            if not os.path.exists(output_path):
                raise RuntimeError("MultiNash planner did not produce output.json")

            with open(output_path, 'r') as f:
                data_out = json.load(f)

        positions = np.array(data_out['positions'], dtype=float)  # (2, tau_eff, 3)
        success = data_out.get('success', True)
        message = data_out.get('message', '')
        cost = data_out.get('cost', 0.0)

        if not success:
            self.get_logger().warn(f"MultiNash reported non-success: {message}")

        tau_eff = positions.shape[1]
        dt_eff = dt
        return positions, tau_eff, dt_eff

    def on_cf1_upload_done(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"cf1 UploadTrajectory failed: {e}")
            return
        self.get_logger().info("cf1 UploadTrajectory succeeded.")
        self.cf1_uploaded = True
        self.maybe_start_both()

    def on_cf2_upload_done(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"cf2 UploadTrajectory failed: {e}")
            return
        self.get_logger().info("cf2 UploadTrajectory succeeded.")
        self.cf2_uploaded = True
        self.maybe_start_both()

    def maybe_start_both(self):
        # Start both CFs only once both uploads have succeeded
        if self.cf1_uploaded and self.cf2_uploaded:
            self.get_logger().info("Both trajectories uploaded; starting both CFs...")

            start1 = StartTrajectory.Request()
            start1.group_mask = 0
            start1.trajectory_id = 1
            start1.timescale = 1.0
            start1.reversed = False
            start1.relative = False

            start2 = StartTrajectory.Request()
            start2.group_mask = 0
            start2.trajectory_id = 1
            start2.timescale = 1.0
            start2.reversed = False
            start2.relative = False

            self.cf1_start_future = self.cf1_start_cli.call_async(start1)
            self.cf1_start_future.add_done_callback(self.on_cf1_start_done)

            self.cf2_start_future = self.cf2_start_cli.call_async(start2)
            self.cf2_start_future.add_done_callback(self.on_cf2_start_done)

    def on_cf1_start_done(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"cf1 StartTrajectory failed: {e}")
            return
        self.get_logger().info("cf1 is now executing its MultiNash-based trajectory.")

    def on_cf2_start_done(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"cf2 StartTrajectory failed: {e}")
            return
        self.get_logger().info("cf2 is now executing its MultiNash-based trajectory.")


def main(args=None):
    rclpy.init(args=args)
    node = MultiNashExecTwoCF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
