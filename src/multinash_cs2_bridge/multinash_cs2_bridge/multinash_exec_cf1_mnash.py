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

# Import the helper from your planning repo (waypoints -> Trajectory)
sys.path.append('/home/rk32226/DronePotentialGame')
from poly_helper import waypoints_to_trajectory

# Paths for venv Python and MultiNash script
PYTHON_VENV = '/home/rk32226/cs2_ws/.venv_multinash/bin/python'
MULTINASH_SCRIPT = '/home/rk32226/DronePotentialGame/run_multinash_once.py'


class MultiNashExecCF1MNash(Node):
    def __init__(self):
        super().__init__('multinash_exec_cf1_mnash')

        # TF buffer and listener to get world->cf1/cf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Service clients for cf1 and cf2
        self.cf1_takeoff_cli = self.create_client(Takeoff, '/cf1/takeoff')
        self.cf2_takeoff_cli = self.create_client(Takeoff, '/cf2/takeoff')
        self.upload_cli = self.create_client(UploadTrajectory, '/cf1/upload_trajectory')
        self.start_cli  = self.create_client(StartTrajectory,  '/cf1/start_trajectory')

        self.get_logger().info("Waiting for /cf1/takeoff...")
        self.cf1_takeoff_cli.wait_for_service()
        self.get_logger().info("Waiting for /cf2/takeoff...")
        self.cf2_takeoff_cli.wait_for_service()
        self.get_logger().info("Waiting for /cf1/upload_trajectory...")
        self.upload_cli.wait_for_service()
        self.get_logger().info("Waiting for /cf1/start_trajectory...")
        self.start_cli.wait_for_service()
        self.get_logger().info("Services are available.")

        self.started = False
        self.ascent_complete = False
        self.timer = self.create_timer(1.0, self.timer_cb)

    def get_positions_world(self):
        """Return positions cf1, cf2 in world frame as (2,3) numpy array."""
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

        # 1) Get initial positions from TF
        try:
            positions_world = self.get_positions_world()  # shape (2,3)
        except Exception as e:
            self.get_logger().error(f"Failed to get TF positions: {e}")
            self.started = False
            return

        # Unpack cf1 and cf2 initial positions
        p1_0 = positions_world[0]
        p2_0 = positions_world[1]

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

        # Build x0_all and goals_all for MultiNash
        # State: [p,q,r,theta,phi,v,omega_theta,omega_phi]
        x0_1 = np.zeros(8, dtype=float)
        x0_2 = np.zeros(8, dtype=float)

        # Start both at z=1.0 (they should already be there)
        x0_1[0:3] = p1_0
        x0_2[0:3] = p2_0

        # Simple initial yaw/speed guesses
        x0_1[3] = 0.0        # yaw
        x0_1[4] = 0.0        # pitch
        x0_1[5] = 0.5        # speed
        x0_2[3] = np.pi      # yaw
        x0_2[4] = 0.0
        x0_2[5] = 0.5

        x0_all = np.stack([x0_1, x0_2], axis=0)   # shape (2,8)

        # Goals: swap x,y positions but both at z=1.0
        goal_1 = p2_0.copy()
        goal_1[2] = 1.0  # cf1 goes to cf2's x,y position at z=1.0
        goal_2 = p1_0.copy()
        goal_2[2] = 1.0  # cf2 goes to cf1's x,y position at z=1.0
        goals_all = np.stack([goal_1, goal_2], axis=0)  # shape (2,3)

        # Planner hyperparameters
        tau = 30
        dt  = 0.1

        self.get_logger().info("Calling MultiNash planner via run_multinash_once.py...")
        try:
            positions_cf1, tau_eff, dt_eff = self.run_multinash_and_get_cf1_positions(
                x0_all, goals_all, tau=tau, dt=dt
            )
        except Exception as e:
            self.get_logger().error(f"MultiNash planning failed: {e}")
            return

        self.get_logger().info(f"MultiNash returned cf1 trajectory with tau={tau_eff}, dt={dt_eff}")

        # Build timed waypoints for cf1: [t,x,y,z,yaw]
        K = positions_cf1.shape[0]
        t_array = np.arange(K, dtype=float) * dt_eff
        yaw_array = np.zeros(K, dtype=float)  # simple yaw=0 for now
        txyz_yaw = np.column_stack([t_array,
                                    positions_cf1[:, 0],
                                    positions_cf1[:, 1],
                                    positions_cf1[:, 2],
                                    yaw_array])

        # Convert to polynomial Trajectory
        self.get_logger().info("Converting MultiNash waypoints to polynomial trajectory...")
        traj = waypoints_to_trajectory(txyz_yaw, pieces=min(8, K - 1))
        self.get_logger().info(f"Trajectory duration: {traj.duration}, segments: {len(traj.polynomials)}")

        # Convert to TrajectoryPolynomialPiece[] and upload/start
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

        upload_req = UploadTrajectory.Request()
        upload_req.trajectory_id = 1
        upload_req.piece_offset = 0
        upload_req.pieces = pieces

        self.get_logger().info("Uploading MultiNash-based trajectory to cf1 (async)...")
        self.upload_future = self.upload_cli.call_async(upload_req)
        self.upload_future.add_done_callback(self.on_upload_done)

    def run_multinash_and_get_cf1_positions(self, x0_all, goals_all, tau, dt):
        """Call run_multinash_once.py via venv and return cf1 positions as (K,3)."""
        import tempfile

        x0_all = np.asarray(x0_all, dtype=float)
        goals_all = np.asarray(goals_all, dtype=float)

        if x0_all.shape != (2, 8):
            raise ValueError(f"x0_all must be (2,8), got {x0_all.shape}")
        if goals_all.shape != (2, 3):
            raise ValueError(f"goals_all must be (2,3), got {goals_all.shape}")

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

            # Read output.json
            if not os.path.exists(output_path):
                raise RuntimeError("MultiNash planner did not produce output.json")

            with open(output_path, 'r') as f:
                data_out = json.load(f)

        positions = np.array(data_out['positions'], dtype=float)  # shape (2, tau_eff, 3)
        success = data_out.get('success', True)
        message = data_out.get('message', '')
        cost = data_out.get('cost', 0.0)

        if not success:
            self.get_logger().warn(f"MultiNash reported non-success: {message}")

        # We assume dt is as requested; if you ever change dt in planner, adjust here.
        tau_eff = positions.shape[1]
        dt_eff = dt

        positions_cf1 = positions[0, :, :]  # shape (tau_eff, 3)
        return positions_cf1, tau_eff, dt_eff

    def on_upload_done(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"UploadTrajectory call failed: {e}")
            return

        self.get_logger().info("UploadTrajectory succeeded.")
        # Start trajectory on cf1
        start_req = StartTrajectory.Request()
        start_req.group_mask = 0
        start_req.trajectory_id = 1
        start_req.timescale = 1.0
        start_req.reversed = False
        start_req.relative = False

        self.get_logger().info("Starting trajectory on cf1 (async)...")
        self.start_future = self.start_cli.call_async(start_req)
        self.start_future.add_done_callback(self.on_start_done)

    def on_start_done(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"StartTrajectory call failed: {e}")
            return

        self.get_logger().info("cf1 is now executing the MultiNash-based trajectory.")


def main(args=None):
    rclpy.init(args=args)
    node = MultiNashExecCF1MNash()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
