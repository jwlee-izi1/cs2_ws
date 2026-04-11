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


class MultiNashExecNCF(Node):
    def __init__(self):
        super().__init__('multinash_exec_n_cf_mnash')

        # Declare and get parameter for number of drones
        self.declare_parameter('n_drones', 2)
        self.n_drones = self.get_parameter('n_drones').value
        self.get_logger().info(f"Initializing MultiNashExecNCF with n_drones={self.n_drones}")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Dynamic service clients
        self.takeoff_clis = []
        self.upload_clis = []
        self.start_clis = []
        self.drone_names = []

        for i in range(1, self.n_drones + 1):
            name = f'cf{i}'
            self.drone_names.append(name)
            
            takeoff_topic = f'/{name}/takeoff'
            upload_topic = f'/{name}/upload_trajectory'
            start_topic = f'/{name}/start_trajectory'

            self.takeoff_clis.append(self.create_client(Takeoff, takeoff_topic))
            self.upload_clis.append(self.create_client(UploadTrajectory, upload_topic))
            self.start_clis.append(self.create_client(StartTrajectory, start_topic))

            self.get_logger().info(f"Waiting for services for {name}...")
            self.takeoff_clis[-1].wait_for_service()
            self.upload_clis[-1].wait_for_service()
            self.start_clis[-1].wait_for_service()

        self.get_logger().info(f"All services for {self.n_drones} drones are available.")

        self.started = False
        self.ascent_complete = False
        self.uploads_completed = [False] * self.n_drones
        self.takeoffs_completed = [False] * self.n_drones
        
        # Store initial positions
        self.initial_positions = [None] * self.n_drones

        self.timer = self.create_timer(1.0, self.timer_cb)

    def get_positions_world(self):
        now = rclpy.time.Time()
        positions = []
        for name in self.drone_names:
            try:
                tf = self.tf_buffer.lookup_transform('world', name, now)
                p = tf.transform.translation
                positions.append([p.x, p.y, p.z])
            except Exception as e:
                self.get_logger().warn(f"Could not get transform for {name}: {e}")
                raise e
        
        pos_array = np.array(positions, dtype=float)
        self.get_logger().info(f"Initial positions (world): {pos_array}")
        return pos_array

    def timer_cb(self):
        if self.started:
            return
        self.started = True

        # 1) Get initial positions
        try:
            p = self.get_positions_world()  # shape (N,3)
        except Exception as e:
            self.get_logger().error(f"Failed to get TF positions: {e}")
            self.started = False
            return

        # 2) Command all drones to ascend to z=1.0 before planning
        self.get_logger().info("Commanding drones to ascend to z=1.0...")
        self.ascend_to_height(p, target_z=0.8, duration_sec=3.0)

    def ascend_to_height(self, current_positions, target_z=0.8, duration_sec=3.0):
        """Command all drones to ascend to target_z height."""
        self.initial_positions = current_positions.copy()
        
        # Update z to target_z for planning purposes later
        for i in range(self.n_drones):
            self.initial_positions[i][2] = target_z

        for i in range(self.n_drones):
            req = Takeoff.Request()
            req.group_mask = 0
            req.height = float(target_z)
            d = Duration()
            d.sec = int(duration_sec)
            d.nanosec = int((duration_sec - d.sec) * 1e9)
            req.duration = d

            self.get_logger().info(f"Sending {self.drone_names[i]} takeoff to height {target_z:.2f}")
            
            # Use a closure or default arg to capture 'i' correctly
            future = self.takeoff_clis[i].call_async(req)
            future.add_done_callback(lambda f, idx=i: self.on_takeoff_done(f, idx))

    def on_takeoff_done(self, future, idx):
        try:
            result = future.result()
            self.get_logger().info(f"{self.drone_names[idx]} Takeoff command accepted.")
        except Exception as e:
            self.get_logger().error(f"{self.drone_names[idx]} Takeoff failed: {e}")
            return
        
        self.takeoffs_completed[idx] = True
        self.maybe_continue_after_ascent()

    def maybe_continue_after_ascent(self):
        """Continue with planning once all drones have reached target height."""
        if all(self.takeoffs_completed) and not self.ascent_complete:
            self.ascent_complete = True
            self.get_logger().info(f"All {self.n_drones} drones commanded to takeoff. Waiting 4 seconds for ascent...")
            self.ascent_timer = self.create_timer(4.0, self.continue_after_ascent)

    def continue_after_ascent(self):
        """Continue with MultiNash planning after ascent is complete."""
        self.ascent_timer.cancel()
        self.get_logger().info("Ascent complete. Proceeding with MultiNash planning...")

        # Build x0_all and goals_all
        x0_all = np.zeros((self.n_drones, 8), dtype=float)
        goals_all = np.zeros((self.n_drones, 3), dtype=float)

        for i in range(self.n_drones):
            # Start state: [x, y, z, phi, theta, psi, v, w] (approx, based on 2-drone code)
            # 2-drone code used: x0_1[0:3] = p1_0, x0_1[3]=0, x0_1[4]=0, x0_1[5]=0.5
            # We'll assume similar defaults.
            x0_all[i, 0:3] = self.initial_positions[i]
            x0_all[i, 3] = 0.0 if i % 2 == 0 else np.pi # Alternating yaw for variety? Or just 0?
            # Keeping it simple: just 0 for now unless we want to mimic the 2-drone exact setup
            # The 2-drone code had: x0_1[3]=0.0, x0_2[3]=np.pi. 
            # Let's just set 0 for all for simplicity, or distribute them.
            # Let's stick to the 2-drone logic if N=2, else 0.
            if self.n_drones == 2 and i == 1:
                x0_all[i, 3] = np.pi
            else:
                x0_all[i, 3] = 0.0
            
            x0_all[i, 4] = 0.0
            x0_all[i, 5] = 0.5 # v_init?

            # Goal: Move across the center (e.g., Drone 0 -> Pos N/2, Drone 1 -> Pos N/2 + 1)
            N = self.n_drones
            target_idx = (i + N // 2) % N
            goals_all[i] = self.initial_positions[target_idx].copy()
            goals_all[i][2] = 1.0 # Ensure goal is at z=1.0

        tau = 30
        dt  = 0.1

        self.get_logger().info(f"Calling MultiNash planner for {self.n_drones} CFs...")
        try:
            positions_all, tau_eff, dt_eff = self.run_multinash_and_get_positions(
                x0_all, goals_all, tau=tau, dt=dt
            )
        except Exception as e:
            self.get_logger().error(f"MultiNash planning failed: {e}")
            return

        self.get_logger().info(f"MultiNash returned trajectories with tau={tau_eff}, dt={dt_eff}")

        # Process and upload trajectories for each drone
        K = positions_all.shape[1]
        t_array = np.arange(K, dtype=float) * dt_eff
        yaw_array = np.zeros(K, dtype=float)

        for i in range(self.n_drones):
            positions_cf = positions_all[i, :, :] # (K, 3)
            
            txyz_yaw = np.column_stack([t_array,
                                        positions_cf[:,0],
                                        positions_cf[:,1],
                                        positions_cf[:,2],
                                        yaw_array])
            
            self.get_logger().info(f"Converting {self.drone_names[i]} waypoints to polynomial trajectory...")
            traj = waypoints_to_trajectory(txyz_yaw, pieces=min(8, K-1))
            pieces = self.traj_to_pieces(traj)

            upload_req = UploadTrajectory.Request()
            upload_req.trajectory_id = 1
            upload_req.piece_offset = 0
            upload_req.pieces = pieces

            self.get_logger().info(f"Uploading trajectory to {self.drone_names[i]} (async)...")
            future = self.upload_clis[i].call_async(upload_req)
            future.add_done_callback(lambda f, idx=i: self.on_upload_done(f, idx))

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

        positions = np.array(data_out['positions'], dtype=float)  # (N, tau_eff, 3)
        success = data_out.get('success', True)
        message = data_out.get('message', '')
        
        if not success:
            self.get_logger().warn(f"MultiNash reported non-success: {message}")

        tau_eff = positions.shape[1]
        dt_eff = dt

        # Save the plan for later visualization/analysis
        save_path = '/home/rk32226/DronePotentialGame/latest_plan.npy'
        try:
            np.save(save_path, positions)
            self.get_logger().info(f"Saved planner trajectories to {save_path}")
        except Exception as e:
            self.get_logger().warn(f"Failed to save planner trajectories: {e}")

        return positions, tau_eff, dt_eff

    def on_upload_done(self, future, idx):
        try:
            result = future.result()
            self.get_logger().info(f"{self.drone_names[idx]} UploadTrajectory succeeded.")
        except Exception as e:
            self.get_logger().error(f"{self.drone_names[idx]} UploadTrajectory failed: {e}")
            return
        
        self.uploads_completed[idx] = True
        self.maybe_start_all()

    def maybe_start_all(self):
        # Start all CFs only once all uploads have succeeded
        if all(self.uploads_completed):
            self.get_logger().info(f"All {self.n_drones} trajectories uploaded; starting all CFs...")

            for i in range(self.n_drones):
                start_req = StartTrajectory.Request()
                start_req.group_mask = 0
                start_req.trajectory_id = 1
                start_req.timescale = 1.0
                start_req.reversed = False
                start_req.relative = False

                future = self.start_clis[i].call_async(start_req)
                future.add_done_callback(lambda f, idx=i: self.on_start_done(f, idx))

    def on_start_done(self, future, idx):
        try:
            result = future.result()
            self.get_logger().info(f"{self.drone_names[idx]} is now executing its MultiNash-based trajectory.")
        except Exception as e:
            self.get_logger().error(f"{self.drone_names[idx]} StartTrajectory failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MultiNashExecNCF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
