#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
import os
import csv

from builtin_interfaces.msg import Duration
from crazyflie_interfaces.srv import UploadTrajectory, StartTrajectory, Takeoff
from crazyflie_interfaces.msg import TrajectoryPolynomialPiece
from tf2_ros import Buffer, TransformListener

sys.path.append('/home/rk32226/DronePotentialGame')
from poly_helper import waypoints_to_trajectory


class MultiNashExecNCFTraj(Node):
    def __init__(self):
        super().__init__('multinash_exec_n_cf_traj')

        # Declare parameters
        self.declare_parameter('n_drones', 2)
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('target_z', 0.8)
        
        self.n_drones = self.get_parameter('n_drones').value
        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.target_z = self.get_parameter('target_z').value
        
        if not self.waypoints_file:
            self.get_logger().error("No waypoints_file specified! Use --ros-args -p waypoints_file:=/path/to/file.csv")
            rclpy.shutdown()
            return
        
        self.get_logger().info(f"Initializing with n_drones={self.n_drones}, waypoints_file={self.waypoints_file}")

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
            
            self.takeoff_clis.append(self.create_client(Takeoff, f'/{name}/takeoff'))
            self.upload_clis.append(self.create_client(UploadTrajectory, f'/{name}/upload_trajectory'))
            self.start_clis.append(self.create_client(StartTrajectory, f'/{name}/start_trajectory'))

            self.get_logger().info(f"Waiting for services for {name}...")
            self.takeoff_clis[-1].wait_for_service()
            self.upload_clis[-1].wait_for_service()
            self.start_clis[-1].wait_for_service()

        self.get_logger().info(f"All services for {self.n_drones} drones are available.")

        self.started = False
        self.ascent_complete = False
        self.uploads_completed = [False] * self.n_drones
        self.takeoffs_completed = [False] * self.n_drones

        self.timer = self.create_timer(1.0, self.timer_cb)

    def timer_cb(self):
        if self.started:
            return
        self.started = True

        # Command all drones to ascend
        self.get_logger().info(f"Commanding drones to ascend to z={self.target_z}...")
        self.ascend_all_drones()

    def ascend_all_drones(self):
        """Command all drones to ascend to target_z height."""
        for i in range(self.n_drones):
            req = Takeoff.Request()
            req.group_mask = 0
            req.height = float(self.target_z)
            d = Duration()
            d.sec = 3
            d.nanosec = 0
            req.duration = d

            self.get_logger().info(f"Sending {self.drone_names[i]} takeoff to height {self.target_z:.2f}")
            
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
        """Continue with trajectory upload once all drones have reached target height."""
        if all(self.takeoffs_completed) and not self.ascent_complete:
            self.ascent_complete = True
            self.get_logger().info(f"All {self.n_drones} drones commanded to takeoff. Waiting 4 seconds for ascent...")
            self.ascent_timer = self.create_timer(4.0, self.continue_after_ascent)

    def continue_after_ascent(self):
        """Load waypoints and upload trajectories after ascent is complete."""
        self.ascent_timer.cancel()
        self.get_logger().info("Ascent complete. Loading waypoints from CSV...")

        try:
            waypoints_per_drone = self.load_waypoints_from_csv(self.waypoints_file)
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            return

        # Convert and upload for each drone
        for i in range(self.n_drones):
            drone_name = self.drone_names[i]
            
            if drone_name not in waypoints_per_drone:
                self.get_logger().error(f"No waypoints found for {drone_name} in CSV!")
                return
            
            waypoints = waypoints_per_drone[drone_name]  # List of [t, x, y, z, yaw]
            
            self.get_logger().info(f"Converting {drone_name} waypoints to polynomial trajectory...")
            traj = waypoints_to_trajectory(waypoints, pieces=min(8, len(waypoints)-1))
            pieces = self.traj_to_pieces(traj)

            upload_req = UploadTrajectory.Request()
            upload_req.trajectory_id = 1
            upload_req.piece_offset = 0
            upload_req.pieces = pieces

            self.get_logger().info(f"Uploading trajectory to {drone_name} (async)...")
            future = self.upload_clis[i].call_async(upload_req)
            future.add_done_callback(lambda f, idx=i: self.on_upload_done(f, idx))

    def load_waypoints_from_csv(self, filepath):
        """
        Load waypoints from CSV file.
        Expected format:
        drone_id,t,x,y,z,yaw
        cf1,0.0,0.0,0.0,1.0,0.0
        cf1,0.1,0.01,0.0,1.0,0.0
        cf2,0.0,0.5,0.5,1.0,0.0
        ...
        """
        waypoints_per_drone = {}
        
        with open(filepath, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                drone_id = row['drone_id']
                t = float(row['t'])
                x = float(row['x'])
                y = float(row['y'])
                z = float(row['z'])
                yaw = float(row['yaw'])
                
                if drone_id not in waypoints_per_drone:
                    waypoints_per_drone[drone_id] = []
                
                waypoints_per_drone[drone_id].append([t, x, y, z, yaw])
        
        # Convert to numpy arrays for poly_helper
        import numpy as np
        for drone_id in waypoints_per_drone:
            waypoints_per_drone[drone_id] = np.array(waypoints_per_drone[drone_id])
        
        return waypoints_per_drone

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
            self.get_logger().info(f"{self.drone_names[idx]} is now executing its trajectory.")
        except Exception as e:
            self.get_logger().error(f"{self.drone_names[idx]} StartTrajectory failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MultiNashExecNCFTraj()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
