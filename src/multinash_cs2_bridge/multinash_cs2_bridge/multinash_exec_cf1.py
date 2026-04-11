#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
import numpy as np

from builtin_interfaces.msg import Duration
from crazyflie_interfaces.srv import UploadTrajectory, StartTrajectory
from crazyflie_interfaces.msg import TrajectoryPolynomialPiece

# Import the helper from your planning repo
sys.path.append('/home/rk32226/DronePotentialGame')
from poly_helper import waypoints_to_trajectory


class MultiNashExecCF1(Node):
    def __init__(self):
        super().__init__('multinash_exec_cf1')

        # Create service clients for cf1
        self.upload_cli = self.create_client(UploadTrajectory, '/cf1/upload_trajectory')
        self.start_cli  = self.create_client(StartTrajectory,  '/cf1/start_trajectory')

        # Wait for services to be available
        self.get_logger().info("Waiting for /cf1/upload_trajectory...")
        self.upload_cli.wait_for_service()
        self.get_logger().info("Waiting for /cf1/start_trajectory...")
        self.start_cli.wait_for_service()
        self.get_logger().info("Services are available.")

        # We'll trigger once via a timer
        self.started = False
        self.timer = self.create_timer(1.0, self.timer_cb)

    def timer_cb(self):
        if self.started:
            return
        self.started = True

        # Hard-coded test waypoints [t,x,y,z,yaw] in world frame
        txyz_yaw = np.array([
            [0.0, -0.5, -0.5, 0.5, 0.0],
            [1.5, -0.25, -0.25, 0.5, 0.0],
            [2.5,  0.0,  0.0,  0.5, 0.0],
            [4.0,  0.5,  0.5,  0.5, 0.0],
        ])

        # Convert waypoints -> Trajectory
        self.get_logger().info("Generating polynomial trajectory from waypoints...")
        traj = waypoints_to_trajectory(txyz_yaw, pieces=4)
        self.get_logger().info(f"Trajectory duration: {traj.duration}, segments: {len(traj.polynomials)}")

        # Convert Trajectory.polynomials into TrajectoryPolynomialPiece[]
        pieces = []
        for poly in traj.polynomials:
            piece = TrajectoryPolynomialPiece()

            # Duration
            d = Duration()
            d_sec = float(poly.duration)
            d.sec = int(d_sec)
            d.nanosec = int((d_sec - d.sec) * 1e9)
            piece.duration = d

            # Polynomial coefficients
            piece.poly_x = [float(c) for c in poly.px.p]
            piece.poly_y = [float(c) for c in poly.py.p]
            piece.poly_z = [float(c) for c in poly.pz.p]
            piece.poly_yaw = [float(c) for c in poly.pyaw.p]

            pieces.append(piece)

        # Prepare UploadTrajectory request
        upload_req = UploadTrajectory.Request()
        upload_req.trajectory_id = 1
        upload_req.piece_offset = 0
        upload_req.pieces = pieces

        self.get_logger().info("Uploading trajectory to cf1 (async)...")
        self.upload_future = self.upload_cli.call_async(upload_req)
        self.upload_future.add_done_callback(self.on_upload_done)

    def on_upload_done(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"UploadTrajectory call failed: {e}")
            return

        self.get_logger().info("UploadTrajectory succeeded.")
        # Now start the trajectory
        start_req = StartTrajectory.Request()
        # Bitmask; ignored when calling /cf1, but must be set
        start_req.group_mask = 0
        # Must match the trajectory_id used in UploadTrajectory
        start_req.trajectory_id = 1
        # 1.0 = nominal speed
        start_req.timescale = 1.0
        # Fly forward, not reversed
        start_req.reversed = False
        # Interpret as absolute (world-frame) trajectory
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

        self.get_logger().info("cf1 is now executing the uploaded trajectory.")


def main(args=None):
    rclpy.init(args=args)
    node = MultiNashExecCF1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

