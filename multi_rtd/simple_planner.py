# Publishes a set nominal trajectory to be plotted in Gazebo

import rclpy
from rclpy.node import Node
import numpy as np
from scipy.io import loadmat
import os

from LPM import LPM
from planner_utils import wrap_traj_msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SimplePlanner(Node):

    def __init__(self):
        super().__init__('simple_planner')
        self.traj_pub = self.create_publisher(JointTrajectory, 'planner/traj', 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # init LPM object
        filepath = '/home/talbot330-red/ros2_ws/src/multi_rtd/multi_rtd/quadrotor_linear_planning_model.mat'
        test_lpm = loadmat(filepath)
        lpm = LPM(filepath)

        # compute trajectory for some arbitrary trajectory parameters
        v_x_0 = -1.0; a_x_0 = -1.0; v_x_peak = 2.3
        v_y_0 = 1.0; a_y_0 = 5.0; v_y_peak = -3.3
        v_z_0 = 0.5; a_z_0 = 2.2; v_z_peak = 2.0

        t2start = [0, 0] # seconds, nanoseconds 

        k = np.array([[v_x_0, a_x_0, v_x_peak],
                    [v_y_0, a_y_0, v_y_peak],
                    [v_z_0, a_z_0, v_z_peak]])

        traj = lpm.compute_trajectory(k)

        # convert to JointTrajectory msg 
        self.traj_msg = wrap_traj_msg(traj,t2start)


    # publish the trajectory
    def timer_callback(self):
        self.traj_pub.publish(self.traj_msg)


def main(args=None):
    print("Starting simple planner...")
    rclpy.init(args=args)

    simple_planner = SimplePlanner()

    rclpy.spin(simple_planner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simple_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()