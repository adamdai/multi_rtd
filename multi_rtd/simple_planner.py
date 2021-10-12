# Publishes a set nominal trajectory to be plotted in Gazebo

import rclpy
import rospkg
from rclpy.node import Node
import numpy as np
from scipy.io import loadmat
import os

from LPM import LPM
from planner_utils import wrap_traj_msg, wrap_robot_traj_msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from multi_rtd_interfaces.msg import RobotTrajectory

class SimplePlanner(Node):

    def __init__(self):
        super().__init__('simple_planner')
        self.traj_pub = self.create_publisher(RobotTrajectory, 'planner/traj', 10)

        timer_period = 10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.ns = self.get_namespace()

        # init LPM object
        script_dir = os.path.dirname(os.path.abspath('src'))
        filepath = script_dir + '/src/px4_multi_agent_planning/multi_rtd/multi_rtd/quadrotor_linear_planning_model.mat'
        #filepath = '/home/talbot330-red/ros2_ws/src/multi_rtd/multi_rtd/quadrotor_linear_planning_model.mat'
        lpm = LPM(filepath)

        # compute trajectory for some arbitrary trajectory parameters
        v_x_0 = 0; a_x_0 = 0; v_x_peak = 2.3
        v_y_0 = 0; a_y_0 = 0; v_y_peak = -3.3
        v_z_0 = 0; a_z_0 = 0; v_z_peak = 2.0

        t2start = 0 # seconds

        k = np.array([[v_x_0, a_x_0, v_x_peak],
                    [v_y_0, a_y_0, v_y_peak],
                    [v_z_0, a_z_0, v_z_peak]])

        traj = lpm.compute_trajectory(k)

        # convert to RobotTrajectory msg 
        self.traj_msg = wrap_robot_traj_msg(traj,t2start,self.ns)
        #self.traj_msg = wrap_traj_msg(traj,t2start)


    # publish the trajectory
    def timer_callback(self):
        print(" publishing trajectory")
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