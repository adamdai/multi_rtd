# Publishes a set nominal trajectory to be plotted in Gazebo

import rclpy
import rospkg
from rclpy.node import Node
import numpy as np
from scipy.io import loadmat
import os

from LPM import LPM
from planner_utils import wrap_robot_traj_msg
from std_msgs.msg import Bool 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from multi_rtd_interfaces.msg import RobotTrajectory

class TrajectorySampler(Node):

    def __init__(self):
        super().__init__('simple_planner')
        self.traj_pub = self.create_publisher(RobotTrajectory, 'planner/traj', 10)

        self.at_home_sub = self.create_subscription(Bool, 'at_home', self.at_home_callback, 10)

        self.ns = self.get_namespace()

        # parameters
        self.v_max = 3.0 # maximum velocity
        self.N_runs = 10 # number of times to run each trajectory
        self.N_trajs = 8 # total number of trajectories

        # state variables
        self.run_count = 0 # counter for number of times current trajectory has been run
        self.traj_count = 0 # counter for which trajectory we are currently testing

        # init LPM object
        script_dir = os.path.dirname(os.path.abspath('src'))
        filepath = script_dir + '/src/px4_multi_agent_planning/multi_rtd/multi_rtd/quadrotor_linear_planning_model.mat'
        lpm = LPM(filepath)

        # zero initial velocity and acceleration
        v_x_0 = 0; a_x_0 = 0; 
        v_y_0 = 0; a_y_0 = 0; 
        v_z_0 = 0; a_z_0 = 0; 

        # generate all trajectories to test
        # self.trajs = []
        # for v_pk_x in [-self.v_max,self.v_max]:
        #     for v_pk_y in [-self.v_max,self.v_max]:
        #         for v_pk_z in [-self.v_max,self.v_max]:
        #             k = np.array([[v_x_0, a_x_0, v_pk_x],
        #                           [v_y_0, a_y_0, v_pk_y],
        #                           [v_z_0, a_z_0, v_pk_z]])
        #             traj = lpm.compute_trajectory(k)
        #             self.trajs.append(traj)

        # hard-code trajectory
        v_pk_x = 1; v_pk_y = 1; v_pk_z = -3
        k = np.array([[v_x_0, a_x_0, v_pk_x],
                      [v_y_0, a_y_0, v_pk_y],
                      [v_z_0, a_z_0, v_pk_z]])
        self.traj = lpm.compute_trajectory(k)

        self.t2start = 0 # seconds


    def at_home_callback(self, msg):
        if msg.data:
            self.publish_traj()

    # publish the trajectory
    def publish_traj(self):
        # iterate through trajectory space
        # if self.traj_count < self.N_trajs:
        #     print(" publishing trajectory", self.traj_count, "of", self.N_trajs, ", run", self.run_count, "of", self.N_runs)
        #     traj_msg = wrap_robot_traj_msg(self.trajs[self.traj_count], self.t2start, self.ns)
        #     self.traj_pub.publish(traj_msg)
        #     self.run_count = self.run_count + 1
        #     if self.run_count > 10:
        #         self.traj_count  = self.traj_count + 1
        #         self.run_count = 0

        # hard-coded trajectory
        if self.run_count < 10:
            print(" publishing trajectory", self.run_count)
            traj_msg = wrap_robot_traj_msg(self.traj, self.t2start, self.ns)
            self.traj_pub.publish(traj_msg)
            self.run_count = self.run_count + 1
        else:
            rclpy.shutdown()


def main(args=None):
    print("Starting trajectory sampler...")
    rclpy.init(args=args)

    trajectory_sampler = TrajectorySampler()

    rclpy.spin(trajectory_sampler)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    trajectory_sampler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()