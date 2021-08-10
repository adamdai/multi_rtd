import rclpy
from rclpy.node import Node
import numpy as np
import os

from LPM import LPM
from planner_utils import *
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped
from multi_rtd_interfaces.msg import RobotTrajectory

class MultiPlanner(Node):
    """Multi-agent Planner

    """

    def __init__(self, lpm_file):
        super().__init__('multi_planner')
        """ --------------- Timing and global variables --------------- """
        self.T_REPLAN = 0.3 # [s] amount of time between replans
        self.T_PLAN = 0.1 # [s] amount of time allotted for planning itself (remaining time allotted for checking)
        self.N_BOTS = 3 # total number of bots in the sim
        self.N_DIM = 3
        self.R_BOT = 0.25 # [m]
        self.XY_BOUNDS  = [-5.0, 5.0] # [m]
        self.Z_BOUNDS = [0.0, 5.0] # [m]

        # get namespace
        # self.name = self.get_namespace()
        self.name = '/ibqr1/'

        # replan timer
        self.replan_timer = self.create_timer(self.T_REPLAN, self.replan)
        self.init_time = self.get_abs_time()

        """ --------------- Publishers and Subscribers --------------- """
        # publisher for planned trajectory
        self.traj_pub = self.create_publisher(RobotTrajectory, self.name + 'planner/traj', 10)

        # subscriber for goal position
        goal_sub = self.create_subscription(PointStamped, self.name + 'planner/goal', self.goal_callback, 10)
        self.goal_pub = self.create_publisher(PointStamped, self.name + 'planner/goal', 10)

        # subscribers for peer robot plans
        plan_subs = {}
        peer_bots = ['/ibqr' + str(i) + '/' for i in range(1,self.N_BOTS+1)]
        peer_bots.remove(self.name)
        for peer_bot in peer_bots:
            topic = peer_bot + 'planner/traj'
            plan_subs[peer_bot] = self.create_subscription(RobotTrajectory, topic, self.traj_callback, 10)

        """ --------------- Planning variables --------------- """
        # init LPM object
        self.lpm = LPM(lpm_file)
        self.n_t_plan = len(self.lpm.time) # planned trajectory length
        self.n_plan_max = 10000 # max number of plans to evaluate

        # initial conditions [m],[m/s],[m/s^2]
        self.p_0 = np.zeros((3,1))
        self.p_0[0] = 0; self.p_0[1] = 0
        self.p_0[:2] = np.reshape(np.random.uniform(self.XY_BOUNDS[0], self.XY_BOUNDS[1], 2), (2,1))
        self.v_0 = np.zeros((3,1))
        self.a_0 = np.zeros((3,1))

        # own current plan (10 x N) 
        # pending and committed
        # each column has [t, px, py, pz, vx, vy, vz, ax, ay, az]^T 
        self.pend_plan = np.zeros((1+3*self.N_DIM, self.n_t_plan))
        self.commit_plan = np.zeros((1+3*self.N_DIM, self.n_t_plan))

        # initialize with stationary plan
        self.pend_plan[0,:] = self.lpm.time
        self.pend_plan[1:4,:] = np.tile(self.p_0, (1,self.n_t_plan))
        self.commit_plan[0,:] = self.lpm.time
        self.commit_plan[1:4,:] = np.tile(self.p_0, (1,self.n_t_plan))

        # dictionary of peers' committed plans 
        self.committed_plans = {}

        # max velocity constraints [m/s]
        self.v_max = 2.0 # L1 velocity constraints
        self.V_bounds = np.tile(np.array([-self.v_max, self.v_max]), (1,3))[0]
        self.v_max_norm = 2.0 # L2 velocity constraints
        self.delta_v_peak_max = 3.0 # delta from initial velocity constraint

        # goal [m]
        self.p_goal = np.zeros((3,1))
        self.p_goal[:2] = np.reshape(np.random.uniform(self.XY_BOUNDS[0], self.XY_BOUNDS[1], 2), (2,1))
        self.p_goal[2] = np.random.uniform(self.Z_BOUNDS[0], self.Z_BOUNDS[1])
        #self.p_goal = np.array([[5],[5],[0]])
        self.flag_new_goal = False
        self.r_goal_reached = 0.3 # [m] stop planning when within this dist of goal


    def get_time(self):
        """Time since class was initialized.
        
        Rounded to nearest t_sample.
        
        """
        return self.get_abs_time() - self.init_time


    def get_abs_time(self):
        """ROS time in seconds.
        
        Returns
        -------
        float
            ROS time in seconds.

        """
        return self.get_clock().now().nanoseconds / 1e9


    # def get_discrete_time(self):
    #     """ROS time rounded to nearest t_sample.

    #     """
    #     return round_to(self.get_abs_time(), self.t_sample)


    def goal_callback(self, data):
        """Goal subscriber callback.

        Stores desired goal.

        """
        rospy.loginfo("Goal Callback")
        self.p_goal = np.array([[data.point.x], [data.point.y], [data.point.z]])
        self.flag_new_goal = True


    def traj_callback(self, data):
        """Peer plan callback.

        Save peer committed plan.

        """
        bot_name = data.robot_name
        traj = data.trajectory
        x_pos = traj.points[0].positions
        y_pos = traj.points[1].positions
        z_pos = traj.points[2].positions
        t2start = traj.points[0].time_from_start 
        time = self.lpm.time + t2start.to_sec()
        plan = np.vstack((time, x_pos, y_pos, z_pos))
        # save plan and time received 
        self.committed_plans[bot_name] = (plan, self.get_time())
    

    def check_peer_plan_collisions(self, plan):
        """ Check a plan against peers' committed plans for collision.

        Parameters
        ----------
        plan : np.array
            Candidate plan to check against peers' committed plans.

        Returns
        -------
        bool
            True if plan is safe, False if there is a collision.

        """
        for bot in self.committed_plans.keys():
            other_plan = self.committed_plans[bot][0]
            if not check_plan_collision(plan, other_plan, 2*self.R_BOT):
                return False
        return True


    def traj_opt(self, t_start_plan, T_old, X_old, t2start, T_new):
        """Trajectory optimization.

        Attempt to find a collision-free plan (v_peak) which brings the agent 
        closest to its goal.

        Parameters
        ----------
        t_start_plan : float
            Start time of current planning instance.
        T_old : np.array
            Time vector for the old plan.
        X_old : np.array
            State vector for the old plan.
        t2start : float
            Time to start the new plan from.
        T_new: np.array
            Time vector for new plan.

        Returns
        -------
        np.array
            Peak velocities for new plan.

        """
        # index current plan to get initial condition for new plan
        x_0 = trajectory_closest_point(t2start, T_old, X_old)
        # update initial conditions
        self.p_0 = np.reshape(x_0[:self.N_DIM], (3,1))
        self.v_0 = np.reshape(x_0[self.N_DIM:2*self.N_DIM], (3,1))
        self.a_0 = np.reshape(x_0[2*self.N_DIM:3*self.N_DIM], (3,1))

        # generate potential v_peak samples
        V_peak = rand_in_bounds(self.V_bounds, self.n_plan_max)

        # eliminate samples that exceed the max velocity and max delta
        # from initial velocity
        V_peak_mag = np.linalg.norm(V_peak, 2, 0)
        delta_V_peak_mag = np.linalg.norm(V_peak - np.tile(self.v_0,(1,self.n_plan_max)), 2, 0)
        V_peak_test = np.logical_and(V_peak_mag < self.v_max_norm, delta_V_peak_mag < self.delta_v_peak_max)
        V_peak = V_peak[:,V_peak_test]

        # get number of potentially feasible sample to iterate through
        n_V_peak = V_peak.shape[1]

        # calculate the endpoints for the sample v_peaks
        lpm_p_final = self.lpm.p_mat[:,-1]
        p_from_v_0_and_a_0 = np.reshape(np.dot(np.hstack((self.v_0, self.a_0)), lpm_p_final[:2]),(3,1)) + self.p_0
        p_from_v_peak = lpm_p_final[2] * V_peak + np.tile(p_from_v_0_and_a_0, (1,n_V_peak))

        # sort V_peaks by distance to goal
        dist_to_goal = np.linalg.norm(p_from_v_peak - np.tile(self.p_goal, (1,n_V_peak)), 2, 0)
        V_sort_idxs = np.argsort(dist_to_goal)
        V_peak = V_peak[:,V_sort_idxs]

        # iterate through V_peaks until we find a feasible one
        idx_v_peak = 0
        while (idx_v_peak <= n_V_peak) and (self.get_time() - t_start_plan < self.T_PLAN):
            
            # get trajectory positions for current v_peak
            v_peak = np.reshape(V_peak[:,idx_v_peak], (3,1))
            P_idx = np.matmul(np.hstack((self.v_0, self.a_0, v_peak)), self.lpm.p_mat) + np.tile(self.p_0, (1,self.n_t_plan))
            cand_plan = np.vstack((T_new, P_idx))

            # check against other plans
            check_others = self.check_peer_plan_collisions(cand_plan)

            # check against obstacles
            # TODO

            if check_others:
                return v_peak
            else:
                idx_v_peak += 1

        # no v_peaks are feasible
        return None


    def check(self, t_start_plan):
        """Post optimization check.

        Checks if current pending plan collides with any peer plans.

        Returns
        -------
        bool
            True if success, False if failed.
        """
        for bot in self.committed_plans.keys():
            # plan was committed during optimization
            if self.committed_plans[bot][1] > t_start_plan:
                # check if it collides with pending plan
                if not check_plan_collision(self.pend_plan, self.committed_plans[bot][0], 2*self.R_BOT):
                    # check failed
                    return False
        # no conflicts - check success
        return True


    def recheck(self, t_start_check):
        """Post check check.

        Checks if any peer plans were committed (published) during the check.

        Returns
        -------
        bool
            True if success, False if failed.

        """
        for bot in self.committed_plans.keys():
            # plan was committed during check   
            if self.committed_plans[bot][1] > t_start_check:
                # bail out :(
                return False
        # no conflicts - recheck success
        return True


    def replan(self):
        """Replan

        Perform trajectory optimization, checking, and rechecking 

        """
        # start timer
        t_start_plan = self.get_time()
        self.get_logger().info(str(t_start_plan))

        # get current plan
        T_old = self.commit_plan[0,:]
        X_old = self.commit_plan[1:,:]

        # time to start the trajectory from (relative to absolute time)
        t2start = self.get_abs_time() + self.T_REPLAN 

        # create time vector for the new plan
        T_new = self.lpm.time + t2start

        # find a new v_peak
        v_peak = self.traj_opt(t_start_plan, T_old, X_old, t2start, T_new)

        # if no new plan found continue previous plan
        if v_peak is None:
            self.get_logger().info("Failed to find a new plan")
            # select parts of the previous plan that are yet to be executed
            T_log = T_old >= self.get_time()

            self.get_logger().info(str(T_log.shape))
            self.get_logger().info(str(T_old.shape))
            self.get_logger().info(str((T_old[-1] + self.T_PLAN).shape))

            # increase the length of the old plan by t_plan
            self.commit_plan[0,:] = np.hstack((T_old[T_log], T_old[-1] + self.T_PLAN))
            self.commit_plan[1:,:] = np.hstack((X_old[:,T_log], X_old[:,-1]))

            return

        # otherwise, create a new pending plan and enter checking phase
        else:  
            self.get_logger().info("Found a new plan")
            k = np.hstack((self.v_0, self.a_0, v_peak))
            p,v,a = self.lpm.compute_trajectory(k) 
            p = p + self.p_0 # translate to p_0
            self.pend_plan[0,:] = T_new
            self.pend_plan[1:,:] = np.vstack((p,v,a))

            # get checking start time
            t_start_check = self.get_time()

            # check pending plan against recently committed plans
            if self.check(t_start_plan):
                self.get_logger().info("Check succeeded")
                # if check passes, do final recheck
                if self.recheck(t_start_check):
                    self.get_logger().info("Recheck succeeded")
                    # planning has succeeded! commit and publish the plan
                    self.commit_plan[0,:] = self.pend_plan[0,:]
                    self.commit_plan[1:,:] = self.pend_plan[1:,:]
                    traj_msg = wrap_robot_traj_msg((p,v,a), t2start, self.name)
                    self.traj_pub.publish(traj_msg)
            
            # if either check or recheck fails, bail out and revert to previous plan
            # TODO?

            return
        
        # end if current committed trajectory endpoint reaches goal
        # if np.linalg.norm(np.reshape(self.commit_plan[1:4,-1], (3,1)) - self.p_goal) < self.r_goal_reached:
        #     rospy.loginfo("Successfully planned trajectory to goal...exiting")
        #     rospy.signal_shutdown("Planner finished")
        # return




def main(args=None):
    print("Running Multi-Robot Planner")
    rclpy.init(args=args)

    # get LPM path
    script_dir = os.path.dirname(os.path.abspath('src'))
    lpm_path = script_dir + '/src/px4_multi_agent_planning/multi_rtd/multi_rtd/quadrotor_linear_planning_model.mat'
    
    multi_planner = MultiPlanner(lpm_path)

    rclpy.spin(multi_planner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    multi_planner.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()