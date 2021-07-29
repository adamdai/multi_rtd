# Utilities for planner nodes

import rclpy
import numpy as np

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# check 2 plans against each other to see if they collide
# return true if plan is good, false if there is a collision
def check_plan_collision(plan_1, plan_2, r_collision):
    time_1 = plan_1[0,:]; traj_1 = plan_1[1:4,:]
    time_2 = plan_2[0,:]
    traj_2 = match_trajectories(time_1, time_2, plan_2[1:4,:])
    d_vec = np.linalg.norm(traj_1 - traj_2, 2, 0)
    if any(d_vec <= r_collision):
        return False
    else:
        return True

# take a 3-D trajectory as a (p,v,a) tuple and wrap it in a 
# JointTrajectory message with x,y,z joints
def wrap_traj_msg(traj, t2start):
    p,v,a = traj
    traj_msg = JointTrajectory()

    jtp_x = JointTrajectoryPoint()
    jtp_x.positions = list(p[0])
    jtp_x.velocities = list(v[0])
    jtp_x.accelerations = list(a[0])
    jtp_x.time_from_start = Duration()
    jtp_x.time_from_start.sec = t2start[0]
    jtp_x.time_from_start.nanosec = t2start[1]

    jtp_y = JointTrajectoryPoint()
    jtp_y.positions = list(p[1])
    jtp_y.velocities = list(v[1])
    jtp_y.accelerations = list(a[1])
    jtp_y.time_from_start = Duration()
    jtp_y.time_from_start.sec = t2start[0]
    jtp_y.time_from_start.nanosec = t2start[1]

    jtp_z = JointTrajectoryPoint()
    jtp_z.positions = list(p[2])
    jtp_z.velocities = list(v[2])
    jtp_z.accelerations = list(a[2])
    jtp_z.time_from_start = Duration()
    jtp_z.time_from_start.sec = t2start[0]
    jtp_z.time_from_start.nanosec = t2start[1]

    traj_msg.points = [jtp_x, jtp_y, jtp_z]
    traj_msg.joint_names = ['x','y','z']

    return traj_msg


# generate random samples within specified bounds
#  bounds - [xmin xmax ymin ymax] (in 2D) or
#           [xmin xmax ymin ymax zmin zmax] (in 3D)
#  mean   - (optional)
#  std    - (optional)
#  n      - number of points
# 
def rand_in_bounds(bounds, n):

    x_pts = np.random.uniform(bounds[0], bounds[1], n)
    y_pts = np.random.uniform(bounds[2], bounds[3], n)
    # 2D 
    if len(bounds) == 4:
        return np.vstack((x_pts, y_pts))
    elif len(bounds) == 6:
        z_pts = np.random.uniform(bounds[4], bounds[5], n)
        return np.vstack((x_pts, y_pts, z_pts))
    else:
        raise ValueError('Please pass in bounds as either [xmin xmax ymin ymax] '
                            'or [xmin xmax ymin ymax zmin zmax] ')


# Given an input trajectory and associated time vector, resample
# according to a desired time and return the resulting trajectory
# assumes input and output times are sampled at same interval
#  T_out - 1 x n_t_out
#  T_in  - 1 x n_t_in
#  X_in  - n_states x n_t_in
#  X_out - n_states x n_t_out
def match_trajectories(T_out, T_in, X_in):
    # initialize output 
    n_states = X_in.shape[0]
    n_t_out = len(T_out)
    X_out = np.zeros((n_states, n_t_out))
    for i in range(n_t_out):
        X_out[:,i] = trajectory_closest_point(T_out[i], T_in, X_in)
    return X_out

    # # output time starts before input time starts
    # if T_out[0] < T_in[0]:
    #     # if output ends before input ends 
    #     if T_out[-1] < T_in[-1]:
    #         # if output ends before input starts - no overlap
    #         if T_out[-1] < T_in[0]:
    #             # populate X_out with copies of X_in[:,0]
    #             X_out[:] = np.reshape(X_in[:,0], (n_states,1))
    #         else:
    #             # sliding overlap
    #             start_idx = np.abs(T_in[0] - T_out).argmin()
    #             end_idx = np.abs(T_out[-1] - T_in).argmin()
    #             X
    #     else:
    #         # input contained in output

    # # initialize output 
    # n_states = X_in.shape[0]
    # n_t_out = len(T_out)
    # X_out = np.zeros((n_states, n_t_out))
    # # find matching idx of T_in that matches T_out[0]
    # start_idx = int(np.where(T_in==T_out[0])[0])
    # # amount of overlap between T_in and T_out
    # len_overlap = min(len(T_out), len(T_in[start_idx:]))
    # # fill in overlap with X_in entries
    # X_out[:,:len_overlap] = X_in[:,start_idx:start_idx+len_overlap]
    # # fill in remainder (if any) with copies of final entry of X_in
    # if len_overlap < n_t_out:
    #     X_out[:,len_overlap:] = X_in[:,-1]
    # return X_out


# Return the point in a trajectory closest in time to the provided time
#  time - requested time
#  T    - trajectory time vector
#  X    - trajectory state vector
def trajectory_closest_point(time, T, X):
    idx = np.abs(time - T).argmin()
    return X[:,idx]