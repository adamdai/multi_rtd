# Utilities for planner nodes

import rclpy
import numpy as np

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from multi_rtd_interfaces.msg import RobotTrajectory

def round_to(value, interval):
    """Round value to nearest multiple of interval.

    Parameters
    ----------
    value : float
    interval : float

    Returns
    -------
    float
        Rounded value.
    """
    return round(float(value) / interval) * interval


def check_plan_collision(plan_1, plan_2, r_collision):
    """Check 2 plans against each other to see if they collide.

    Parameters
    ----------
    plan_1 : np.array
    plan_2 : np.array
    r_collision : float

    Returns
    -------
    bool
        True if the plan is safe, False is there is a collision

    """
    time_1 = plan_1[0,:]; traj_1 = plan_1[1:4,:]
    time_2 = plan_2[0,:]
    traj_2 = match_trajectories(time_1, time_2, plan_2[1:4,:])
    d_vec = np.linalg.norm(traj_1 - traj_2, 2, 0)
    if any(d_vec <= r_collision):
        return False
    else:
        return True


def wrap_traj_msg(traj, t2start):
    """Wraps a 3-D trajectory in a JointTrajectory message.

    Parameters
    ----------
    traj : np.array
    t2start : float
        Time to start in seconds

    Returns
    -------
    JointTrajectory 
        Wrapped message.
    """

    p,v,a = traj
    traj_msg = JointTrajectory()

    t2start_sec = int(t2start)
    t2start_nanosec = int(1e9 * (t2start - t2start_sec))

    jtp_x = JointTrajectoryPoint()
    jtp_x.positions = list(p[0])
    jtp_x.velocities = list(v[0])
    jtp_x.accelerations = list(a[0])
    jtp_x.time_from_start = Duration()
    jtp_x.time_from_start.sec = t2start_sec
    jtp_x.time_from_start.nanosec = t2start_nanosec

    jtp_y = JointTrajectoryPoint()
    jtp_y.positions = list(p[1])
    jtp_y.velocities = list(v[1])
    jtp_y.accelerations = list(a[1])
    jtp_y.time_from_start = Duration()
    jtp_y.time_from_start.sec = t2start_sec
    jtp_y.time_from_start.nanosec = t2start_nanosec

    jtp_z = JointTrajectoryPoint()
    jtp_z.positions = list(p[2])
    jtp_z.velocities = list(v[2])
    jtp_z.accelerations = list(a[2])
    jtp_z.time_from_start = Duration()
    jtp_z.time_from_start.sec = t2start_sec
    jtp_z.time_from_start.nanosec = t2start_nanosec

    traj_msg.points = [jtp_x, jtp_y, jtp_z]
    traj_msg.joint_names = ['x','y','z']

    return traj_msg


def wrap_robot_traj_msg(traj, t2start, name):
    """Wraps a 3-D trajectory in a RobotTrajectory message.

    Parameters
    ----------
    traj : np.array
    t2start : float
    name : string

    Returns
    -------
    RobotTrajectory 
        Wrapped message.
    """
    traj_msg = wrap_traj_msg(traj, t2start)
    robot_traj_msg = RobotTrajectory()
    robot_traj_msg.robot_name = name
    robot_traj_msg.trajectory = traj_msg

    return robot_traj_msg


def rand_in_bounds(bounds, n):
    """Generate random sample within specified bounds

    Parameters
    ----------
    bounds : list
        List of min and max values for each dimension.
    n : int
        Number of points to generate.
    """
    x_pts = np.random.uniform(bounds[0], bounds[1], n)
    y_pts = np.random.uniform(bounds[2], bounds[3], n)
    # 2D 
    if len(bounds) == 4:
        return np.vstack((x_pts, y_pts))
    # 3D
    elif len(bounds) == 6:
        z_pts = np.random.uniform(bounds[4], bounds[5], n)
        return np.vstack((x_pts, y_pts, z_pts))
    else:
        raise ValueError('Please pass in bounds as either [xmin xmax ymin ymax] '
                            'or [xmin xmax ymin ymax zmin zmax] ')


def match_trajectories(T_out, T_in, X_in):
    """Resample a trajectory according to new time vector.

    Given an input trajectory and associated time vector, resample according to 
    a desired time and return the resulting trajectory assumes input and output 
    times are sampled at same interval.

    Parameters
    ----------
    T_out : np.array
        Desired times (1 x n_t_out).
    T_in : np.array
        Input times (1 x n_t_in, associated with X_in).
    X_in : np.array
        Input trajectory (n_states x n_t_in).

    Returns
    -------
    np.array
        Resampled trajectory (n_states x n_t_in).

    """
    # initialize output 
    n_states = X_in.shape[0]
    n_t_out = len(T_out)
    X_out = np.zeros((n_states, n_t_out))
    for i in range(n_t_out):
        X_out[:,i] = trajectory_closest_point(T_out[i], T_in, X_in)
    return X_out


def trajectory_closest_point(time, T, X):
    """Return the point the trajectory closest in time to the provided time.

    Parameters
    ----------
    time : float
        Requested time.
    T : np.array
        Trajectory time vector.
    X : np.array
        Trajectory state vector.

    Returns
    -------
    np.array
        Closest point (state).

    """
    idx = np.abs(time - T).argmin()
    return X[:,idx]