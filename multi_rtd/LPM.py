# Linear Planning Model Class

import numpy as np
from scipy.io import loadmat

class LPM:
    # Class variables
    t_peak = 0
    t_total = 0
    t_sample = 0
    time = []
    p_mat = []
    v_mat = []
    a_mat = []

    # Construct LPM object from .mat file
    def __init__(self, mat_file):
        # Load the .mat file
        lpm = loadmat(mat_file)
        lpm = lpm['LPM']

        # Extract variables, convert arrays to numpy arrays
        self.t_peak = lpm['t_peak'][0,0][0][0]
        self.t_total = lpm['t_total'][0,0][0][0]
        self.t_sample = lpm['t_sample'][0,0][0][0]
        self.time = np.array(lpm['time'][0,0])[0]
        self.p_mat = np.array(lpm['position'][0,0])
        self.v_mat = np.array(lpm['velocity'][0,0])
        self.a_mat = np.array(lpm['acceleration'][0,0])

    # Compute nominal trajectory from a given trajectory parameter k
    #  k = (v_0, a_0, v_peak), n x 3 where n is workspace dimension
    def compute_trajectory(self, k):
        p = np.dot(k, self.p_mat)
        v = np.dot(k, self.v_mat)
        a = np.dot(k, self.a_mat)
        return p,v,a

    # Given initial velocity and accleration, solve for the peak velocity 
    # which reaches the desired goal position
    # - note that this solution does not account for max velocity constraints
    # - v_0, a_0, and p_goal are all passed in as row vectors
    def solve_trajectory(self, v_0, a_0, p_goal):
        # change to column vectors
        v_0 = np.reshape(v_0, (3,1))
        a_0 = np.reshape(a_0, (3,1))
        # position component due to v_0 and a_0
        p_from_ic = np.dot(np.hstack((v_0, a_0)), self.p_mat[0:2,-1])
        # solve for v_peak
        v_peak = (p_goal - p_from_ic) / self.p_mat[2,-1]
        return v_peak