# Dynamical system describing robot


import numpy as np

class Agent:

    def __init__(self):
        # parameters
        self.n_dim = 3 # number of dimensions agent exists in (i.e. 2D or 3D)
        self.v_max = 2 # [m/s] max allowed velocity in any dimension

        # system matrices
        self.A = np.array([])  # dynamics matrix
        self.B = np.array([])  # control matrix
        self.C = np.array([])  # measurement matrix
        self.K = np.array([])  # feedback matrix
        self.Q = np.array([])  # process noise covariance
        self.R = np.array([])  # measurement noise covariance

        # state, measurements, and history
        self.x = np.zeros((self.n_dim*3,1)) # state = (position, velocitiy, acceleration)


    # state update
    def step(self, input):
        pass
