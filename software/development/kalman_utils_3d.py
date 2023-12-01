# Ideas for Kalman

# 1. Hidden Variable
# Use position measurements to calculate velocity
# Use Velocity as a variable to adjust for position (a hidden variable)
# Process error is likely the error from just moving the machine around 

# 2. Sensor Fusion
# Use the accelerometer for acceleration measurements or velocity which can be fed into the position Kalman for more certainty
# Not sure if the position can be fed into the Kalman for accelerometer for better results - will investigate

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np

def PE_filter(x, P, R, Q, dt):
    """ Returns a KalmanFilter which implements a
    constant acceleration model for a state [x dx].T
    """
    kf = KalmanFilter(dim_x=3, dim_z=2)
    kf.x = np.array([x[0], x[1], x[2]]) # position, velocity, acceleration
    kf.F = np.array([[1., dt, 1/2*(dt**2)], 
                     [0., 1., dt], 
                     [0., 0., 1.]]) # State Transition matrix
    kf.H = np.array([[1., 0., 0.]]) # Measurement function
    kf.R *= R # measurement uncertainty

    if np.isscalar(P):
        kf.P *= P # covariance matrix
    else:
        kf.P[:] = P # [:] makes deep copy
    if np.isscalar(Q):
        kf.Q = Q_discrete_white_noise(dim=3, dt=dt, var=Q)
    else:
        kf.Q[:] = Q

    return kf