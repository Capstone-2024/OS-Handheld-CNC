# Ideas for Kalman

# 1. Hidden Variable
# Use position measurements to calculate velocity
# Use Velocity as a variable to adjust for position (a hidden variable)
# Process error is likely the error from just moving the machine around 

# 2. Sensor Fusion
# Use the accelerometer for acceleration measurements or velocity which can be fed into the position Kalman for more certainty
# Not sure if the position can be fed into the Kalman for accelerometer for better results - will investigate

from filterpy.kalman import KalmanFilter, predict, update
from filterpy.common import Q_discrete_white_noise
import numpy as np

# 1. Initialize belief of the system
# - initialize state of the filter
x_x = np.array([0., 0]) # x state, pos and velocity
x_y = np.array([0., 0]) # y state, pos and velocity

# - initialize our belief in the state (use first measurement)
P_x = np.diag([1.5**2., 50/3**2]) # covariance matrix
P_y = np.diag([1.5**2., 50/3**2]) # covariance matrix
# assuming 0 correlation between position and velocity to begin with
# std of pose estimation is likely around 1.5 mm, so about 1.5**2 mm^2 variance
# max speed user can move is 50mm/s, if three standard deviations out, then 50/3**2 variance

# loop start here
# 2. Predict
# - use process model to predict the state at the next time step
# linear equations are used
F = np.array([[1., dt],[0, 1]])

# - adjust belief to account for the uncertainty in prediction
Q = Q_discrete_white_noise(dim=2, dt=dt, var=0.5**2)
# Q should also be added to f(x), this will be done later...
B = 0 # no control model
u = 0 # no control input
x_x, P_x = predict(x=x_x, P=P_x, F=F, Q=Q, B=B, u=u)


# 3. Update
# - compute residual  
H_x = [1. , 0] # only position is needed

# - get measurement and associated belief about its accuracy
# We need to design H
z = pose_estimation()
R_x = np.array([1.5**2])

x_x, P_x = update(x=x_x, P=P_x, z=z, R=R_x, H=H_x)


# - compute scaling factor based on whether the measurement or prediction is more accurate
# - set state between the prediction and measurement based on scaling factor
# - update belief in the state based on how certain we are in the measurement