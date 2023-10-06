# https://medium.com/analytics-vidhya/kalman-filters-a-step-by-step-implementation-guide-in-python-91e7e123b968
# Gaussian - a continuous function over the space of locations and the area underneath sums up to 1
# Defined by the mean and the variance

# The filter should be applied on x and y separately


from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise

kf = ExtendedKalmanFilter(dim_x=2, dim_z=2)

kf.x = array([])