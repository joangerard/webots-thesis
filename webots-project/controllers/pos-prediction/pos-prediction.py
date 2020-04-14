"""obstacle_avoid_test controller."""

from robot_manager import RobotManager
from params import Params
import numpy as np

np.random.seed(13482736)
params = Params(1.49999, 0.9999, particles_number=500, sigma_xy=0.01, sigma_theta=20)
robotManager = RobotManager(params)
robotManager.execute()

