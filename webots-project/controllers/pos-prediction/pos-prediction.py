"""obstacle_avoid_test controller."""

from robot_manager import RobotManager
from params import Params
import numpy as np

np.random.seed(13482736)
params = Params(1.49999, 0.9999)
robotManager = RobotManager(params)
robotManager.execute()

