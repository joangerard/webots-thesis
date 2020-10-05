"""obstacle_avoid_test controller."""

from robot_manager import RobotManager
from args_manager import ArgsManager
import numpy as np

np.random.seed(13482737)
# np.random.seed(13482736)
argsManager = ArgsManager()

robotManager = RobotManager(argsManager.process_args())
robotManager.execute()

