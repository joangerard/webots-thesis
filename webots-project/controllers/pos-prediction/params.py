import numpy as np


class Params:
    def __init__(self,
                 init_x=1,
                 init_y=.75,
                 experiment_duration_steps=2000,
                 particles_number=200,
                 sigma_xy=0.001,
                 sigma_theta=2,
                 calculate_pred_error=False,
                 calculate_odo_error=False,
                 pred_error_file='data_pred_error.pckl',
                 pred_odo_file='data_odo_error.pckl',
                 go_straiht_move=False,
                 capturing_data=False,
                 global_localization=False):
        self.MAX_SPEED = 6
        self.TIME_STEP = 8
        self.WHEEL_RADIUS = 0.05
        self.SAMPLING_PERIOD = 10
        self.MAX_X = 3
        self.MAX_Y = 3
        self.ENCODER_UNIT = 159.23
        self.INIT_X = init_x
        self.INIT_Y = init_y
        self.INIT_ANGLE = np.pi
        self.PRED_STEPS = 1
        self.CAPTURING_DATA = capturing_data
        self.MOVING_ROBOT_STEPS = 100
        self.EXPERIMENT_DURATION_STEPS = experiment_duration_steps
        self.CALCULATE_PRED_ERROR = calculate_pred_error
        self.CALCULATE_ODO_ERROR = calculate_odo_error
        self.PRED_ERROR_FILE = pred_error_file
        self.ODO_ERROR_FILE = pred_odo_file
        self.GO_STRAIGHT_MOVE = go_straiht_move
        self.PARTICLES_NUMBER = particles_number
        self.SIGMA_XY = sigma_xy
        self.SIGMA_THETA = sigma_theta
        self.GLOBAL_LOCALIZATION = global_localization
