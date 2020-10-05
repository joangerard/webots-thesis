import argparse
from params import Params


class ArgsManager:
    def __init__(self):
        self.parser = argparse.ArgumentParser()
        self.parser.add_argument("--initx", type=float,
                            help="Initial X position of robot")

        self.parser.add_argument("--inity", type=float,
                                 help="Initial Y position of robot")

        self.parser.add_argument("--experiment_duration_steps", type=int,
                                 help="Max number of duration steps")

        self.parser.add_argument("-pn", "--particles_number", type=int,
                                 help="Number of particles")

        self.parser.add_argument("--sigma_xy", type=float,
                                 help="Sigma XY that controls the SDE of the x,y coordinates")

        self.parser.add_argument("--sigma_theta", type=float,
                                 help="Sigma Theta that controls the SDE of the robot angle")

        self.parser.add_argument("--calculate_pred_error", action="store_true",
                            help="Store the prediction error in form of .pkl file")

        self.parser.add_argument("--calculate_odo_error", action="store_true",
                                 help="Store the odometry error in form of .pkl file")

        self.parser.add_argument("--pred_error_file",
                                 help="Name of the file where to save the prediction error")

        self.parser.add_argument("--pred_odo_file",
                                 help="Name of the file where to save the odometry error")

        self.parser.add_argument("--go_straight", action="store_true",
                                 help="Let the robot go straight")

        self.parser.add_argument("--capture_data", action="store_true",
                                 help="Capture data mode on")

        self.parser.add_argument("--global_localization", action="store_true",
                                 help="Global localization problem")


    def process_args(self):
        args = self.parser.parse_args()
        init_x=1
        init_y=.75
        experiment_duration_steps=2000
        particles_number=200
        sigma_xy=0.001
        sigma_theta=2
        calculate_pred_error=False
        calculate_odo_error=False
        pred_error_file='data_pred_error.pckl'
        pred_odo_file='data_odo_error.pckl'
        go_straiht_move=False
        capturing_data=False
        global_localization=False

        if args.initx:
            init_x = args.initx
        if args.inity:
            init_y = args.inity
        if args.experiment_duration_steps:
            experiment_duration_steps = args.experiment_duration_steps
        if args.particles_number:
            particles_number = args.particles_number
        if args.sigma_xy:
            sigma_xy = args.sigma_xy
        if args.sigma_theta:
            sigma_theta = args.sigma_theta
        if args.calculate_pred_error:
            calculate_pred_error = args.calculate_pred_error
        if args.calculate_odo_error:
            calculate_odo_error = args.calculate_odo_error
        if args.pred_error_file:
            pred_error_file = args.pred_error_file
        if args.pred_odo_file:
            pred_odo_file = args.pred_odo_file
        if args.go_straight:
            go_straiht_move = args.go_straight
        if args.capture_data:
            capturing_data = args.capture_data
        if args.global_localization:
            global_localization = args.global_localization

        return Params(init_x,
                      init_y,
                      experiment_duration_steps,
                      particles_number,
                      sigma_xy,
                      sigma_theta,
                      calculate_pred_error,
                      calculate_odo_error,
                      pred_error_file,
                      pred_odo_file,
                      go_straiht_move,
                      capturing_data,
                      global_localization)
