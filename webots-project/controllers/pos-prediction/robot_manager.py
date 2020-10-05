from controller import Supervisor
from robot_movement.odometry import Odometry
from data_collector.data_collector import DataCollector
from robot_movement.movement_controller import MovementController
from robot_window.window_communicator import WindowCommunicator
from particles_filter.robot_configuration import RobotConfiguration
from particles_filter.environment_configuration import EnvironmentConfiguration
from particles_filter.particles_filter import ParticlesFilter
from params import Params
from predictors.predictor_NN_coordinates import PredictorNNCoordinates
from predictors.predictor_NN_sensors_not_normalized import PredictorNNSensorsNotNormalized
import matplotlib.pyplot as plt
import numpy as np
import math
import pickle
import json


class RobotManager:
    def __init__(self, params):
        self.robot = Supervisor()
        self.robot_sup = self.robot.getFromDef("e-puck")
        self.robot_trans = self.robot_sup.getField("translation")
        self.robot_rotation = self.robot_sup.getField("rotation")
        self.compass = self.robot.getCompass("compass")
        self.motorLeft = self.robot.getMotor("left wheel motor")
        self.motorRight = self.robot.getMotor("right wheel motor")

        self.positionLeft = self.robot.getPositionSensor("left wheel sensor")
        self.positionRight = self.robot.getPositionSensor("right wheel sensor")
        self.params = params

        # real robot state
        self.x = []
        self.y = []
        self.theta = []
        self.distance_sensors_info = []

        # odometry estimation
        self.x_odometry = []
        self.y_odometry = []
        self.theta_odometry = []
        self.sensorNames = ['ds0', 'ds1', 'ds2', 'ds3', 'ds4', 'ds5', 'ds6', 'ds7']

        # predicted data
        self.x_pred = []
        self.y_pred = []
        self.theta_pred = []

        self.data_collector = DataCollector()
        self.movement_controller = MovementController()
        self.window_communicator = WindowCommunicator(self.robot)

        # predictors
        self.predictor = PredictorNNSensorsNotNormalized()
        self.predictorCoord = PredictorNNCoordinates()

        self.timestep = int(self.robot.getBasicTimeStep())

        # particles filter initialization
        robot_initial_conf = RobotConfiguration(self.params.INIT_X, self.params.INIT_Y,
                                                self.convert_angle_to_xy_coordinates(self.params.INIT_ANGLE))
        environment_conf = EnvironmentConfiguration(self.params.MAX_X, self.params.MAX_Y)

        self.particles_filter = ParticlesFilter(environment_conf, robot_initial_conf, self.predictor, self.params)

        self.movement_random = True

        pass

    def init_actuators(self):
        self.compass.enable(self.timestep)
        self.motorLeft.setPosition(float('inf'))
        self.motorRight.setPosition(float('inf'))
        self.positionRight.enable(self.timestep)
        self.positionLeft.enable(self.timestep)

    def robot_to_xy(self, x, y):
        return x+1.5, y+1.5

    def xy_to_robot(self, x, y):
        return x-1.5, y-1.5

    def init_robot_pos(self, x, y):
        x, y = self.xy_to_robot(x, y)
        self.robot_trans.setSFVec3f([y, 0, x])
        self.robot_rotation.setSFRotation([0, 1, 0, 0])
        self.robot_sup.resetPhysics()

    def get_bearing_degrees(self):
        north = self.compass.getValues()
        rad = np.arctan2(north[0], north[2])
        bearing = (rad) / np.pi * 180
        if bearing < 0.0:
            bearing += 360
        bearing = 360 - bearing - 90
        if bearing < 0.0:
            bearing += 360
        return bearing

    def save_supervisor_coordinates(self):
        # true robot position information
        trans_info = self.robot_trans.getSFVec3f()
        # print('SUP COORD:', trans_info)
        x_coordinate, y_coordinate = self.robot_to_xy(trans_info[2], trans_info[0])
        self.x.append(x_coordinate)
        self.y.append(y_coordinate)
        angle = self.get_bearing_degrees()
        self.theta.append(angle)

    def step(self):
        return self.robot.step(self.timestep) != -1

    def save_odometry_coordinates(self, coordinate):
        # convert robot coordinates into global coordinate system
        self.x_odometry.append(coordinate.x)
        self.y_odometry.append(coordinate.y)
        self.theta_odometry.append(self.convert_angle_to_xy_coordinates(coordinate.theta))

    def save_sensor_distances(self, distanceSensors):
        distances = []
        for distanceSensor in distanceSensors:
            distance = distanceSensor.getValue()

            #there is no real messure.
            if distance == 10:
                distance = None
            distances.append(distance)
        self.distance_sensors_info.append(distances)

    def get_sensor_distance(self):
        # Read the sensors, like:
        distanceSensors = []

        for sensorName in self.sensorNames:
            sensor = self.robot.getDistanceSensor(sensorName)
            sensor.enable(self.timestep)
            distanceSensors.append(sensor)

        return distanceSensors

    def convert_angle_to_xy_coordinates(self, angle):
        angle = angle*180/np.pi
        if angle < 0.0:
            angle = 360 + angle

        return angle

    def plot(self):
        # Enter here exit cleanup code.
        plt.ylim([0, self.params.MAX_Y])
        plt.xlim([0, self.params.MAX_X])
        plt.xlabel("x")
        plt.ylabel("y")
        plt.plot(self.x, self.y, label="real")
        plt.plot(self.x_odometry, self.y_odometry, label="odometry")
        plt.plot(self.x_pred, self.y_pred, 's', label="correction", marker='o')
        plt.title("Robot position estimation")
        plt.legend()
        plt.savefig("results/position.eps", format='eps')


    def move_robot_to_random_position(self):
        new_x = -(1/2 * self.params.MAX_X - 0.1) + np.random.random() * (self.params.MAX_X - 0.2)
        new_y = -(1/2 * self.params.MAX_Y - 0.1) + np.random.random() * (self.params.MAX_Y - 0.2)
        # old_z = robot_trans.getSFVec3f()[1]

        new_theta = np.random.random() * 2 * np.pi

        self.robot_trans.setSFVec3f([new_y, 0, new_x])
        self.robot_rotation.setSFRotation([0, 1, 0, new_theta])
        self.robot_sup.resetPhysics()

    def force_move(self, step):
        """
        Path for the small arena 2x1.5
        :param step:
        :return:
        """
        if step == 1:
            return '{"code": "stop_randomness"}'
        if step == 2:
            return '{"code": "move", "direction": "UP"}'
        if step == 200:
            return '{"code": "move", "direction": "RIGHT"}'
        if step == 230:
            return '{"code": "move", "direction": "UP"}'
        if step == 400:
            return '{"code": "move", "direction": "RIGHT"}'
        if step == 450:
            return '{"code": "move", "direction": "UP"}'
        if step == 800:
            return '{"code": "move", "direction": "LEFT"}'
        if step == 820:
            return '{"code": "move", "direction": "UP"}'
        if step == 1000:
            return '{"code": "move", "direction": "LEFT"}'
        if step == 1030:
            return '{"code": "move", "direction": "UP"}'
        if step == 1200:
            return '{"code": "move", "direction": "LEFT"}'
        if step == 1250:
            return '{"code": "move", "direction": "UP"}'
        if step == 1700:
            return '{"code": "move", "direction": "LEFT"}'
        if step == 1750:
            return '{"code": "move", "direction": "UP"}'
        return None

    def force_move_complex(self, step):
        """
        Path for the bigger arena of 3x3 not symetric: world called 'complex'
        :param step:
        :return:
        """
        if step == 1:
            return '{"code": "stop_randomness"}'
        if step == 2:
            return '{"code": "move", "direction": "LEFT"}'
        if step == 35:
            return '{"code": "move", "direction": "UP"}'
        if step == 950:
            return '{"code": "move", "direction": "LEFT"}'
        if step == 982:
            return '{"code": "move", "direction": "UP"}'
        if step == 1400:
            return '{"code": "move", "direction": "LEFT"}'
        if step == 1432:
            return '{"code": "move", "direction": "UP"}'
        if step == 1900:
            return '{"code": "move", "direction": "RIGHT"}'
        if step == 1933:
            return '{"code": "move", "direction": "UP"}'
        if step == 2300:
            return '{"code": "move", "direction": "RIGHT"}'
        if step == 2333:
            return '{"code": "move", "direction": "UP"}'
        if step == 2700:
            return '{"code": "move", "direction": "LEFT"}'
        if step == 2733:
            return '{"code": "move", "direction": "UP"}'
        if step == 2833:
            return '{"code": "move", "direction": "LEFT"}'
        if step == 2866:
            return '{"code": "move", "direction": "UP"}'
        if step == 3700:
            return '{"code": "move", "direction": "LEFT"}'
        if step == 3730:
            return '{"code": "move", "direction": "UP"}'
        if step == 3830:
            return '{"code": "move", "direction": "LEFT"}'
        if step == 3865:
            return '{"code": "move", "direction": "UP"}'
        if step == 4200:
            return '{"code": "move", "direction": "RIGHT"}'
        if step == 4233:
            return '{"code": "move", "direction": "UP"}'
        if step == 4500:
            return '{"code": "move", "direction": "RIGHT"}'
        if step == 4532:
            return '{"code": "move", "direction": "UP"}'
        if step == 4850:
            return '{"code": "move", "direction": "LEFT"}'
        if step == 4883:
            return '{"code": "move", "direction": "UP"}'
        if step == 5050:
            return '{"code": "move", "direction": "LEFT"}'
        if step == 5082:
            return '{"code": "move", "direction": "UP"}'
        # if step == 6000:
        #     return '{"code": "move", "direction": "RIGHT"}'
        # if step == 6032:
        #     return '{"code": "move", "direction": "UP"}'
        # if step == 6250:
        #     return '{"code": "move", "direction": "RIGHT"}'
        # if step == 6282:
        #     return '{"code": "move", "direction": "UP"}'
        return None

    def force_big_complex_2(self, step):
        """
        Path for the symetric arena 3x3: world called 'complex-2'
        :param step:
        :return:
        """
        if step == 1:
            return '{"code": "stop_randomness"}'
        if step == 2:
            return '{"code": "move", "direction": "LEFT"}'
        if step == 65:
            return '{"code": "move", "direction": "UP"}'
        if step == 350:
            return '{"code": "move", "direction": "RIGHT"}'
        if step == 382:
            return '{"code": "move", "direction": "UP"}'
        if step == 600:
            return '{"code": "move", "direction": "RIGHT"}'
        if step == 632:
            return '{"code": "move", "direction": "UP"}'
        if step == 850:
            return '{"code": "move", "direction": "LEFT"}'
        if step == 883:
            return '{"code": "move", "direction": "UP"}'
        if step == 1650:
            return '{"code": "move", "direction": "LEFT"}'
        if step == 1682:
            return '{"code": "move", "direction": "UP"}'
        if step == 2000:
            return '{"code": "move", "direction": "LEFT"}'
        if step == 2033:
            return '{"code": "move", "direction": "UP"}'
        if step == 2600:
            return '{"code": "move", "direction": "RIGHT"}'
        if step == 2632:
            return '{"code": "move", "direction": "UP"}'
        if step == 3000:
            return '{"code": "move", "direction": "RIGHT"}'
        if step == 3032:
            return '{"code": "move", "direction": "UP"}'
        if step == 3600:
            return '{"code": "move", "direction": "LEFT"}'
        if step == 3632:
            return '{"code": "move", "direction": "UP"}'
        if step == 4000:
            return '{"code": "move", "direction": "LEFT"}'
        if step == 4033:
            return '{"code": "move", "direction": "UP"}'
        if step == 4800:
            return '{"code": "move", "direction": "LEFT"}'
        if step == 4832:
            return '{"code": "move", "direction": "UP"}'
        if step == 5080:
            return '{"code": "move", "direction": "RIGHT"}'
        if step == 5112:
            return '{"code": "move", "direction": "UP"}'
        if step == 5300:
            return '{"code": "move", "direction": "RIGHT"}'
        if step == 5332:
            return '{"code": "move", "direction": "UP"}'

    def execute(self):
        self.init_actuators()
        self.init_robot_pos(self.params.INIT_X, self.params.INIT_Y)
        self.step()
        self.init_robot_pos(self.params.INIT_X, self.params.INIT_Y)
        self.window_communicator.sendInitialParams(self.params)

        odometry = Odometry(self.params.ENCODER_UNIT * (self.positionLeft.getValue()),
                            self.params.ENCODER_UNIT * (self.positionRight.getValue()), self.params.INIT_X, self.params.INIT_Y, self.params.INIT_ANGLE)

        count = 0
        particles = np.array([[], []])
        last_move = 'none'



        errorPos = []
        errorOdo = []

        errorPosTheta = []
        errorOdoTheta = []

        continue_running = True

        # principal robot step cycle
        while(continue_running):

            # if the number of steps is greater than EXPERIMENT_DURATION_STEPS then stop running
            if count == self.params.EXPERIMENT_DURATION_STEPS and not self.params.CAPTURING_DATA:
                continue_running = False

            # receive message from robot window
            message = self.window_communicator.receiveMessage()
            # message = self.force_big_complex_2(count)
            message = json.loads(message) if message else None

            if message and message['code'] == 'start_randomness':
                self.movement_random = True
            elif message and message['code'] == 'stop_randomness':
                self.movement_random = False
            elif message and message['code'] == 'params_modification':
                self.particles_filter.reset_particles(message["number_particles"],
                                                      message["sigma_xy"],
                                                      message["sigma_theta"],
                                                      RobotConfiguration(self.x_pred[-1],
                                                                         self.y_pred[-1],
                                                                         self.theta_pred[-1]))

            # odometry.wheel_diameter_right += 0.000007
            # odometry.wheel_diameter_left += 0.000007
            # get odometry data
            odometry_info, delta_movement = odometry.track_step(self.params.ENCODER_UNIT * (self.positionLeft.getValue()),
                                                                self.params.ENCODER_UNIT * (self.positionRight.getValue()))

            # transoform the angle to xy coordinates
            delta_movement[2] = self.convert_angle_to_xy_coordinates(delta_movement[2])

            # for capturing robot positioning
            if not self.step() and self.params.CAPTURING_DATA:
                print('saving data')
                self.data_collector.collect(self.x, self.y, self.theta, np.array(self.distance_sensors_info))
                self.plot()

            # get sensor distances
            distanceSensors = self.get_sensor_distance()

            # collect data: real, odometry, predicted
            self.save_sensor_distances(distanceSensors)
            self.save_odometry_coordinates(odometry_info)
            self.save_supervisor_coordinates()

            # calculate new velocity
            left_speed, right_speed = 0, 0
            if self.movement_random:
                if self.params.GO_STRAIGHT_MOVE:
                    left_speed, right_speed = self.movement_controller.calculate_velocity(distanceSensors)
                else:
                    left_speed, right_speed = self.movement_controller.calculate_velocity_random_move(distanceSensors)
                last_move = 'none'
            else:
                # joystick control
                if message and message['code'] == "move" and last_move != message['direction']:
                    last_move = message['direction']
                if last_move == 'UP':
                    left_speed, right_speed = self.movement_controller.move_straight()
                elif last_move == 'DOWN':
                    left_speed, right_speed = self.movement_controller.move_backwards()
                elif last_move == 'RIGHT':
                    left_speed, right_speed = self.movement_controller.move_right()
                elif last_move == 'LEFT':
                    left_speed, right_speed = self.movement_controller.move_left()

            self.motorLeft.setVelocity(left_speed)
            self.motorRight.setVelocity(right_speed)

            # print(self.x[-1], self.y[-1], self.theta[-1])

            # predict position based on particles data
            if not self.params.CAPTURING_DATA and count % self.params.PRED_STEPS == 0 and count != 0:
                # get particles
                particles = self.particles_filter.get_particles(delta_movement, self.distance_sensors_info[-1], count % 2 == 0)

                # get weights sum
                weighted_sum = np.sum(particles[3])

                # get weighted average from particles data
                x_prim = np.sum(particles[0]*particles[3])/weighted_sum
                y_prim = np.sum(particles[1]*particles[3])/weighted_sum
                theta_prim = np.sum(particles[2]*particles[3])/weighted_sum

                # # get the position prediction given the sensor measurements
                # predicted_coord = predictorCoord.predict(distance_sensors_info[-1])
                #
                # print(predicted_coord)
                # # combine both previous models
                # x_pred.append((x_prim + float(predicted_coord[0]))/2)
                # y_pred.append((y_prim + float(predicted_coord[1]))/2)

                self.x_pred.append(x_prim)
                self.y_pred.append(y_prim)
                self.theta_pred.append(theta_prim)

                # self.predictor.refit_models(self.x_pred[-1], self.y_pred[-1], self.theta_pred[-1], self.distance_sensors_info[-1])

                # calculate error
                if self.params.CALCULATE_PRED_ERROR:
                    errorPos.append(np.sqrt((self.x[-1] - self.x_pred[-1]) ** 2 + (self.y[-1] - self.y_pred[-1]) ** 2))
                    errorPosTheta.append(np.sqrt((self.theta[-1] - self.theta_pred[-1]) ** 2))

            if self.params.CALCULATE_ODO_ERROR:
                errorOdo.append(np.sqrt((self.x[-1] - self.x_odometry[-1]) ** 2 + (self.y[-1] - self.y_odometry[-1]) ** 2))
                errorOdoTheta.append(np.sqrt((self.theta[-1] - self.theta_odometry[-1]) ** 2))

            # send data to html page
            # if count == self.params.EXPERIMENT_DURATION_STEPS:
            if self.params.GLOBAL_LOCALIZATION:
                self.x_odometry = []
                self.y_odometry = []
                self.x_pred = []
                self.y_pred = []

            self.window_communicator.sendCoordinatesParticles(self.x, self.y, self.x_odometry, self.y_odometry, self.x_pred, self.y_pred, particles.tolist())

            # move robot to a random position after a while
            if self.params.CAPTURING_DATA and count % self.params.MOVING_ROBOT_STEPS == 0:
                self.move_robot_to_random_position()

            count += 1

        if self.params.CALCULATE_PRED_ERROR:
            print('Saving prediction SDE')
            pickle.dump([errorPos, errorPosTheta], open(self.params.PRED_ERROR_FILE, "wb"))

        if self.params.CALCULATE_ODO_ERROR:
            print('Saving odometry SDE')
            pickle.dump([errorOdo, errorOdoTheta], open(self.params.ODO_ERROR_FILE, "wb"))
