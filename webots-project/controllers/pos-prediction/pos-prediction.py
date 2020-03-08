"""obstacle_avoid_test controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor
from controller import Supervisor
from odometry import Odometry
from data_collector import DataCollector
from movement_controller import MovementController
from predictor_NN import PredictorNN
from predictor import Predictor
from predictor_online import PredictorOnline
from window_communicator import WindowCommunicator
from robot_configuration import RobotConfiguration
from environment_configuration import EnvironmentConfiguration
from particles_filter import ParticlesFilter
from predictor_NN_sensors import PredictorNNSensors
from predictor_NN_coordinates import PredictorNNCoordinates
import matplotlib.pyplot as plt
import numpy as np
import math
import pickle

np.random.seed(13482736)
# os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

# hello = tf.constant("hello TensorFlow!")
# sess=tf.Session()
# print(sess.run(hello))

MAX_SPEED = 6
TIME_STEP = 8
WHEEL_RADIUS = 0.05
SAMPLING_PERIOD = 10
MAX_X = 2
MAX_Y = 1.5
ENCODER_UNIT = 159.23
INIT_X = 1
INIT_Y = 0.75
INIT_ANGLE = np.pi
PRED_STEPS = 1
CAPTURING_DATA = False
MOVING_ROBOT_STEPS = 100
correction_x = 0
correction_y = 0
correction_theta = 0

# create the Robot instance.

robot = Supervisor()
robot_sup = robot.getFromDef("e-puck")
robot_trans = robot_sup.getField("translation")
compass = robot.getCompass("compass")
motorLeft = robot.getMotor("left wheel motor")
motorRight = robot.getMotor("right wheel motor")

positionLeft = robot.getPositionSensor("left wheel sensor")
positionRight = robot.getPositionSensor("right wheel sensor")

timestep = int(robot.getBasicTimeStep())

x = []
y = []
theta = []
distance_sensors_info = []

x_odometry = []
y_odometry = []
theta_odometry = []
sensorNames = ['ds0', 'ds1', 'ds2', 'ds3', 'ds4', 'ds5', 'ds6', 'ds7']

x_pred = []
y_pred = []
theta_pred = []

data_collector = DataCollector()
movement_controller = MovementController()
window_communicator = WindowCommunicator(robot)
# predictor = PredictorOnline(data_collector)
# predictor = PredictorNN(data_collector)
predictor = Predictor()
# predictor = PredictorNNSensors()
predictorCoord = PredictorNNCoordinates()

movement_random = True


def init():
    compass.enable(timestep)
    # motorLeft.setPosition(0.5/WHEEL_RADIUS)
    # motorRight.setPosition(0.5/WHEEL_RADIUS)
    motorLeft.setPosition(float('inf'))
    motorRight.setPosition(float('inf'))
    positionRight.enable(timestep)
    positionLeft.enable(timestep)

def robot_to_xy(x, y):
    return x+1, y+0.75
    # return x, y

def xy_to_robot(x, y):
    return x-1, y-0.75
    # return x, y

def get_bearing_degrees():
    north = compass.getValues()
    rad = np.arctan2(north[0], north[2])
    bearing = (rad) / np.pi * 180
    if bearing < 0.0:
        bearing += 360
    bearing = 360 - bearing - 90
    if bearing < 0.0:
        bearing += 360
    return bearing

def step():
    return (robot.step(timestep) != -1)

def save_supervisor_coordinates():
    # true robot position information
    trans_info = robot_trans.getSFVec3f()
    # print('SUP COORD:', trans_info)
    x_coordinate, y_coordinate = robot_to_xy(trans_info[2], trans_info[0])
    x.append(x_coordinate)
    y.append(y_coordinate)
    angle = get_bearing_degrees()
    theta.append(angle)

def save_odometry_coordinates(coordinate):
    # convert robot coordinates into global coordinate system
    x_odometry.append(coordinate.x + correction_x)
    y_odometry.append(coordinate.y + correction_y)
    theta_odometry.append(convert_angle_to_xy_coordinates(coordinate.theta) + correction_theta)

def save_sensor_distances(distanceSensors):
    distances = []
    for distanceSensor in distanceSensors:
        distance = distanceSensor.getValue()

        #there is no real messure.
        if distance == 10:
            distance = None
        distances.append(distance)
    distance_sensors_info.append(distances)


def get_sensor_distance():
    # Read the sensors, like:
    distanceSensors = []

    for sensorName in sensorNames:
        sensor = robot.getDistanceSensor(sensorName)
        sensor.enable(timestep)
        distanceSensors.append(sensor)
    return distanceSensors


def are_there_sensor_measurements(distanceSensors):
    there_are = True
    for sensor in distanceSensors:
        if math.isnan(sensor.getValue()):
            there_are = False

    return there_are


def calculate_velocity_random(distanceSensors):
    """

    :param distanceSensors:
    :return: right,left speeds

    This method is in charge of moving the robot in a non-deterministic way
    """

    sensorValues = [distanceSensor.getValue() + np.random.normal(0, 0.1) for distanceSensor in distanceSensors]


def convert_angle_to_xy_coordinates(angle):
    angle = angle*180/np.pi
    if angle < 0.0:
        angle = 360 + angle
    return angle


def plot():
    # Enter here exit cleanup code.
    plt.ylim([0, MAX_Y])
    plt.xlim([0, MAX_X])
    plt.xlabel("x")
    plt.ylabel("y")
    plt.plot(x, y, label="real")
    plt.plot(x_odometry, y_odometry, label="odometry")
    plt.plot(x_pred, y_pred, 's', label="correction", marker='o')
    plt.title("Robot position estimation")
    plt.legend()
    plt.savefig("results/position.eps", format='eps')


def predict(x, y, theta, sensors_data):
    predictions = []
    errors = []
    interval = 4
    interval_angle = 4

    xrange = [l/100 for l in range(max(0, int(x*100) - interval), min(MAX_X*100, int(x*100) + interval), 2)]
    yrange = [l/100 for l in range(max(0, int(y*100) - interval), min(int(MAX_Y*100), int(y*100) + interval), 2)]
    thetarange = [l for l in range(max(0, int(theta) - interval_angle), min(360, int(theta) + interval_angle), 1)]

    print("XRANGE------------------")
    print(x)
    print(xrange)

    print("YRANGE------------------")
    print(y)
    print(yrange)

    print("THETARANGE------------------")
    print("theta: ", theta)
    print(thetarange)

    for i in xrange:
        for j in yrange:
            for k in thetarange:
                est, bad_data = predictor.predict(i, j, k, sensors_data)
                if not bad_data:
                    predictions.append([i, j, k])
                    errors.append(est)

    if len(errors) > 0:
        ix = errors.index(min(errors))
        return predictions[ix]

    return -1


def move_robot_to_random_position():
    # new_x = -(1/2 * MAX_X - 0.1) + np.random.random() * (MAX_X - 0.2)
    # new_y = -(1/2 * MAX_Y - 0.1) + np.random.random() * (MAX_Y - 0.2)
    # old_z = robot_trans.getSFVec3f()[1]

    # new_theta = -np.pi + np.random.random() * 2 * np.pi

    # print('New Values: ', new_x, new_y, old_z)
    robot_sup = robot.getFromDef("e-puck")
    robot_trans = robot_sup.getField("translation")
    robot_trans.setSFVec3f([0, 0, 0])


if __name__ == '__main__':
    init()
    step()
    odometry = Odometry(ENCODER_UNIT * (positionLeft.getValue()),
                        ENCODER_UNIT * (positionRight.getValue()), INIT_X, INIT_Y, INIT_ANGLE)

    count = 0
    particles = np.array([[], []])
    last_move = 'none'
    apply_movement = True
    delta_movement = np.array([0, 0, 0])

    robot_initial_conf = RobotConfiguration(INIT_X, INIT_Y, convert_angle_to_xy_coordinates(INIT_ANGLE))
    environment_conf = EnvironmentConfiguration(MAX_X, MAX_Y)
    particles_filter = ParticlesFilter(environment_conf, robot_initial_conf, predictor)

    errorPos = []
    cont = True

    while(cont):
        if count == 2000:
            cont = False

        print(count)

        # receive message
        message = window_communicator.receiveMessage()
        if message == 'start_randomness':
            movement_random = True
        elif message == 'stop_randomness':
            movement_random = False

        odometry_info, delta_movement = odometry.track_step(ENCODER_UNIT * (positionLeft.getValue()),
                                            ENCODER_UNIT * (positionRight.getValue()))

        delta_movement[2] = convert_angle_to_xy_coordinates(delta_movement[2])

        # for capturing robot positioning
        if not step() and CAPTURING_DATA:
            print('saving data')
            data_collector.collect(x, y, theta, np.array(distance_sensors_info))
            plot()

        # print('Compass: ', get_bearing_degrees(), 'Odometry:', convert_angle_to_xy_coordinates(odometry_info.theta))

        distanceSensors = get_sensor_distance()

        # collect data
        save_sensor_distances(distanceSensors)
        save_odometry_coordinates(odometry_info)
        save_supervisor_coordinates()


        # calculate new velocity
        # left_speed, right_speed = movement_controller.calculate_velocity(distanceSensors)
        left_speed, right_speed = 0, 0
        if movement_random:
            left_speed, right_speed = movement_controller.calculate_velocity(distanceSensors)
            last_move = 'none'
        else:
            if last_move == 'none' or (message and last_move != message):
                last_move = message
            if last_move == 'UP':
                left_speed, right_speed = movement_controller.move_straight()
            elif last_move == 'DOWN':
                left_speed, right_speed = movement_controller.move_backwards()
            elif last_move == 'RIGHT':
                left_speed, right_speed = movement_controller.move_right()
            elif last_move == 'LEFT':
                left_speed, right_speed = movement_controller.move_left()

        motorLeft.setVelocity(left_speed)
        motorRight.setVelocity(right_speed)

        # correction step each PRED_STEPS steps
        if not CAPTURING_DATA and count % PRED_STEPS == 0 and count != 0:
            particles = particles_filter.get_particles(delta_movement, distance_sensors_info[-1], count % 3 == 0)

            weighted_sum = np.sum(particles[3])

            # get weighted average from particles data
            x_prim = np.sum(particles[0]*particles[3])/weighted_sum
            y_prim = np.sum(particles[1]*particles[3])/weighted_sum
            theta_prim = np.sum(particles[2]*particles[3])/weighted_sum

            # # get the position prediction given the sensor measurements
            # predicted_coord = predictorCoord.predict(distance_sensors_info[-1])
            #
            # # combine both previous models
            # x_pred.append((x_prim + float(predicted_coord[['x']]))/2)
            # y_pred.append((y_prim + float(predicted_coord[['y']]))/2)
            # theta_pred.append((theta_prim + float(predicted_coord[['theta']]))/2)

            x_pred.append(x_prim)
            y_pred.append(y_prim)
            theta_pred.append(theta_prim)

            # print('x: {0}, y: {1}, theta: {2} | real_x: {3}, real_y: {4}, real_theta: {5} | weight: {6}'
            #       .format(x_pred[-1], y_pred[-1], theta_pred[-1], x[-1], y[-1], theta[-1], weighted_sum))

            # calculate correction
            correction_x += (x_pred[-1] - x_odometry[-1])
            correction_y += (y_pred[-1] - y_odometry[-1])
            correction_theta += (theta_pred[-1] - theta_odometry[-1])

        # angleDiff = 180 - abs(abs(theta[-1] - theta_odometry[-1]) - 180)
        # if (angleDiff > 100):
        #     print("Theta: {0}; Theta Odometry: {1}".format(theta[-1], theta_odometry[-1]))

        errorPos.append(np.sqrt((x[-1] - x_odometry[-1]) ** 2 + (y[-1] - y_odometry[-1]) ** 2))

        # send data to html page
        # window_communicator.sendCoordinatesParticles(x, y, x_odometry, y_odometry,  particles.tolist())


        # move robot to a random position after a while
        # if CAPTURING_DATA and count % MOVING_ROBOT_STEPS == 0:
        #     move_robot_to_random_position()

        count += 1

    print('saving error')
    pickle.dump(errorPos, open("data_rf_500.pckl", "wb"))
