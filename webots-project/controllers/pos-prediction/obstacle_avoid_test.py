"""obstacle_avoid_test controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor
from controller import Supervisor
from odometry import Odometry
from data_collector import DataCollector
from movement_controller import MovementController
from predictor import Predictor
from window_communicator import WindowCommunicator
import matplotlib.pyplot as plt
import numpy as np
import math

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
INIT_X = 0.0
INIT_Y = 0.0
INIT_ANGLE = 0
PRED_STEPS = 20
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

predictor = Predictor()

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

def xy_to_robot(x, y):
    return x-1, y-0.75

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
    x_coordinate, y_coordinate = robot_to_xy(trans_info[2], trans_info[0])
    x.append(x_coordinate)
    y.append(y_coordinate)
    theta.append((get_bearing_degrees()))


def save_odometry_coordinates(coordinate):
    # convert robot coordinates into global coordinate system
    x_odometry.append(1 + 2*INIT_X - coordinate.x + correction_x)
    y_odometry.append(0.75 + 2*INIT_Y - coordinate.y + correction_y)
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

def calculate_velocity_random(distanceSensors):
    """

    :param distanceSensors:
    :return: right,left speeds

    This method is in charge of moving the robot in a non-deterministic way
    """

    sensorValues = [distanceSensor.getValue() + np.random.normal(0, 0.1) for distanceSensor in distanceSensors]


def convert_angle_to_xy_coordinates(angle):
    angle = angle*180/np.pi
    angle = angle - 180
    if angle < 0.0:
        angle += 360
    return angle


def plot():
    # Enter here exit cleanup code.
    plt.ylim([0, 1.5])
    plt.xlim([0, 2])
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
    interval = 6
    interval_angle = 6

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

if __name__ == '__main__':
    init()
    step()
    odometry = Odometry(ENCODER_UNIT * (positionLeft.getValue()),
                        ENCODER_UNIT * (positionRight.getValue()), INIT_X, INIT_Y, INIT_ANGLE)

    count = 0



    while(True):

        odometry_info = odometry.track_step(ENCODER_UNIT * (positionLeft.getValue()),
                                            ENCODER_UNIT * (positionRight.getValue()))

        if not step():
            # print('saving data')
            data_collector.collect(x_odometry, y_odometry, theta_odometry, x, y, theta, np.array(distance_sensors_info))
            plot()

        # print('Compass: ', get_bearing_degrees(), 'Odometry:', convert_angle_to_xy_coordinates(odometry_info.theta))

        distanceSensors = get_sensor_distance()

        # collect data
        save_sensor_distances(distanceSensors)
        save_odometry_coordinates(odometry_info)
        save_supervisor_coordinates()

        # calculate new velocity
        # left_speed, right_speed = movement_controller.calculate_velocity(distanceSensors)
        left_speed, right_speed = movement_controller.calculate_velocity_random_move(distanceSensors)

        motorLeft.setVelocity(left_speed)
        motorRight.setVelocity(right_speed)

        # correction step each 100 steps
        if count % PRED_STEPS == 0:
            pred = predict(x_odometry[-1], y_odometry[-1], theta_odometry[-1], distanceSensors)
            if pred != -1:
                # save correction
                x_pred.append(pred[0])
                y_pred.append(pred[1])
                theta_pred.append(pred[2])

                # calculate correction
                correction_x = correction_x + (x_pred[-1] - x_odometry[-1])
                correction_y = correction_y + (y_pred[-1] - y_odometry[-1])
                correction_theta = correction_theta + (theta_pred[-1] - theta_odometry[-1])

        # send data to html page
        window_communicator.sendCoordinates(x, y, x_odometry, y_odometry, x_pred, y_pred)

        count += 1


