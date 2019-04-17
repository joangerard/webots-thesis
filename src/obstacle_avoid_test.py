"""obstacle_avoid_test controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor
from controller import Supervisor
from odometry import Odometry
from data_collector import DataCollector
import matplotlib.pyplot as plt
import numpy as np
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

data_collector = DataCollector()

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
    x_odometry.append(1 - coordinate.x)
    y_odometry.append(0.75 - coordinate.y)
    theta_odometry.append(convert_angle_to_xy_coordinates(coordinate.theta))

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


def calculate_velocity(distanceSensors):
    # Process sensor data here
    sensorValues = [distanceSensor.getValue() for distanceSensor in distanceSensors]
    
    rightObstacle = sensorValues[0] < 0.15 or sensorValues[1] < 0.15
    leftObstacle = sensorValues[6] < 0.15 or sensorValues[7] < 0.15
  
    left_speed = .5 * MAX_SPEED
    right_speed = .5 * MAX_SPEED
    # avoid collition
    if leftObstacle:
        left_speed += .7 * MAX_SPEED
        right_speed -= .7 * MAX_SPEED
    elif rightObstacle:
        left_speed -= .7 * MAX_SPEED
        right_speed += .7 * MAX_SPEED
        
    return left_speed, right_speed


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
    plt.title("Robot position estimation")
    plt.legend()
    plt.savefig("results/position.png")


if __name__ == '__main__':
    init()
    step()
    odometry = Odometry(ENCODER_UNIT * (positionLeft.getValue()),
                        ENCODER_UNIT * (positionRight.getValue()))

    while(True):

        odometry_info = odometry.track_step(ENCODER_UNIT * (positionLeft.getValue()),
                                          ENCODER_UNIT * (positionRight.getValue()))

        if not step():
            # print('saving data')
            data_collector.collect(x_odometry, y_odometry, theta_odometry, x, y, theta, np.array(distance_sensors_info))
            plot()

        distanceSensors = get_sensor_distance()

        # collect data
        save_sensor_distances(distanceSensors)
        save_odometry_coordinates(odometry_info)
        save_supervisor_coordinates()

        # calculate new velocity
        left_speed, right_speed = calculate_velocity(distanceSensors)
        motorLeft.setVelocity(left_speed)
        motorRight.setVelocity(right_speed)


