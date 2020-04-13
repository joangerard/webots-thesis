"""obstacle_avoid_test controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor
from controller import Robot
from controller import Supervisor
from controller import Node
from controller import DifferentialWheels
import matplotlib.pyplot as plt
import tensorflow as tf
import logging
import os
import numpy as np
# os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

# hello = tf.constant("hello TensorFlow!")
# sess=tf.Session()
# print(sess.run(hello))

MAX_SPEED = 6.28
TIME_STEP = 8
WHEEL_RADIUS = 0.0206625
SAMPLING_PERIOD = 10
MAX_X = 2
MAX_Y = 1.5

    
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



# create the Robot instance.
robot = Supervisor()

robot_sup = robot.getFromDef("e-puck")
robot_trans = robot_sup.getField("translation")
compass = robot.getCompass("compass")
compass.enable(SAMPLING_PERIOD)

motorLeft = robot.getMotor("left wheel motor")
motorRight = robot.getMotor("right wheel motor")

motorLeft.setPosition(0.5/WHEEL_RADIUS)
motorRight.setPosition(0.5/WHEEL_RADIUS)
motorLeft.setPosition(float('inf'))
motorRight.setPosition(float('inf'))

timestep = int(robot.getBasicTimeStep())
x = []
y = []

while (robot.step(timestep) != -1):

    # true robot position information
    trans_info = robot_trans.getSFVec3f()
    x_coordinate, y_coordinate = robot_to_xy(trans_info[2], trans_info[0])
    x.append(x_coordinate)
    y.append(y_coordinate)
    print('coordinates: ', trans_info, get_bearing_degrees())

    # Read the sensors, like:
    distanceSensors = []
    sensorNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
    for sensorName in sensorNames:
        sensor = robot.getDistanceSensor(sensorName)
        sensor.enable(timestep)
        distanceSensors.append(sensor)
        
    # Process sensor data here
    sensorValues = [distanceSensor.getValue() for distanceSensor in distanceSensors]
    rightObstacle = sensorValues[0] > 100.0 or sensorValues[1] > 100.0 or sensorValues[2] > 100.0
    leftObstacle = sensorValues[5] > 100.0 or sensorValues[6] > 100.0 or sensorValues[7] > 100.0
  
    left_speed = .5 * MAX_SPEED
    right_speed = .5 * MAX_SPEED
  
    # if leftObstacle:
        # left_speed += .5 * MAX_SPEED
        # right_speed -= .5 * MAX_SPEED
    # elif rightObstacle:
        # left_speed -= .5 * MAX_SPEED
        # right_speed += .5 * MAX_SPEED
      
    motorLeft.setVelocity(left_speed)
    motorRight.setVelocity(right_speed)

  
# Enter here exit cleanup code.
plt.ylim([0, 1.5])
plt.xlim([0, 2])
plt.plot(x, y)
plt.savefig("/Users/sebastiangerard/Documents/Academico/ULB/Classes/Thesis/images/position.png")