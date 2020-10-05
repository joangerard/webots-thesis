import numpy as np
import random

class MovementController:
    GOING_STRAIGHT = 0
    TURNING_RIGHT = 1
    TURNING_LEFT = 2
    TURN_CAPACITY_MAX = 30

    MAX_SPEED = 6
    THRESHOLD_PROXIMITY = 0.15
    THRESHOLD_PROB_TURN = 0.1

    SENSOR_ERROR = 0.1

    def __init__(self):
        self.status = self.GOING_STRAIGHT
        self.turn_capacity = 500
        self.avoiding_obstacle = False

    def calculate_velocity(self, distanceSensors):
        # Process sensor data here
        sensorValues = [distanceSensor.getValue() for distanceSensor in distanceSensors]

        rightObstacle = sensorValues[0] < 0.15 or sensorValues[1] < 0.15
        leftObstacle = sensorValues[6] < 0.15 or sensorValues[7] < 0.15

        left_speed = .5 * self.MAX_SPEED
        right_speed = .5 * self.MAX_SPEED

        # avoid collition
        if leftObstacle:
            left_speed += .7 * self.MAX_SPEED
            right_speed -= .7 * self.MAX_SPEED
        elif rightObstacle:
            left_speed -= .7 * self.MAX_SPEED
            right_speed += .7 * self.MAX_SPEED

        return left_speed, right_speed

    def move_straight(self):
        left_speed = .5 * self.MAX_SPEED
        right_speed = .5 * self.MAX_SPEED

        return left_speed, right_speed

    def move_left(self):
        left_speed = -.5 * self.MAX_SPEED
        right_speed = .5 * self.MAX_SPEED

        return left_speed, right_speed

    def move_right(self):
        left_speed = .5 * self.MAX_SPEED
        right_speed = -.5 * self.MAX_SPEED

        return left_speed, right_speed

    def move_backwards(self):
        left_speed = -.5 * self.MAX_SPEED
        right_speed = -.5 * self.MAX_SPEED

        return left_speed, right_speed

    def calculate_velocity_random_move(self, distance_sensors):

        left_speed = .5 * self.MAX_SPEED
        right_speed = .5 * self.MAX_SPEED
        # Process sensor data here
        sensorValues = [distanceSensor.getValue() for distanceSensor in distance_sensors]

        rightObstacle = sensorValues[0] < self.THRESHOLD_PROXIMITY or sensorValues[1] < self.THRESHOLD_PROXIMITY
        leftObstacle = sensorValues[6] < self.THRESHOLD_PROXIMITY or sensorValues[7] < self.THRESHOLD_PROXIMITY

        # The robot detected an obstacle so avoid it
        if (rightObstacle or leftObstacle) and not self.avoiding_obstacle:
            self.avoiding_obstacle = True
        # The robot did not detect any obstacle so go ahead
        elif self.avoiding_obstacle and not (rightObstacle or leftObstacle):
            self.avoiding_obstacle = False

        # avoid collition
        if self.avoiding_obstacle:
            # print("[INFO] - avoiding obstacle")
            if leftObstacle:
                left_speed += .7 * self.MAX_SPEED
                right_speed -= .7 * self.MAX_SPEED
            elif rightObstacle:
                left_speed -= .7 * self.MAX_SPEED
                right_speed += .7 * self.MAX_SPEED
        # make a movement: go straight, turn right or turn left
        else:
            if self.status == self.GOING_STRAIGHT:
                # print("[INFO] - going straight")
                # with x% chance turn right or turn left
                p = np.random.uniform(0, 1)

                if 1 - self.THRESHOLD_PROB_TURN < p <= 1 - (self.THRESHOLD_PROB_TURN / 2):
                    # print("[INFO] - TURN RIGHT order")
                    self.status = self.TURNING_RIGHT
                    self.turn_capacity = np.random.randint(self.TURN_CAPACITY_MAX)
                elif 1 - (self.THRESHOLD_PROB_TURN / 2) < p < 1:
                    # print("[INFO] - TURN LEFT order")
                    self.status = self.TURNING_LEFT
                    self.turn_capacity = np.random.randint(self.TURN_CAPACITY_MAX)

            if self.status == self.TURNING_LEFT and self.turn_capacity > 0:
                # print("[INFO]: turning left. Capacity: ", self.turn_capacity)
                left_speed -= .7 * self.MAX_SPEED
                right_speed += .7 * self.MAX_SPEED
                self.turn_capacity -= 1
            if self.status == self.TURNING_RIGHT and self.turn_capacity > 0:
                # print("[INFO]: turning right. Capacity: ", self.turn_capacity)
                left_speed += .7 * self.MAX_SPEED
                right_speed -= .7 * self.MAX_SPEED
                self.turn_capacity -= 1

            # if turn right or left is finished then go straight again
            if self.turn_capacity <= 0:
                self.status = self.GOING_STRAIGHT

        return left_speed, right_speed
