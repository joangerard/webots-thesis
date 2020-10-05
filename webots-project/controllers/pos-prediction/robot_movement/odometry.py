from robot_movement.wheels_state import WheelsState
from robot_movement.coordinate import Coordinate
import numpy as np


class Odometry:
    def __init__(self, pos_left, pos_right, x=0, y=0, angle=0):
        self.increments_per_tour = 1000
        # self.axis_wheel_ratio = 1.660
        self.axis_wheel_ratio = 1.885
        # self.axis_wheel_ratio = 1.935
        self.wheel_diameter_left = 0.05
        self.wheel_diameter_right = 0.05
        self.scaling_factor = 1.01
        # self.scaling_factor = .8662
        self.wheels_state = WheelsState()
        # angle adjustment to coordinates x, y
        self.coordinate = Coordinate(x+0.01190, y-0.0001642675221653, angle - 0.08325221)

        self.wheels_state.pos_left_prev = int(pos_left)
        self.wheels_state.pos_right = int(pos_right)

        # print(pos_left, pos_right)

        self.wheel_distance = self.axis_wheel_ratio * self.scaling_factor * (self.wheel_diameter_left
                                                                             + self.wheel_diameter_right) / 2
        self.wheel_conversion_left = self.wheel_diameter_left * self.scaling_factor * np.pi / self.increments_per_tour
        self.wheel_conversion_right = self.wheel_diameter_right * self.scaling_factor * np.pi / self.increments_per_tour

    def track_step(self, pos_left, pos_right):
        # self.wheel_distance = self.axis_wheel_ratio * self.scaling_factor * (self.wheel_diameter_left
        #                                                                      + self.wheel_diameter_right) / 2
        # self.wheel_conversion_left = self.wheel_diameter_left * self.scaling_factor * np.pi / self.increments_per_tour
        # self.wheel_conversion_right = self.wheel_diameter_right * self.scaling_factor * np.pi / self.increments_per_tour

        delta_pos_left = int(pos_left - self.wheels_state.pos_left_prev)
        delta_pos_right = int(pos_right - self.wheels_state.pos_right_prev)

        delta_left = delta_pos_left * self.wheel_conversion_left
        delta_right = delta_pos_right * self.wheel_conversion_right
        delta_theta = (delta_right - delta_left) / self.wheel_distance

        theta2 = self.coordinate.theta + delta_theta * 0.5
        delta_x = (delta_left + delta_right) * 0.5 * np.cos(theta2)
        delta_y = (delta_left + delta_right) * 0.5 * np.sin(theta2)

        self.coordinate.x += delta_x
        self.coordinate.y += delta_y
        self.coordinate.theta += delta_theta

        if self.coordinate.theta > np.pi:
            self.coordinate.theta -= 2 * np.pi
        if self.coordinate.theta < - np.pi:
            self.coordinate.theta += 2 * np.pi
        self.wheels_state.pos_left_prev = pos_left
        self.wheels_state.pos_right_prev = pos_right

        return self.coordinate, np.array([delta_x, delta_y, delta_theta])
