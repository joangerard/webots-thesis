import json
import numpy as np


class WindowCommunicator:

    def __init__(self, robot):
        self.robot = robot

    def sendCoordinates(self, x, y, x_odometry, y_odometry, x_pred, y_pred):
        self.robot.wwiSendText(json.dumps(
            {
                'items': {
                    'x': x,
                    'y': y,
                    'x_odometry': x_odometry,
                    'y_odometry': y_odometry,
                    'x_pred': x_pred,
                    'y_pred': y_pred
                }
            }))
