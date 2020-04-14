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

    def sendCoordinatesParticles(self, x, y, x_odometry, y_odometry, x_pred, y_pred, particles):
        self.robot.wwiSendText(json.dumps(
            {
                'items': {
                    'x': x,
                    'y': y,
                    'x_odometry': x_odometry,
                    'y_odometry': y_odometry,
                    'x_pred': x_pred,
                    'y_pred': y_pred,
                    'particles_x': particles[0],
                    'particles_y': particles[1]
                }
            }
        ))

    def sendInitialParams(self, params):
        self.robot.wwiSendText(json.dumps(
            {
                'code': 'init',
                'num_particles': params.PARTICLES_NUMBER,
                'sigma_xy': params.SIGMA_XY,
                'sigma_theta': params.SIGMA_THETA
            }
        ))

    def receiveMessage(self):
        text = self.robot.wwiReceiveText()

        return text
