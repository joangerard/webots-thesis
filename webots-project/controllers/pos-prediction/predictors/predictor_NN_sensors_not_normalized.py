from keras import models
from keras import layers
import numpy as np
import math
import pickle
import os.path
from keras.models import load_model
import pandas as pd


class PredictorNNSensorsNotNormalized:
    def __init__(self):
        # define set of models
        self.models = []

        # load all the models from memory
        for i in range(1, 9):
            self.models.append(load_model('models/nn-dev-complex-2/nn_sensor_{0}.h5'.format(i)))

    def get_particles_weight(self, particles_state, sensors, p=1):
        """
        Given the particles state and robot sensor measurements it returns the weight of each particle.

        :param particles_state: [[x1,y1,theta1],[x2,y2,theta2],[x3,y4,theta4], ..., [x1000,y1000,theta1000]]
        :param sensors: [z1, z2, z3, z4,..., z8]
        :param p: hyper parameter
        :return: weights: [w1,w2,w3,..., w1000]
        """
        predicted_sensor_measurements = []

        # predict the sensor measurements (8 sensors) for every particle
        for i in range(1, 9):
            predicted_sensor_measurements.append(
                self.models[i-1].predict(particles_state))

        err = 0
        bad_data = True

        # calculate error: how far the predicted values are from the real sensor measurements
        for ix, elem in enumerate(predicted_sensor_measurements):
            if not math.isnan(sensors[ix]):
                bad_data = False
                err += np.absolute(elem - sensors[ix]) ** p

        # the bigger the error, the less the weight of the particle and thus the far from the real state
        weight = 1/err
        weight = weight.reshape((weight.shape[0],))

        return weight, bad_data

    def refit_models(self, x, y, theta, sensors):
        for i in range(1, 9):
            self.models[i-1].fit(np.array([[x, y, theta]]), np.array([sensors[i - 1]]), epochs=1, verbose=0)
