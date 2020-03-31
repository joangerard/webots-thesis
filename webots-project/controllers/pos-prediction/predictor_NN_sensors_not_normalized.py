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

        self.models = []
        for i in range(1, 9):
            self.models.append(load_model('models/nn/nn_sensor_{0}.h5'.format(i)))pyt

    def prediction_error(self, x, y, theta, sensors):

        pre_sensors = []
        for i in range(1, 9):
            pre_sensors.append(self.models[i-1].predict(np.array([[x, y, theta]]))[0][0])

        err = 0
        bad_data = True

        # print(true_dist)
        for ix, elem in enumerate(pre_sensors):
            if not math.isnan(sensors[ix]):
                bad_data = False
                err += np.absolute(elem - sensors[ix]) ** 3

        return 1/err, bad_data

    def refit_models(self, x, y, theta, sensors):
        for i in range(1, 9):
            self.models[i-1].fit(np.array([[x, y, theta]]), np.array([sensors[i - 1]]), epochs=50, verbose=0)
