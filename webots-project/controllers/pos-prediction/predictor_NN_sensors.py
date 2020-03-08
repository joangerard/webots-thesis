from keras import models
from keras import layers
import numpy as np
import math
import pickle
import os.path
from keras.models import load_model
import pandas as pd


class PredictorNNSensors:
    def __init__(self):
        data = pd.read_csv('results/robot_info_dataset-jumped.csv')

        self.inputs = data[['x', 'y', 'theta']]
        self.output = data[['sensor_1', 'sensor_2', 'sensor_3', 'sensor_4', 'sensor_5', 'sensor_6', 'sensor_7', 'sensor_8']]

        self.models = []
        for i in range(1, 9):
            self.models.append(load_model('models/nn/nn_sensor_{0}.h5'.format(i)))

        self.inputs_max = self.inputs.max()
        self.inputs_min = self.inputs.min()
        self.output_max = self.output.max()
        self.output_min = self.output.min()

        self.input_max_min = self.inputs_max - self.inputs_min
        self.output_max_min = self.output_max - self.output_min

    def normalize_data(self, train_data, test_data):
        mean = train_data.mean(axis=0)
        train_data -= mean
        std = train_data.std(axis=0)
        train_data /= std
        test_data -= mean
        test_data /= std

    def build_model(self, train_data):
        model = models.Sequential()
        model.add(layers.Dense(64, activation='relu',
                               input_shape=(train_data.shape[1],)))
        model.add(layers.Dense(64, activation='relu'))
        model.add(layers.Dense(8))
        model.compile(optimizer='rmsprop', loss='mse', metrics=['mae'])

        return model

    def normalize_inputs(self, inputs):
        return (inputs - self.inputs_min)/self.input_max_min

    def denormalize_output(self, output):
        return output*self.output_max_min + self.output_min

    def prediction_error(self, x, y, theta, sensors):
        features = self.normalize_inputs(np.array([x, y, theta]))

        pre_sensors = []
        for i in range(1, 9):
            pre_sensors.append(self.models[i-1].predict(np.array([features]))[0][0])

        pre_sensors = self.denormalize_output(pre_sensors)

        err = 0
        bad_data = True

        # print(true_dist)
        for ix, elem in enumerate(pre_sensors):
            if not math.isnan(sensors[ix]):
                bad_data = False
                # print('err', elem)
                # print('true', true_dist[ix])
                err += (elem - sensors[ix]) ** 2

        return 1/np.sqrt(err), bad_data
