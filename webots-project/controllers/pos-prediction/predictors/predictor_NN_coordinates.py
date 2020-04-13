from keras import models
from keras import layers
import numpy as np
import math
import pickle
import os.path
from keras.models import load_model
import pandas as pd


class PredictorNNCoordinates:
    def __init__(self):

        data = pd.read_csv('results/robot_info_dataset-jumped.csv')

        self.output = data[['x', 'y', 'theta']]
        self.inputs = data[['sensor_1', 'sensor_2', 'sensor_3', 'sensor_4', 'sensor_5', 'sensor_6', 'sensor_7', 'sensor_8']]

        self.model = load_model('models/nn/nn_sensor_reverse.h5')

        self.inputs_max = self.inputs.max()
        self.inputs_min = self.inputs.min()
        self.output_max = self.output.max()
        self.output_min = self.output.min()

    def normalize_inputs(self, inputs):
        return (inputs - self.inputs_min)/(self.inputs_max - self.inputs_min)

    def denormalize_output(self, outputs):
        return outputs*(self.output_max - self.output_min)+self.output_min

    def predict(self, sensors):
        # features = self.normalize_inputs(sensors)
        coordinates = self.model.predict(np.array([sensors]))[0]

        return coordinates
