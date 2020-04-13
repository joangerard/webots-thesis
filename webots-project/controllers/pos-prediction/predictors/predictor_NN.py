from keras import models
from keras import layers
import numpy as np
import math
import pickle
import os.path
from keras.models import load_model


class PredictorNN:
    def __init__(self, data_collector):
        self.percentageTraining = .8

        self.dc = data_collector
        data = self.dc.get_data_frame()

        # delete NA examples
        data = data.dropna()

        # if model exists load it otherwise create it
        if os.path.isfile('train_data_model_NN.h5'):
            self.inputs = data[['x', 'y', 'theta']]
            self.output = data[['sensor_1', 'sensor_2', 'sensor_3', 'sensor_4', 'sensor_5', 'sensor_6', 'sensor_7', 'sensor_8']]

            self.model = load_model('train_data_model_NN.h5')

            self.inputs_max = self.inputs.max()
            self.inputs_min = self.inputs.min()
            self.output_max = self.output.max()
            self.output_min = self.output.min()

    def create_model(self, train_data, train_targets, test_data, test_targets):
        k = 4
        num_val_samples = len(train_data) // k
        num_epochs = 100
        all_scores = []
        all_mae_histories = []

        model = self.build_model(train_data)
        history = model.fit(train_data, train_targets, epochs=num_epochs, batch_size=1)
        model.save('train_data_model_NN.h5')

        f = open('history.pckl', 'wb')
        pickle.dump(history, f)
        f.close()

        mae_history = history.history['mean_absolute_error']
        val_mse, val_mae = model.evaluate(test_data, test_targets)
        all_scores.append(val_mae)
        all_mae_histories.append(mae_history)

        print('Scores of the k-fold', all_scores)
        print('Saving all scores and mae histories in local files')

        # save
        f = open('all_scores.pckl', 'wb')
        pickle.dump(all_scores, f)
        f.close()
        f = open('all_mae_histories.pckl', 'wb')
        pickle.dump(all_mae_histories, f)
        f.close()

        return model

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
        return (inputs - self.inputs_min)/(self.inputs_max - self.inputs_min)

    def denormalize_output(self, outputs):
        return outputs*(self.output_max - self.output_min)+self.output_min

    def prediction_error(self, x, y, theta, sensors):
        features = self.normalize_inputs(np.array([x, y, theta]))
        pre_sensors = self.denormalize_output(self.model.predict(np.array([features]))[0])

        err = 0
        n_sensors = len(sensors)
        bad_data = True

        # print(true_dist)
        for ix, elem in enumerate(pre_sensors):
            if not math.isnan(sensors[ix]):
                bad_data = False
                # print('err', elem)
                # print('true', true_dist[ix])

                err += (elem - sensors[ix]) ** 2

        return 1/err, bad_data
