from keras import models
from keras import layers
import numpy as np
import math
import pickle


class PredictorNN:
    def __init__(self, data_collector):
        self.percentageTraining = .8

        self.dc = data_collector
        data = self.dc.get_data_frame()

        # delete NA examples
        data = data.dropna()

        # shuffle data
        data = data.sample(frac=1).reset_index(drop=True)

        inputs = data[['x', 'y', 'theta']]
        output = data[['sensor_1', 'sensor_2', 'sensor_3', 'sensor_4', 'sensor_5', 'sensor_6', 'sensor_7', 'sensor_8']]

        size = len(inputs)

        train_size = int(self.percentageTraining * size)

        train_data = inputs[:train_size]
        train_targets = output[:train_size]

        test_data = inputs[train_size:]
        test_targets = output[train_size:]

        print('train size: ', train_size, 'of', size)

        # normalize data
        self.normalize_data(train_data, test_data)

        # kfold
        self.create_model(train_data, train_targets, test_data, test_targets)

    def create_model(self, train_data, train_targets, test_data, test_targets):
        k = 4
        num_val_samples = len(train_data) // k
        num_epochs = 100
        all_scores = []
        all_mae_histories = []

        model = self.build_model(train_data)
        history = model.fit(train_data, train_targets, epochs=num_epochs, batch_size=1)
        model.save('train_data_model_NN.h5')
        mae_history = history.history['val_mean_absolute_error']
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

    def predict(self, x, y, theta, sensors):
        pre_sensors = self.model.predict([[x, y, theta]])

        err = 0
        n_sensors = len(sensors)
        bad_data = True

        true_dist = [sensor.getValue() for sensor in sensors]
        # print(true_dist)
        for ix, elem in enumerate(pre_sensors[0]):
            if not math.isnan(true_dist[ix]):
                bad_data = False
                # print('err', elem)
                # print('true', true_dist[ix])

                err += (elem - true_dist[ix]) ** 2

        return err, bad_data
