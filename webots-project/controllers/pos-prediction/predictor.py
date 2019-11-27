import pandas as pd
from data_collector import DataCollector
from sklearn.ensemble import RandomForestRegressor
import math


class Predictor:
    def __init__(self):
        dc = DataCollector()
        data = dc.get_data_frame()

        # delete NA examples
        data = data.dropna()

        # shuffle data
        data = data.sample(frac=1).reset_index(drop=True)

        inputs = data[['x', 'y', 'theta']]
        output = data[['sensor_1', 'sensor_2', 'sensor_3', 'sensor_4', 'sensor_5', 'sensor_6', 'sensor_7', 'sensor_8']]

        size = len(inputs)

        train_size = int(.8*size)

        train_input = inputs[:train_size]
        train_output = output[:train_size]

        print('train size: ', train_size, 'of', size)

        test_input = inputs[train_size:]
        test_output = output[train_size:]

        reg = RandomForestRegressor(n_estimators=5)
        reg.fit(train_input, train_output)
        print('importance:', reg.feature_importances_)
        print('score', reg.score(test_input, test_output))

        self.model = reg

        # plt.plot(test_input[['x']][:100], test_output[['sensor_1']][:100], 'bo', markersize=0.1)
        # plt.plot(test_input[['x']][:100], [reg.predict([[row['x'],
        #                                                  row['y'],
        #                                                  row['theta']]])[0]
        #                                    for index, row in test_input.iterrows()][:100], 'ro', markersize=0.5)
        # plt.xlabel('x')
        # plt.ylabel('sensor 1 data')
        # plt.savefig('results/x_vs_sensor1.eps', format='eps', dpi=900)

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
