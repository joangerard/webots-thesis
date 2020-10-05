from data_collector.data_collector import DataCollector
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

        filename = 'models/train_data_model_rf_5.pckl'
        # if model exists load otherwise create it
        # if os.path.isfile(filename):
        #     self.model = pickle.load(open(filename, 'rb'))
        # else:
        reg = RandomForestRegressor(n_estimators=5, criterion='mse', verbose=False, n_jobs=5)
        reg.fit(inputs, output)
        # pickle.dump(reg, open(filename, 'wb'))
        self.model = reg

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

                err += abs((elem - true_dist[ix])) ** 2

        return err, bad_data

    def prediction_error(self, x, y, theta, sensors):
        pre_sensors = self.model.predict([[x, y, theta]])

        err = 0
        bad_data = True

        for ix, elem in enumerate(pre_sensors[0]):
            if not math.isnan(sensors[ix]):
                bad_data = False
                # print('err', elem)
                # print('true', true_dist[ix])

                err += (elem - sensors[ix]) ** 2

        return 1/err, bad_data
