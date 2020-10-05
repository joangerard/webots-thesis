import pandas as pd
import numpy as np


class DataCollector:

    def __init__(self):
        self.path = 'results/robot_info_dataset.csv'
        self.read_path = 'results/robot_info_dataset-jumped.csv'

    def collect(self, real_x, real_y, real_theta, distances):
        # calculate DELTA_X DELTA_S: take real_x and displace to one position ->
        # then eliminate last data from real_x to match real_x_time1 size

        real_x_time1 = real_x[:len(real_x) - 1]
        real_y_time1 = real_y[:len(real_y) - 1]
        real_theta_time1 = real_theta[:len(real_theta) - 1]

        real_x = real_x[1:]
        real_y = real_y[1:]
        real_theta = real_theta[1:]

        distances_time1 = distances[:len(distances) - 1]

        distances = distances[1:, :]

        data = {
            'x': real_x,
            'y': real_y,
            'theta': real_theta,
            'dx': np.subtract(real_x, real_x_time1),
            'dy': np.subtract(real_y, real_y_time1),
            'dtheta': np.subtract(real_theta, real_theta_time1),
            'sensor_1': distances[:, 0],
            'sensor_2': distances[:, 1],
            'sensor_3': distances[:, 2],
            'sensor_4': distances[:, 3],
            'sensor_5': distances[:, 4],
            'sensor_6': distances[:, 5],
            'sensor_7': distances[:, 6],
            'sensor_8': distances[:, 7]
            # 'dsensor_1': np.subtract(distances[:, 0], distances_time1[:, 0]),
            # 'dsensor_2': np.subtract(distances[:, 1], distances_time1[:, 1]),
            # 'dsensor_3': np.subtract(distances[:, 2], distances_time1[:, 2]),
            # 'dsensor_4': np.subtract(distances[:, 3], distances_time1[:, 3]),
            # 'dsensor_5': np.subtract(distances[:, 4], distances_time1[:, 4]),
            # 'dsensor_6': np.subtract(distances[:, 5], distances_time1[:, 5]),
            # 'dsensor_7': np.subtract(distances[:, 6], distances_time1[:, 6]),
            # 'dsensor_8': np.subtract(distances[:, 7], distances_time1[:, 7])
        }

        df = pd.DataFrame(data)

        df.to_csv(self.path)
        print('data collected in', self.path)

    def get_data_frame(self):
        return pd.read_csv(self.read_path)
