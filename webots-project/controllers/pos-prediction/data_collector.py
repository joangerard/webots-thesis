import pandas as pd


class DataCollector:

    def __init__(self):
        self.path = 'results/robot_info_dataset.csv'
        self.read_path = 'results/robot_info_dataset.csv'

    def collect(self, real_x, real_y, real_theta, distances):
        data = {
            'x': real_x,
            'y': real_y,
            'theta': real_theta,
            'sensor_1': distances[:, 0],
            'sensor_2': distances[:, 1],
            'sensor_3': distances[:, 2],
            'sensor_4': distances[:, 3],
            'sensor_5': distances[:, 4],
            'sensor_6': distances[:, 5],
            'sensor_7': distances[:, 6],
            'sensor_8': distances[:, 7]
        }

        df = pd.DataFrame(data)

        df.to_csv(self.path)
        print('data collected in', self.path)

    def get_data_frame(self):
        return pd.read_csv(self.read_path)
