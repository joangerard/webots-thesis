import pandas as pd


class DataCollector:

    def collect(self, estimated_x, estimated_y, estimated_theta, real_x, real_y, real_theta, distances):

        path = 'results/robot_info_dataset.csv'
        data = {
            'x': real_x,
            'y': real_y,
            'theta': real_theta,
            'estimated_x': estimated_x,
            'estimated_y': estimated_y,
            'estimated_theta': estimated_theta,
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
        df.to_csv(path)
        print('data collected in', path)
