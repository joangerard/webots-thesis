import pandas as pd
from data_collector import DataCollector
from sklearn.ensemble import RandomForestRegressor
import matplotlib.pyplot as plt


if __name__ == '__main__':
    dc = DataCollector()
    data = dc.get_data_frame()

    # delete NA examples
    data = data.dropna()

    # shuffle data
    data = data.sample(frac=1).reset_index(drop=True)

    inputs = data[['estimated_x', 'estimated_y', 'estimated_theta']]
    output = data[['sensor_1', 'sensor_2', 'sensor_3', 'sensor_4', 'sensor_5', 'sensor_6', 'sensor_7', 'sensor_8']]

    size = len(inputs)

    train_size = int(.8*size)

    train_input = inputs[:train_size]
    train_output = output[:train_size]

    print('train size: ', train_size)

    test_input = inputs[train_size:]
    test_output = output[train_size:]

    reg = RandomForestRegressor(n_estimators=10, max_depth=1000, max_features=3)
    reg.fit(train_input, train_output)
    print('importance:', reg.feature_importances_)
    print('score', reg.score(test_input, test_output))

    plt.plot(test_input[['estimated_x']], test_output[['sensor_1']], 'bo', markersize=0.1)
    plt.plot(test_input[['estimated_x']], [reg.predict([[row['estimated_x'],
                                                         row['estimated_y'],
                                                         row['estimated_theta']]])[0]
                                           for index, row in test_input.iterrows()], 'ro', markersize=0.5)
    plt.xlabel('estimated x')
    plt.ylabel('sensor 1 data')
    plt.savefig('results/x_vs_sensor1.eps', format='eps', dpi=900)

