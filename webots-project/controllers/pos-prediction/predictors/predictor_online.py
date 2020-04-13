import math
from creme import linear_model
from creme.preprocessing import StandardScaler
from creme.compose import Pipeline
from creme import ensemble
from creme import optim
from creme import metrics


class PredictorOnline:
    def __init__(self, data_collector):
        dc = data_collector
        data = dc.get_data_frame()
        metric = metrics.MAE()

        # delete NA examples
        data = data.dropna()

        # shuffle data
        X_y = data.sample(frac=1).reset_index(drop=True)

        data = X_y[['x', 'y', 'theta']].to_dict('records')
        target_1 = X_y[['sensor_1']]
        target_2 = X_y[['sensor_3']]
        target_3 = X_y[['sensor_5']]
        target_4 = X_y[['sensor_7']]

        print('constructing models')

        # construct our pipeline
        model_1 = Pipeline([
            ("scale", StandardScaler()),
            ("learn", ensemble.HedgeRegressor([
                linear_model.LinearRegression(optim.SGD()),
                linear_model.LinearRegression(optim.RMSProp()),
                linear_model.LinearRegression(optim.Adam())
            ]))
        ])

        # construct our pipeline
        model_2 = Pipeline([
            ("scale", StandardScaler()),
            ("learn", ensemble.HedgeRegressor([
                linear_model.LinearRegression(optim.SGD()),
                linear_model.LinearRegression(optim.RMSProp()),
                linear_model.LinearRegression(optim.Adam())
            ]))
        ])

        # construct our pipeline
        model_3 = Pipeline([
            ("scale", StandardScaler()),
            ("learn", ensemble.HedgeRegressor([
                linear_model.LinearRegression(optim.SGD()),
                linear_model.LinearRegression(optim.RMSProp()),
                linear_model.LinearRegression(optim.Adam())
            ]))
        ])

        # construct our pipeline
        model_4 = Pipeline([
            ("scale", StandardScaler()),
            ("learn", ensemble.HedgeRegressor([
                linear_model.LinearRegression(optim.SGD()),
                linear_model.LinearRegression(optim.RMSProp()),
                linear_model.LinearRegression(optim.Adam())
            ]))
        ])

        print('start training')

        for x, y_1, y_2, y_3, y_4 in zip(data, target_1.values, target_2.values, target_3.values, target_4.values,):
            model_1, y_pred_1 = self._update_model(model_1, x, y_1)
            model_2, y_pred_2 = self._update_model(model_2, x, y_2)
            model_3, y_pred_3 = self._update_model(model_3, x, y_3)
            model_4, y_pred_4 = self._update_model(model_4, x, y_4)

        self.models = [model_1, model_2, model_3, model_4]

        print('done...')

    def update_models(self, x, y):
        models = []
        for i, model in enumerate(self.models):
            models.append(model.fit_one(x, y[i]))

        self.models = models

    def _update_model(self, model, x, y):
        # make predictions on the current set of features, train the
        # model on the features, and then update our metric
        y_pred = model.predict_one(x)
        model = model.fit_one(x, y)

        return model, y_pred

    def _predict(self, x, y, theta):
        xi = {
            'x': x,
            'y': y,
            'theta': theta
        }

        y_pred = []

        for model in self.models:
            y_pred.append(model.predict_one(xi))

        return y_pred

    def prediction_error(self, x, y, theta, sensors):
        pre_sensors = self._predict(x, y, theta)

        err = 0
        bad_data = True

        true_dist = [sensors[0].getValue(), sensors[2].getValue(), sensors[4].getValue(), sensors[6].getValue()]
        # print(true_dist)
        for ix, elem in enumerate(pre_sensors):
            if not math.isnan(true_dist[ix]):
                bad_data = False
                # print('err', elem)
                # print('true', true_dist[ix])

                err += (elem - true_dist[ix]) ** 2

        return err, bad_data

