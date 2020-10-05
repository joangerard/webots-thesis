import matplotlib.pyplot as plt
import pickle
import numpy as np


directory = "big-arena-4"
files = ["part.pkl", "odo.pkl"]
labels = ["Particles", "Odometry"]

data_position = []

for idx, file in enumerate(files):
    value_to_show = labels[idx]+ ": "
    data = pickle.load(open("%s/%s" % (directory, file), "rb"))
    value_to_show += "%.2f" % (np.sqrt(np.mean(np.power(data[1], 2))))
    print(value_to_show)

