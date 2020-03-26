import matplotlib.pyplot as plt
import pickle
import numpy as np

data30 = pickle.load(open("data/data_theta_30.pckl", "rb"))
data100 = pickle.load(open("data/data_theta_100.pckl", "rb"))
data500 = pickle.load(open("data/data_theta_500.pckl", "rb"))
data2000 = pickle.load(open("data/data_theta_2000.pckl", "rb"))
dataOdo = pickle.load(open("data/data_theta_odo.pckl", "rb"))

print("Odometry", np.sqrt(np.mean(np.power(dataOdo, 2))))
print("30 particles", np.sqrt(np.mean(np.power(data30, 2))))
print("100 particles", np.sqrt(np.mean(np.power(data100, 2))))
print("500 particles", np.sqrt(np.mean(np.power(data500, 2))))
print("2000 particles", np.sqrt(np.mean(np.power(data2000, 2))))
