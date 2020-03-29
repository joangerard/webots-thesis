import matplotlib.pyplot as plt
import pickle
import numpy as np

data30 = pickle.load(open("data30/data_30.pckl", "rb"))
data100 = pickle.load(open("data30/data_100.pckl", "rb"))
data500 = pickle.load(open("data30/data_500.pckl", "rb"))
data2000 = pickle.load(open("data30/data_2000.pckl", "rb"))
dataOdo = pickle.load(open("data30/data_odo.pckl", "rb"))

# Reporting RMSD for the data captured using
# self.sigma = 0.0055
# self.sigma_theta = 10

print("Odometry", np.sqrt(np.mean(np.power(dataOdo, 2))) * 100)
print("30 particles", np.sqrt(np.mean(np.power(data30, 2))) * 100)
print("100 particles", np.sqrt(np.mean(np.power(data100, 2))) * 100)
print("500 particles", np.sqrt(np.mean(np.power(data500, 2))) * 100)
print("2000 particles", np.sqrt(np.mean(np.power(data2000, 2))) * 100)
