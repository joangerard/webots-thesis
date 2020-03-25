import matplotlib.pyplot as plt
import pickle

data30 = pickle.load(open("data/data_30.pckl", "rb"))
data100 = pickle.load(open("data/data_100.pckl", "rb"))
data500 = pickle.load(open("data/data_500.pckl", "rb"))
data1000 = pickle.load(open("data/data_1000.pckl", "rb"))
data2000 = pickle.load(open("data/data_2000.pckl", "rb"))
dataOdo = pickle.load(open("data/data_odo.pckl", "rb"))

x = range(0, 2201)
#
plot30, = plt.plot(x, data30, label="30")
plot100, = plt.plot(x, data100, label="100")
plot500, = plt.plot(x, data500, label="500")
# plot1000, = plt.plot(x, data1000, label="1000")
plot2000, = plt.plot(x, data2000, label="2000")
plotOdo, = plt.plot(x, dataOdo, label="Odometry")
#
# plt.ylim(0, 2.0)

plt.legend(handles=[plot30, plot100, plot500, plot2000, plotOdo])

plt.xlabel("Time Steps")
plt.ylabel("Error (cm)")

plt.savefig('particles-distance-error.png')
