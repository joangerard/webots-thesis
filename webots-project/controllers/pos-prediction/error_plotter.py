import matplotlib.pyplot as plt
import pickle

data30 = pickle.load(open("data_rf_30.pckl", "rb"))
data100 = pickle.load(open("data_theta_100.pckl", "rb"))
data500 = pickle.load(open("data_theta_500.pckl", "rb"))
data1000 = pickle.load(open("data_1000.pckl", "rb"))
data2000 = pickle.load(open("data_theta_2000.pckl", "rb"))
dataOdo = pickle.load(open("data_odo.pckl", "rb"))

x = range(3, 2001)
#
plot30, = plt.plot(x, data30[3:], label="30")
# plot100, = plt.plot(x, data100[1500:], label="100")
# plot500, = plt.plot(x, data500[1500:], label="500")
# plot1000, = plt.plot(x, data1000, label="1000")
# plot2000, = plt.plot(x, data2000[1500:], label="2000")
plotOdo, = plt.plot(x, dataOdo[3:2001], label="Odometry")
#
# plt.ylim(0, 2.0)

# plt.legend(handles=[plot30, plot100, plot500, plot2000, plotOdo])

plt.xlabel("Time Steps")
plt.ylabel("Error")

plt.show()
# plt.savefig('particles-angles2.png')
