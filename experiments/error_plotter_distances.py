import matplotlib.pyplot as plt
import pickle

data30 = pickle.load(open("data30/data_30.pckl", "rb"))
data100 = pickle.load(open("data30/data_100.pckl", "rb"))
data500 = pickle.load(open("data30/data_500.pckl", "rb"))
data30norm = pickle.load(open("data30/data_30_refit.pckl", "rb"))
data2000 = pickle.load(open("data30/data_2000.pckl", "rb"))
dataOdo = pickle.load(open("data30/data_odo.pckl", "rb"))

x = range(0, 1000)

def getCumulativeError(data):
    cumulativeErrors = []
    cumulativeError = 0
    for x in data:
        cumulativeError += x
        cumulativeErrors.append(cumulativeError)
    
    return cumulativeErrors
#
plot30, = plt.plot(x, getCumulativeError(data30), label="30")
plot30_norm, = plt.plot(x, getCumulativeError(data30norm), label="30 refit")
plot100, = plt.plot(x, getCumulativeError(data100), label="100")
plot500, = plt.plot(x, getCumulativeError(data500), label="500")
plot2000, = plt.plot(x, getCumulativeError(data2000), label="2000")
plotOdo, = plt.plot(x, getCumulativeError(dataOdo[:1000]), label="Odometry")
#
# plt.ylim(0, 2.0)

plt.legend(handles=[plot30, plot30_norm, plot100, plot500, plot2000, plotOdo])

plt.xlabel("Time Steps")
plt.ylabel("Cumulative Error (cm)")

plt.savefig('particles-distance-error.png')
