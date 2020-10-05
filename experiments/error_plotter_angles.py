import matplotlib.pyplot as plt
import pickle

def getCumulativeError(data):
    cumulativeErrors = []
    cumulativeError = 0
    for x in data:
        cumulativeError += x
        cumulativeErrors.append(cumulativeError)
    
    return cumulativeErrors

directory = "err_angles_sigma_theta"
files = ["err_0005.pkl", "err_05.pkl", "err_1.pkl", "err_20.pkl", "err_odo.pkl"]
labels = ["0.005","0.5","1", "20", "Odometry"]
num_samples = 2000

data = []

for file in files:
   data.append(pickle.load(open("%s/%s" % (directory, file), "rb"))) 

x = range(0, num_samples)

plots = []
for idx, label in enumerate(labels):
    plot, = plt.plot(x, getCumulativeError(data[idx])[:2000], label=label)
    plots.append(plot)


# plt.ylim(0, 2.0)

plt.legend(handles=[plot for plot in plots])
#
plt.xlabel("Time Steps")
plt.ylabel("Cumulative Error")
#
plt.savefig('particles-angles-error.png')
