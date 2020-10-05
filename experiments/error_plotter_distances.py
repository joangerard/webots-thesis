import matplotlib.pyplot as plt
import pickle

def getCumulativeError(data):
    cumulativeErrors = []
    cumulativeError = 0
    for x in data:
        cumulativeError += x
        cumulativeErrors.append(cumulativeError)
    
    return cumulativeErrors

def plot(data, labels, ylabel, name):
    plots = []
    
    for idx, label in enumerate(labels):
        num_samples = len(data[idx])
        x = range(0, num_samples)
        plot, = plt.plot(x, getCumulativeError(data[idx]), label=label)
        plots.append(plot)

    plt.legend(handles=[plot for plot in plots])
    plt.xlabel("Time Steps")
    plt.ylabel(ylabel)
    plt.savefig(name)
    plt.close()

directory = "big-arena-4"
files = ["part.pkl", "odo.pkl"]
labels = ["Particles", "Odometry"]

data_position = []
data_rotation = []

for file in files:
    data = pickle.load(open("%s/%s" % (directory, file), "rb"))
    data_position.append(data[0])
    data_rotation.append(data[1])

plot(data_position, labels, "Cumulative Error (cm)", 'particles-distance-error.png')
plot(data_rotation, labels, "Cumulative Error", 'particles-rotation-error.png')
