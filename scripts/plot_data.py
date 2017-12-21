#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt


# Displays a plot of the channel data
#
# data should be a numpy 2-D array in the format
# [ [0, Ch0 sample 0, Ch1 sample 0, Ch2 sample 0, Ch3 sample 0],
#   [1, Ch0 sample 1, Ch1 sample 1, Ch2 sample 1, Ch3 sample 1],
#   [2, Ch0 sample 3, Ch1 sample 3, Ch2 sample 3, Ch3 sample 3],
#      ...
#   [n, Ch0 sample n, Ch1 sample n, Ch2 sample n, Ch3 sample n],
def plot_samples(data, channels=[0,1,2,3], labels={}, split=False):

    label_list = []
    for channel in channels:
        plt.plot(data[:,0], data[:,channel+1])
        label_list.append(labels.get(channel, "unlabeled"))

    plt.legend(label_list)
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.show()

def plot_correlations(data, channels=[0,1,2,3], labels={}, split=False):

    label_list = []
    for channel in channels:
        #find the max location
        max_index = np.argmax(data[:,channel+1])
        max_val = data[max_index,channel+1]
        max_index = data[max_index,0]

        plt.plot(data[:,0], data[:,channel+1])
        plt.scatter([max_index], [max_val], s=100, marker='x', color='red')
        label_list.append(labels.get(channel, "unlabeled")+ ": {:.2f} us".format(max_index*(10**6)))

    plt.legend(label_list)
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    import time
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', type=str, help='Input CSV file')
    args = parser.parse_args()


    f = open(args.filename, "r")
    label_line = f.readline()
    f.close()

    labels = [x.lstrip() for x in label_line.rstrip().split(",")]

    data = np.loadtxt(args.filename, delimiter=",", skiprows=1)

    channels = [0,1]
    labels = {0:"Ch0", 1:"Ch1"}


    data = np.zeros((20,3))
    for x in xrange(-10,10):
        data[10+x,0] = x
        data[10+x,1] = -(x+3)**2
        data[10+x,2] = -(x+5)**2 + 5
    print data

    plot_correlations(data, [0,1], {0:"ch0-ch1", 1:"ch0-ch2"})


