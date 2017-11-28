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
def plot(data, channels=[0,1,2,3]):

    labels = []
    for channel in channels:
        plt.plot(data[:,0], data[:,channel+1])
        labels.append("Ch{}".format(channel))

    plt.legend(labels)
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.show()


class live_plot:
    def __init__(self, channels=[0,1,2,3], labels={}):
        self.channels = channels
        self.labels = labels
        for channel in self.channels:
            if channel not in self.labels:
                print "warning: no label specified for channel {}".format(channel)

        plt.ion()
        plt.show()

        fig = plt.figure()

    def plot(self, data):
        plt.clf()

        labels = []
        for channel in self.channels:
            plt.plot(data[:,0], data[:,channel+1])
            labels.append(self.labels.get(channel, "unknown"))

        plt.legend(labels)
        plt.xlabel('Time (s)')
        plt.grid(True)

        plt.draw()
        # need to call pause for internal matplotlib event loop to process
        # events
        plt.pause(0.1)


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

    plotter = live_plot([0,1], {0:"Ch0", 1:"Ch1"})
    while 1:
        plotter.plot(data)
        data[1,1] += 1
        time.sleep(1)


