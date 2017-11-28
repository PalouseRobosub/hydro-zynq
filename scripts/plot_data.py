#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
import argparse


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

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', type=str, help='Input CSV file')
    args = parser.parse_args()


    f = open(args.filename, "r")
    label_line = f.readline()
    f.close()

    labels = [x.lstrip() for x in label_line.rstrip().split(",")]

    data = np.loadtxt(args.filename, delimiter=",", skiprows=1)

    plot(data, channels=[1,3])
