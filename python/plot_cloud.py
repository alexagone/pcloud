#!/usr/bin/env python3

import argparse
import numpy as np
import matplotlib.pyplot as plt


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('--cloud', default="/tmp/pcloud_cloud.csv")
    parser.add_argument('--hull', default="/tmp/pcloud_hull.csv")

    args = parser.parse_args()

    cloud = np.genfromtxt(args.cloud, delimiter=' ')
    hull= np.genfromtxt(args.hull, delimiter=' ')

    fig, ax = plt.subplots()
    ax.scatter(cloud[:,0], cloud[:,1])
    ax.scatter(hull[:,0], hull[:,1])

    for i, txt in enumerate(range(len(hull))): # .shape[0])):
        ax.annotate(txt, (hull[:,0][i], hull[:,1][i]))

    plt.show()

if __name__ == "__main__":

    try:
        main()
    except Exception as e:
        print(e)
