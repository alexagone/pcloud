#!/usr/bin/env python3

import argparse
import numpy as np
import matplotlib.pyplot as plt


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('--cloud', default="/tmp/cloud.csv")

    args = parser.parse_args()

    cloud = np.genfromtxt(args.cloud, delimiter=' ')

    fig, ax = plt.subplots()
    ax.scatter(cloud[:,0], cloud[:,1])

    plt.show()

if __name__ == "__main__":

    try:
        main()
    except Exception as e:
        print(e)
