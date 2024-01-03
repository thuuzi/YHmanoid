#! /usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt


class PlotResults(object):
    def __init__(self):
        # Setup plot
        self.fig = plt.figure()
        plt.rcParams["font.size"] = 10
        swingfoot_file_path = "../data/swingfoot_data.txt"
        result_data = np.genfromtxt(swingfoot_file_path, dtype=None, delimiter=None, names=True)
        ax = self.fig.add_subplot(141)
        ax.plot(result_data["t"], result_data["z"],color="blue",  label="z")
        # Set labels, etc.
        ax.set_title("results")
        ax.set_xlabel("time [s]", labelpad=-2)
        ax.set_ylabel("position [m]")
        ax.grid()
        ax.legend(loc="best")
        #traj
        ax2 = self.fig.add_subplot(144)
        ax2.plot(result_data["x"], result_data["z"],color="red", label="xz")
        # Set labels, etc.
        ax2.set_title("results")
        ax2.set_xlabel("z [m]", labelpad=-2)
        ax2.set_ylabel("x [m]")
        ax2.grid()
        ax2.legend(loc="best")
        #v
        ax3 = self.fig.add_subplot(142)
        ax3.plot(result_data["t"], result_data["vz"],color="blue",  label="vz")
        # Set labels, etc.
        ax3.set_title("results")
        ax3.set_xlabel("time [s]", labelpad=-2)
        ax3.set_ylabel("velocity [m/s]")
        ax3.grid()
        ax3.legend(loc="best")
        #a
        ax3 = self.fig.add_subplot(143)
        ax3.plot(result_data["t"], result_data["az"],color="blue",  label="az")
        # Set labels, etc.
        ax3.set_title("results")
        ax3.set_xlabel("time [s]", labelpad=-2)
        ax3.set_ylabel("acceration [m/s^2]")
        ax3.grid()
        ax3.legend(loc="best")
        plt.show()


if __name__ == "__main__":
  
    plot = PlotResults()
