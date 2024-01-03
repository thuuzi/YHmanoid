#! /usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt


class PlotResults(object):
    def __init__(self):
        # Setup plot
        self.fig = plt.figure()
        plt.rcParams["font.size"] = 10
     #    axis_list = ["x", "y"]
      #  subplot_args = [1, len(axis_list), 0]
        result_file_path = "../data/joint_data.txt"
        traj_path = "../data/traj_data.txt"
        result_data = np.genfromtxt(result_file_path, dtype=None, delimiter=None, names=True)
        traj_data = np.genfromtxt(traj_path, dtype=None, delimiter=None, names=True)
        ax = self.fig.add_subplot(121)
        ax.plot(result_data["t"], result_data["q0"],color="red", label="q0")
        ax.plot(result_data["t"], result_data["q1"],color="yellow", label="q1")
        ax.plot(result_data["t"], result_data["q2"],color="blue", linestyle="dashed", label="q2")
        ax.plot(traj_data["t"], traj_data["q0"],'o',color="black" )
        ax.plot(traj_data["t"], traj_data["q1"],'o',color="black" )
        ax.plot(traj_data["t"], traj_data["q2"],'o',color="black" )
        # Set labels, etc.
        ax.set_title("results")
        ax.set_xlabel("time [s]", labelpad=-2)
        ax.set_ylabel("angle [deg]")
        ax.grid()
        #ax.legend(loc="lower right")

        plt.show()


if __name__ == "__main__":
  
    plot = PlotResults()
