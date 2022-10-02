#!/usr/bin/env python3
import rospy
from bumperbot_localization.plot_trajectory import PlotTrajectory


if __name__== '__main__':
    rospy.init_node('plot_trajectories_node')
    plot_ground_truth = PlotTrajectory("base_footprint", "ground_truth_path")
    plot_noisy = PlotTrajectory("base_footprint_noisy", "noisy_path")
    plot_ekf = PlotTrajectory("base_footprint_ekf", "ekf_path")
    
    rospy.spin()