#include "bumperbot_localization/plot_trajectory.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "plot_trajectory_node");
    ros::NodeHandle nh;
    PlotTrajectory plot_ground_truth(nh, "base_footprint", "ground_truth_path");
    PlotTrajectory plot_noisy(nh, "base_footprint_noisy", "noisy_path");
    PlotTrajectory plot_ekf(nh, "base_footprint_ekf", "ekf_path");
    ros::spin();

    return 0;
}