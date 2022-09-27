#include "bumperbot_localization/simulate_motion_noise.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulate_motion_noise_node");
    ros::NodeHandle nh;
    MotionNoise noise(nh);
    ros::spin();

    return 0;
}