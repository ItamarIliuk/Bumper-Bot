#include <ros/ros.h>
#include "bumperbot_controller/simple_controller.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_controller");
    ros::NodeHandle nh;
    // TODO: take this from the partam server and the launch file
    SimpleController controller(nh, 0.033, 0.14);
    ros::spin();

    return 0;
}