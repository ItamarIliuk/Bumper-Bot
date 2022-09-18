#include "bumperbot_template/bumperbot.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "bumperbot_node");
    ros::NodeHandle nh;
    Bumperbot bumperbot(nh);
    ros::spin();

    return 0;
}