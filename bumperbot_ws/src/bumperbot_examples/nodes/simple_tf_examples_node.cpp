#include "bumperbot_examples/simple_tf_examples.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_tf_examples");
    ros::NodeHandle nh;
    SimpleTfExamples examples(nh);
    ros::spin();

    return 0;
}