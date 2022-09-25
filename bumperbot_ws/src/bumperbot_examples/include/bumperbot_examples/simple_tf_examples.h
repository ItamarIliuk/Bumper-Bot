#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>


class SimpleTfExamples
{
public:
    SimpleTfExamples(const ros::NodeHandle &);

private: 
    void timerCallback(const ros::TimerEvent &);

    ros::NodeHandle nh_;
    geometry_msgs::TransformStamped static_transform_stamped_;
    geometry_msgs::TransformStamped dynamic_transform_stamped_;
    ros::Timer timer_;
    double counter_;
};