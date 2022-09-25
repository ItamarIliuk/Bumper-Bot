#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include "bumperbot_examples/GetTransform.h"
#include <tf2/LinearMath/Quaternion.h>


class SimpleTfExamples
{
public:
    SimpleTfExamples(const ros::NodeHandle &);

    bool getTransformCallback(bumperbot_examples::GetTransform::Request &,
                              bumperbot_examples::GetTransform::Response &);

private: 
    void timerCallback(const ros::TimerEvent &);

    ros::NodeHandle nh_;
    geometry_msgs::TransformStamped static_transform_stamped_;
    geometry_msgs::TransformStamped dynamic_transform_stamped_;
    ros::Timer timer_;
    double x_counter_;
    ros::ServiceServer get_transform_srv_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2::Quaternion last_orientation_;
    tf2::Quaternion orientation_increment_;
};