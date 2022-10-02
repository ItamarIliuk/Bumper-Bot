#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Path.h>


class PlotTrajectory
{
public:
    PlotTrajectory(const ros::NodeHandle &, std::string frame_id, std::string topic_name);


private:
    void timerCallback(const ros::TimerEvent &);

    ros::NodeHandle nh_;
    std::string frame_id_;
    std::string topic_name_;
    ros::Timer timer_;
    ros::Publisher path_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    nav_msgs::Path path_;
};

