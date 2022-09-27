#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>


class MotionNoise
{
public:
    MotionNoise(const ros::NodeHandle &);

private:

    void odomCallback(const nav_msgs::Odometry &);

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    double linear_velocity_sd_;
    double angular_velocity_sd_;
    nav_msgs::Odometry noisy_odom_;
};

