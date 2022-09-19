#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Core>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>


class SimpleController
{
public:
    SimpleController(const ros::NodeHandle &, double radius, double separation);

private:
    void velCallback(const geometry_msgs::Twist &);

    void jointCallback(const sensor_msgs::JointState &);

    ros::NodeHandle nh_;
    ros::Subscriber vel_sub_;
    ros::Publisher right_cmd_pub_;
    ros::Publisher left_cmd_pub_;
    ros::Subscriber joint_sub_;
    ros::Publisher odom_pub_;

    double wheel_radius_;
    double wheel_separation_;
    Eigen::Matrix2d speed_conversion_;
    double right_wheel_prev_pos_;
    double left_wheel_prev_pos_;
    ros::Time prev_time_;
    nav_msgs::Odometry odom_msg_;
};