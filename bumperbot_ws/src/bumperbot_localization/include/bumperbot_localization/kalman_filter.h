#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>


class KalmanFilter
{
public:
    KalmanFilter(const ros::NodeHandle &);

    void statePrediction();

    void measurementUpdate();


private:
    void odomCallback(const nav_msgs::Odometry &);

    void imuCallback(const sensor_msgs::Imu &);


    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;
    double mean_;
    double variance_;
    double motion_variance_;
    double measurement_variance_;
    double motion_;
    ros::Time last_imu_ts_;
    ros::Time last_odom_ts_;
    double imu_theta_;
    bool is_first_imu_;
    bool is_first_odom_;
    double last_yaw_;
    double x_;
    double y_;
    geometry_msgs::TransformStamped transform_stamped_;
};

