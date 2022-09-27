#include "bumperbot_localization/simulate_motion_noise.h"
#include <random>


MotionNoise::MotionNoise(const ros::NodeHandle &nh)
                        : nh_(nh),
                          linear_velocity_sd_(0.1),
                          angular_velocity_sd_(0.1)
{
    noisy_odom_.header.frame_id = "odom";
    noisy_odom_.child_frame_id = "base_footprint";

    pub_ = nh_.advertise<nav_msgs::Odometry>("odom_noisy", 10);
    sub_ = nh_.subscribe("bumperbot_controller/odom", 1000, &MotionNoise::odomCallback, this);
}


void MotionNoise::odomCallback(const nav_msgs::Odometry &msg)
{
    // Add noise and publish message
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine noise_generator(seed);
    std::normal_distribution<double> linear_noise(0.0, linear_velocity_sd_);
    std::normal_distribution<double> angular_noise(0.0, angular_velocity_sd_);
    noisy_odom_.pose = msg.pose;
    noisy_odom_.twist.twist.linear.x = msg.twist.twist.linear.x + linear_noise(noise_generator);
    noisy_odom_.twist.twist.angular.z = msg.twist.twist.angular.z + angular_noise(noise_generator);
    noisy_odom_.header.stamp = msg.header.stamp;

    pub_.publish(noisy_odom_);
}