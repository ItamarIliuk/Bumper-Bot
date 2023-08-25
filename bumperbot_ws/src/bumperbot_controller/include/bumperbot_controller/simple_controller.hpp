#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <Eigen/Core>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>


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

    // Odometry
    double wheel_radius_;
    double wheel_separation_;
    Eigen::Matrix2d speed_conversion_;
    double right_wheel_prev_pos_;
    double left_wheel_prev_pos_;
    ros::Time prev_time_;
    nav_msgs::Odometry odom_msg_;
    double x_;
    double y_;
    double theta_;

    // TF
    geometry_msgs::TransformStamped transform_stamped_;
};

#endif // SIMPLE_CONTROLLER_HPP