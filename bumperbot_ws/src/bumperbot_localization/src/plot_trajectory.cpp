#include "bumperbot_localization/plot_trajectory.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>


PlotTrajectory::PlotTrajectory(const ros::NodeHandle &nh, std::string frame_id, std::string topic_name)
                    : nh_(nh),
                      frame_id_(frame_id),
                      topic_name_(topic_name),
                      tf_listener_(tf_buffer_)
{
    path_pub_ = nh_.advertise<nav_msgs::Path>(topic_name_, 10);
    // Wait for the other nodes to begin publish
    ros::Duration(1.0).sleep();

    timer_ = nh_.createTimer(ros::Duration(0.1), &PlotTrajectory::timerCallback, this);

    // Initialize the Path messages
    // Path - odom --> frame_id
    path_.header.frame_id = "odom";
}


void PlotTrajectory::timerCallback(const ros::TimerEvent &event)
{
    geometry_msgs::TransformStamped transform;
    try
    {
        transform = tf_buffer_.lookupTransform("odom", frame_id_, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_ERROR_STREAM("Something went wrong " << ex.what());
      return;
    }
    
    // Publish Path
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    path_.poses.push_back(pose);
    path_pub_.publish(path_);
}