#include "bumperbot_examples/simple_tf_examples.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>


SimpleTfExamples::SimpleTfExamples(const ros::NodeHandle &nh)
                                  : nh_(nh),
                                    counter_(0.0)
{
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    static_transform_stamped_.header.stamp = ros::Time::now();
    static_transform_stamped_.header.frame_id = "bumperbot_base";
    static_transform_stamped_.child_frame_id = "bumperbot_top";
    static_transform_stamped_.transform.translation.x = 0;
    static_transform_stamped_.transform.translation.y = 0;
    static_transform_stamped_.transform.translation.z = 0.3;
    static_transform_stamped_.transform.rotation.x = 0;
    static_transform_stamped_.transform.rotation.y = 0;
    static_transform_stamped_.transform.rotation.z = 0;
    static_transform_stamped_.transform.rotation.w = 1;
    static_broadcaster.sendTransform(static_transform_stamped_);
    ROS_INFO("Spinning until killed publishing to world");

    // Timer
    timer_ = nh_.createTimer(ros::Duration(0.1), &SimpleTfExamples::timerCallback, this);
}


void SimpleTfExamples::timerCallback(const ros::TimerEvent &event)
{
  ROS_INFO_STREAM("Timer Triggered, Publishing new message");
  static tf2_ros::TransformBroadcaster dynamic_broadcaster;
  
  dynamic_transform_stamped_.header.stamp = ros::Time::now();
  dynamic_transform_stamped_.header.frame_id = "odom";
  dynamic_transform_stamped_.child_frame_id = "bumperbot_base";
  dynamic_transform_stamped_.transform.translation.x = counter_;
  dynamic_transform_stamped_.transform.translation.y = 0;
  dynamic_transform_stamped_.transform.translation.z = 0;
  dynamic_transform_stamped_.transform.rotation.x = 0;
  dynamic_transform_stamped_.transform.rotation.y = 0;
  dynamic_transform_stamped_.transform.rotation.z = 0;
  dynamic_transform_stamped_.transform.rotation.w = 1;

  dynamic_broadcaster.sendTransform(dynamic_transform_stamped_);
  counter_ += 0.05;
}