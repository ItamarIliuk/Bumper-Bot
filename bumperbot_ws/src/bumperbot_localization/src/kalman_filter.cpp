#include "bumperbot_localization/kalman_filter.h"
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>


KalmanFilter::KalmanFilter(const ros::NodeHandle &nh) 
                    : nh_(nh),
                      mean_(0.0),
                      variance_(1000.0),
                      motion_variance_(3.0),
                      measurement_variance_(0.1),
                      motion_(0.0),
                      imu_theta_(0.0),
                      is_first_imu_(true),
                      is_first_odom_(true),
                      last_yaw_(0.0),
                      x_(0.0),
                      y_(0.0)
{
  odom_sub_ = nh_.subscribe("bumperbot_controller/odom_noisy", 1000, &KalmanFilter::odomCallback, this);
  imu_sub_ = nh_.subscribe("imu", 1000, &KalmanFilter::imuCallback, this);

  transform_stamped_.header.frame_id = "odom";
  transform_stamped_.child_frame_id = "base_footprint_kalman";
  transform_stamped_.transform.translation.z = 0.0;
}


void KalmanFilter::odomCallback(const nav_msgs::Odometry &odom)
{
    tf2::Quaternion q;
    tf2::fromMsg(odom.pose.pose.orientation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    if(is_first_odom_)
    {
        last_odom_ts_ = odom.header.stamp;
        last_yaw_ = yaw;
        is_first_odom_ = false;
        return;
    }

    motion_ = yaw - last_yaw_;

    statePrediction();
    measurementUpdate();

    // Recalculate x and y using the filtered theta
    double dt = (odom.header.stamp - last_odom_ts_).toSec();
    double ds = odom.twist.twist.linear.x * dt;
    x_ += ds * cos(mean_);
    y_ += ds * sin(mean_);

    // Broadcast the TF
    tf2::Quaternion q_new;
    geometry_msgs::Quaternion q_msg;
    q_new.setRPY(0, 0, mean_);
    tf2::convert(q_new, q_msg);
    static tf2_ros::TransformBroadcaster br;
    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation = q_msg;
    transform_stamped_.header.stamp = odom.header.stamp;
    br.sendTransform(transform_stamped_);

    // Update for the next iteration
    last_odom_ts_ = odom.header.stamp;
    last_yaw_ = yaw;
}


void KalmanFilter::imuCallback(const sensor_msgs::Imu &imu)
{
    if(is_first_imu_)
    {
      // Initialization
      last_imu_ts_ = imu.header.stamp;
      is_first_imu_ = false;
      return;
    }

    double dt = (imu.header.stamp - last_imu_ts_).toSec();
    double ds = imu.angular_velocity.z * dt;
    imu_theta_ += ds;

    // Update for the next iteration
    last_imu_ts_ = imu.header.stamp;
}


void KalmanFilter::measurementUpdate()
{
    mean_ = (measurement_variance_ * mean_ + variance_ * imu_theta_)
          / (variance_ + measurement_variance_);

    variance_ = (variance_ * motion_variance_) 
              / (variance_ + motion_variance_);
}


void KalmanFilter::statePrediction()
{
    mean_ = mean_ + motion_;
    variance_ = variance_ + motion_variance_;
}
