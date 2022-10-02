#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class KalmanFilter(object):

    def __init__(self):
        self.odom_sub_ = rospy.Subscriber("bumperbot_controller/odom_noisy", Odometry, self.odomCallback)
        self.imu_sub_ = rospy.Subscriber("imu", Imu, self.imuCallback)
        
        # Initially the robot has no idea about how fast is going
        self.mean_ = 0.0
        self.variance_ = 1000.0

        # Modeling the uncertainty of the sensor and the motion
        self.motion_variance_ = 3.0
        self.measurement_variance_ = 0.1

        # Store the messages - only for the orientation
        self.last_imu_ts_ = None
        self.last_odom_ts_ = None
        self.imu_theta_ = 0.0
        self.is_first_imu_ = True
        self.is_first_odom_ = True
        self.last_yaw_ = 0.0
        self.x_ = 0.0
        self.y_ = 0.0
        self.motion_ = 0.0

        # Fill the TF Filtered message
        self.br_ = TransformBroadcaster()
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint_kalman"
        self.transform_stamped_.transform.translation.z = 0.0


    def odomCallback(self, odom):
        q = [odom.pose.pose.orientation.x,
             odom.pose.pose.orientation.y,
             odom.pose.pose.orientation.z,
             odom.pose.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(q)

        if self.is_first_odom_:
            self.last_odom_ts_ = odom.header.stamp
            self.last_yaw_ = yaw
            self.is_first_odom_ = False
            return
        
        self.motion_ = yaw - self.last_yaw_

        self.statePrediction()
        self.measurementUpdate()

        # Recalculate x and y using the filtered theta
        dt = (odom.header.stamp - self.last_odom_ts_).to_sec()
        ds = odom.twist.twist.linear.x * dt
        self.x_ += ds * math.cos(self.mean_)
        self.y_ += ds * math.sin(self.mean_)

        # TF
        q = quaternion_from_euler(0, 0, self.mean_)
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = odom.header.stamp
        self.br_.sendTransform(self.transform_stamped_)

        # Update for the next iteration
        self.last_odom_ts_ = odom.header.stamp
        self.last_yaw_ = yaw


    def imuCallback(self, imu):
        if self.is_first_imu_:
            # Initialization
            self.last_imu_ts_ = imu.header.stamp
            self.is_first_imu_ = False
            return
        
        # Integrate the angular velocity to calculate the orientation
        dt = (imu.header.stamp - self.last_imu_ts_).to_sec()
        ds = imu.angular_velocity.z * dt
        self.imu_theta_ += ds

        # Update for the next iteration
        self.last_imu_ts_ = imu.header.stamp


    def measurementUpdate(self):
        self.mean_ = (self.measurement_variance_ * self.mean_ + self.variance_ * self.imu_theta_) \
                   / (self.variance_ + self.measurement_variance_)
                     
        self.variance_ = (self.variance_ * self.motion_variance_) \
                       / (self.variance_ + self.motion_variance_)


    def statePrediction(self):
        self.mean_ = self.mean_ + self.motion_
        self.variance_ = self.variance_ + self.motion_variance_