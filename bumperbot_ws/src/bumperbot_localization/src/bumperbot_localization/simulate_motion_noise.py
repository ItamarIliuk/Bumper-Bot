#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MotionNoise(object):

    def __init__(self):
        self.pub_ = rospy.Publisher("bumperbot_controller/odom_noisy", Odometry, queue_size=10)
        self.pub_w_ = rospy.Publisher("noisy_heading", Float64, queue_size=10)
        self.sub_ = rospy.Subscriber("bumperbot_controller/odom", Odometry, self.odomCallback)
        self.br_ = tf2_ros.TransformBroadcaster()

        # Motion noise parameters
        self.a1_ = 0.05
        self.a2_ = 0.05
        self.a3_ = 0.05
        self.a4_ = 0.05

        # Motion Parameters
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        self.last_x_ = 0.0
        self.last_y_ = 0.0
        self.last_theta_ = 0.0

        # Fill the Noisy Odometry message with invariant parameters
        self.noisy_odom_msg_ = Odometry()
        self.noisy_odom_msg_.header.frame_id = "odom"
        self.noisy_odom_msg_.child_frame_id = "base_footprint_noisy"
        self.noisy_odom_msg_.twist.twist.linear.y = 0.0
        self.noisy_odom_msg_.twist.twist.linear.z = 0.0
        self.noisy_odom_msg_.twist.twist.angular.x = 0.0
        self.noisy_odom_msg_.twist.twist.angular.y = 0.0
        self.noisy_odom_msg_.pose.pose.orientation.x = 0.0
        self.noisy_odom_msg_.pose.pose.orientation.y = 0.0
        self.noisy_odom_msg_.pose.pose.orientation.z = 0.0
        self.noisy_odom_msg_.pose.pose.orientation.w = 1.0

        # Fill the TF noisy message
        self.noisy_transform_stamped_ = TransformStamped()
        self.noisy_transform_stamped_.header.frame_id = "odom"
        self.noisy_transform_stamped_.child_frame_id = "base_footprint_noisy"
        self.noisy_transform_stamped_.transform.translation.z = 0.0


    def odomCallback(self, msg): 
        # Motion Model
        q = [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w]   
        (r, p, y) = euler_from_quaternion(q)
        d_translation = math.sqrt(math.pow(msg.pose.pose.position.x - self.last_x_, 2) + \
                                  math.pow(msg.pose.pose.position.y - self.last_y_, 2))
        d_rot1 = math.atan2((msg.pose.pose.position.y - self.last_y_), (msg.pose.pose.position.x - self.last_x_)) - self.last_theta_
        d_rot2 = y - self.last_theta_ - d_rot1

        # Add Noise
        sd_rot1 = self.a1_ * abs(d_rot1) + self.a2_ * d_translation
        sd_rot2 = self.a1_ * abs(d_rot2) + self.a2_ * d_translation
        sd_translation = self.a3_ * d_translation + self.a4_ * (abs(d_rot1) + abs(d_rot2))
        d_translation += np.random.normal(0, sd_translation*sd_translation) 
        d_rot1 += np.random.normal(0, sd_rot1*sd_rot1)
        d_rot2 += np.random.normal(0, sd_rot2*sd_rot2)

        # Return to the Cartesian Coordinates
        self.theta_ += d_rot1 + d_rot2 - y
        self.x_ += d_translation * math.cos(self.last_theta_ + d_rot1)
        self.y_ += d_translation * math.sin(self.last_theta_ + d_rot1)

        # Update Variables for the next itheration
        self.last_theta_ = y
        self.last_x_ = msg.pose.pose.position.x
        self.last_y_ = msg.pose.pose.position.y

        # Compose and publish the odom message
        q_new = quaternion_from_euler(0, 0, self.theta_)
        self.noisy_odom_msg_.header.stamp = msg.header.stamp
        self.noisy_odom_msg_.pose.pose.position.x = self.x_
        self.noisy_odom_msg_.pose.pose.position.y = self.y_
        self.noisy_odom_msg_.pose.pose.orientation.x = q_new[0]
        self.noisy_odom_msg_.pose.pose.orientation.y = q_new[1]
        self.noisy_odom_msg_.pose.pose.orientation.z = q_new[2]
        self.noisy_odom_msg_.pose.pose.orientation.w = q_new[3]
        self.noisy_odom_msg_.twist.twist.linear.x = msg.twist.twist.linear.x
        self.noisy_odom_msg_.twist.twist.angular.z = msg.twist.twist.angular.z
        self.pub_.publish(self.noisy_odom_msg_)
        self.pub_w_.publish(self.theta_)
    
        # publish tf
        self.noisy_transform_stamped_.transform.translation.x = self.x_
        self.noisy_transform_stamped_.transform.translation.y = self.y_
        self.noisy_transform_stamped_.transform.rotation.x = q_new[0]
        self.noisy_transform_stamped_.transform.rotation.y = q_new[1]
        self.noisy_transform_stamped_.transform.rotation.z = q_new[2]
        self.noisy_transform_stamped_.transform.rotation.w = q_new[3]
        self.noisy_transform_stamped_.header.stamp = msg.header.stamp
        self.br_.sendTransform(self.noisy_transform_stamped_)
