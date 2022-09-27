#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import numpy as np


class MotionNoise(object):

    def __init__(self):
        self.linear_velocity_sd_ = 0.1
        self.angular_velocity_sd_ = 0.1

        self.noisy_odom_ = Odometry()
        self.noisy_odom_.header.frame_id = "odom"
        self.noisy_odom_.child_frame_id = "base_footprint"

        self.pub_ = rospy.Publisher("odom_noisy", Odometry, queue_size=10)
        self.sub_ = rospy.Subscriber("bumperbot_controller/odom", Odometry, self.odomCallback)


    def odomCallback(self, msg):

        # Add noise and publish message
        self.noisy_odom_.pose = msg.pose
        self.noisy_odom_.twist.twist.linear.x = msg.twist.twist.linear.x + np.random.normal(0, self.linear_velocity_sd_) 
        self.noisy_odom_.twist.twist.angular.z = msg.twist.twist.angular.z + np.random.normal(0, self.linear_velocity_sd_)
        self.noisy_odom_.header.stamp = rospy.Time.now()

        self.pub_.publish(self.noisy_odom_)
