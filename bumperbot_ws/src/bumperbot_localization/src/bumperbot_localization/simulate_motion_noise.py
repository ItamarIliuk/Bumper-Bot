#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MotionNoise(object):

    def __init__(self):
        # Noise to the Rotational component of Rotation
        self.alpha1_ = 0.05
        # Noise to the Translational component of Rotation
        self.alpha2_ = 0.05
        # Noise to the Translational component of Translation
        self.alpha3_ = 0.05 
        # Noise to the Rotational component of Translation
        self.alpha4_ = 0.05

        self.previous_odom_ = None
        self.noisy_odom_ = Odometry()
        self.pose_ = [0.0, 0.0, 0.0]

        self.pub_ = rospy.Publisher("odom_noisy", Odometry, queue_size=10)
        self.sub_ = rospy.Subscriber("bumperbot_controller/odom", Odometry, self.odomCallback)


    def odomCallback(self, msg):
        q_new = [msg.pose.pose.orientation.x,
                 msg.pose.pose.orientation.y,
                 msg.pose.pose.orientation.z,
                 msg.pose.pose.orientation.w]
        (r,p, theta2) = euler_from_quaternion(q_new)

        # Initialization
        if self.previous_odom_ == None:
            self.pose_[0] = msg.pose.pose.position.x
            self.pose_[1] = msg.pose.pose.position.y
            self.pose_[2] = theta2
        else:
            # Calculate the Noisy Pose
            dx = msg.pose.pose.position.x - self.previous_odom_.pose.pose.position.x
            dy = msg.pose.pose.position.y - self.previous_odom_.pose.pose.position.y
            trans = math.sqrt(dx * dx + dy * dy)
            q_prev = [self.previous_odom_.pose.pose.orientation.x,
                      self.previous_odom_.pose.pose.orientation.y,
                      self.previous_odom_.pose.pose.orientation.z,
                      self.previous_odom_.pose.pose.orientation.w]
            (r,p, theta1) = euler_from_quaternion(q_prev)
            
            rot1 = math.atan2(dy, dx) - theta1
            rot2 = theta2 - theta1 - rot1
    
            sd_rot1 = self.alpha1_ * abs(rot1) + self.alpha2_ * trans
            sd_rot2 = self.alpha1_ * abs(rot2) + self.alpha2_ * trans
            sd_trans = self.alpha3_ * trans + self.alpha4_ * (abs(rot1) + abs(rot2))
    
            trans += np.random.normal(0,sd_trans * sd_trans)
            rot1 += np.random.normal(0, sd_rot1 * sd_rot1)
            rot2 += np.random.normal(0, sd_rot2 * sd_rot2)
    
            self.pose_[0] += trans * math.cos(theta1 + rot1)
            self.pose_[1] += trans * math.sin(theta1 + rot1)
            self.pose_[2] += rot1 + rot2

        self.previous_odom_ = msg

        # Add noise and publish message
        self.noisy_odom_.pose.pose.position.x = msg.pose.pose.position.x + np.random.normal(0, 0.05)
        self.noisy_odom_.pose.pose.position.y = msg.pose.pose.position.y + np.random.normal(0, 0.05)

        (r, p, y) = euler_from_quaternion(q_new)
        y_noisy = y + np.random.normal(0, 0.5)
        q_noisy = quaternion_from_euler(0, 0, y_noisy)
        self.noisy_odom_.pose.pose.orientation.x = q_noisy[0]
        self.noisy_odom_.pose.pose.orientation.y = q_noisy[1]
        self.noisy_odom_.pose.pose.orientation.z = q_noisy[2]
        self.noisy_odom_.pose.pose.orientation.w = q_noisy[3]
        self.noisy_odom_.twist.twist.linear.x = msg.twist.twist.linear.x + np.random.normal(0, 0.05) 
        self.noisy_odom_.twist.twist.angular.z = msg.twist.twist.angular.z + np.random.normal(0, 0.05)
        self.noisy_odom_.header.stamp = msg.header.stamp

        self.pub_.publish(self.noisy_odom_)
