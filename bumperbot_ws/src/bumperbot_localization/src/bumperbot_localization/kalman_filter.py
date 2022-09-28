#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


class KalmanFilter(object):

    def __init__(self):
        self.odom_sub_ = rospy.Subscriber("odom_noisy", Odometry, self.odomCallback)
        self.imu_sub_ = rospy.Subscriber("imu", Imu, self.imuCallback)
        self.filter_pub_ = rospy.Publisher("kalman_filter", Float64, queue_size=10)
        
        # Initially the robot has no idea about how fast is going
        self.mean_ = 0.0
        self.standard_deviation_ = 1000.0

        # Modeling the uncertainty of the sensor and the motion
        self.motion_standard_deviation_ = 1.0
        self.measurement_standard_deviation_ = 0.05

        # Store the messages - only for the angular velocity
        self.last_imu_ = 0.0
        self.last_motion_ = 0.0
        self.motion_ = 0.0
        self.imu_ = 0.0


    def odomCallback(self, odom):
        self.motion_ = odom.twist.twist.angular.z - self.last_motion_
        self.last_motion_ = odom.twist.twist.angular.z

        self.statePrediction()
        self.measurementUpdate()

        self.filter_pub_.publish(self.mean_)


    def imuCallback(self, imu):
        self.imu_ = imu.angular_velocity.z - self.last_imu_
        self.last_imu_ = imu.angular_velocity.z 


    def measurementUpdate(self):
        # self.mean_ = (self.measurement_standard_deviation_ * self.mean_ + self.standard_deviation_ * self.last_imu_) \
        #            / (self.standard_deviation_ + self.measurement_standard_deviation_)
        self.mean_ = (1 / (self.standard_deviation_ + self.measurement_standard_deviation_)) *  \
                     (self.measurement_standard_deviation_ * self.mean_ + self.standard_deviation_ * self.last_imu_)
        # self.standard_deviation_ = 1 / (1 / self.standard_deviation_ + 1 / self.measurement_standard_deviation_)
        self.standard_deviation_ = (self.standard_deviation_ * self.motion_standard_deviation_) \
                                  /(self.standard_deviation_ + self.motion_standard_deviation_)

    def statePrediction(self):
        self.mean_ = self.mean_ + self.motion_
        self.standard_deviation_ = self.standard_deviation_ + self.motion_standard_deviation_