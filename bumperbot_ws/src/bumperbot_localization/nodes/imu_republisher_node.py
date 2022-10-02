#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu


frame_id = ""

def imuCallback(imu):
    imu.header.frame_id = frame_id
    imu_pub.publish(imu)

if __name__== '__main__':
    rospy.init_node('imu_republisher_node')
    frame_id = rospy.get_param("~frame_id")

    imu_pub = rospy.Publisher("imu_ekf", Imu, queue_size=10)
    imu_sub = rospy.Subscriber("imu", Imu, imuCallback)

    rospy.spin()