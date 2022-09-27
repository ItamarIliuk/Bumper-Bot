#!/usr/bin/env python3
import rospy
from bumperbot_localization.simulate_motion_noise import MotionNoise


if __name__== '__main__':
    rospy.init_node('simulate_motion_noise_node')
    bumperbot = MotionNoise()

    rospy.spin()