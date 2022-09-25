#!/usr/bin/env python3
import rospy
from bumperbot_template.bumperbot import Bumperbot


if __name__== '__main__':
    rospy.init_node('bumperbot_node')
    bumperbot = Bumperbot()

    rospy.spin()