#!/usr/bin/env python
import rospy
from bumperbot_template.bumperbot import Bumperbot


if __name__== '__main__':
    rospy.init_node('bumperbot_node')
    bumperbot = Bumperbot()

    rospy.spin()