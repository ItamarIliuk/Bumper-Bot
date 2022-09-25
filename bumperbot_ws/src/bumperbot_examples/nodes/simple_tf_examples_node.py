#!/usr/bin/env python3
import rospy
from bumperbot_examples.simple_tf_examples import SimpleTfExamples


if __name__== '__main__':
    rospy.init_node('simple_tf_examples')
    examples = SimpleTfExamples()

    rospy.spin()