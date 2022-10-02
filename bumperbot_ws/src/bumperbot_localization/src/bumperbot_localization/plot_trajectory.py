#!/usr/bin/env python3
from time import sleep
import rospy
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class PlotTrajectory(object):

    def __init__(self, frame_id, topic_name):
        self.frame_id_ = frame_id
        self.topic_name_ = topic_name
        self.path_pub_ = rospy.Publisher(self.topic_name_, Path, queue_size=10)

        # Wait for the other nodes to begin publish messages
        rospy.sleep(1.0)

        self.timer_ = rospy.Timer(rospy.Duration(0.1), self.timerCallback)
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_)

        # Initialize the Path messages
        # Path - odom --> frame_id
        self.path_ = Path()
        self.path_.header.frame_id = "odom"

    
    def timerCallback(self, event):
        try:
            transform = self.tf_buffer_.lookup_transform("odom", self.frame_id_, rospy.Time())
        except Exception as e:
            rospy.logerr("Something went wrong %s" %e)
            return

        # Publish Path
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id_
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        self.path_.poses.append(pose)
        self.path_pub_.publish(self.path_)
      