#!/usr/bin/env python3
import rospy
import actionlib
from bumperbot_template.msg import BumperbotMsg, BumperbotAction, BumperbotResult, BumperbotFeedback
from bumperbot_template.srv import BumperbotSrv, BumperbotSrvResponse


class Bumperbot(object):

    def __init__(self):
        self.counter_ = 0
        self.pub_ = rospy.Publisher("bumperbot_publisher", BumperbotMsg, queue_size=10)
        self.sub_ = rospy.Subscriber("bumperbot_publisher", BumperbotMsg, self.msgCallback)
        self.srv_server_ = rospy.Service("bumperbot_service", BumperbotSrv, self.srvCallback)
        self.srv_client_ = rospy.ServiceProxy("bumperbot_service", BumperbotSrv)
        self.as_ = actionlib.SimpleActionServer("bumperbot_action", BumperbotAction, execute_cb=self.asCallback, auto_start = False)
        self.ac_ = actionlib.SimpleActionClient("bumperbot_action", BumperbotAction)
        self.as_feedback_ = BumperbotFeedback()
        self.as_result_ = BumperbotResult()
        self.timer_ = rospy.Timer(rospy.Duration(5.0), self.timerCallback)
        
        self.as_.start()

        self.srv_client_.wait_for_service()
        rospy.loginfo("Service bumperbot_service is ready")

        self.ac_.wait_for_server()
        rospy.loginfo("Action bumperbot_action is ready")


    def asCallback(self, goal):
        rospy.loginfo("Requested to count up to %d", goal.order)
        r = rospy.Rate(1.0)
        success = True

        for i in range(0, abs(goal.order)):
            if self.as_.is_preempt_requested():
                rospy.loginfo("Action Server Stopped")
                self.as_.set_preempted()
                success = False
                break

            rospy.loginfo("Counting ... %d", i)
            self.as_feedback_.sequence.append(i)
            self.as_.publish_feedback(self.as_feedback_)
            r.sleep()
        
        if success:
            self.as_result_.sequence = self.as_feedback_.sequence
            rospy.loginfo("Action Server Succeeded")
            self.as_.set_succeeded(self.as_result_)

    
    def srvCallback(self, req):
        rospy.loginfo("Requested to sum %d and %d", req.a, req.b)
        res = BumperbotSrvResponse()
        res.sum = req.a + req.b
        rospy.loginfo("Returning result %d", res.sum)
        return res


    def msgCallback(self, msg):
        rospy.loginfo("New message received: %s", msg.message)


    def timerCallback(self, event):
        rospy.loginfo("Timer Triggered, Publishing new message")
        msg = "Timer message %d" % self.counter_
        self.pub_.publish(BumperbotMsg(msg))
        self.counter_ += 1
