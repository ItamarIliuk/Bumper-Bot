#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "bumperbot_template/BumperbotAction.h"
#include "bumperbot_template/BumperbotSrv.h"
#include "bumperbot_template/BumperbotMsg.h"


class Bumperbot
{
public:
    Bumperbot(const ros::NodeHandle &);

private:
    void asCallback(const bumperbot_template::BumperbotGoalConstPtr &);

    bool srvCallback(bumperbot_template::BumperbotSrv::Request &,
                     bumperbot_template::BumperbotSrv::Response &);

    void msgCallback(const bumperbot_template::BumperbotMsg &);

    void timerCallback(const ros::TimerEvent &);


    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::ServiceClient srv_client_;
    ros::ServiceServer srv_server_;
    actionlib::SimpleActionClient<bumperbot_template::BumperbotAction> ac_;
    actionlib::SimpleActionServer<bumperbot_template::BumperbotAction> as_;
    bumperbot_template::BumperbotFeedback as_feedback_;
    bumperbot_template::BumperbotResult as_result_;
    ros::Timer timer_;
    int counter_;
};

