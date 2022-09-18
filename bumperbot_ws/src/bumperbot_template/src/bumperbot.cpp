#include "bumperbot_template/bumperbot.h"


Bumperbot::Bumperbot(const ros::NodeHandle &nh) 
                    : nh_(nh),
                      as_(nh_, "bumperbot_action", boost::bind(&Bumperbot::asCallback, this, _1), false),
                      ac_("bumperbot_action", true),
                      counter_(0)
{
  pub_ = nh_.advertise<bumperbot_template::BumperbotMsg>("bumperbot_publisher", 10);
  sub_ = nh_.subscribe("bumperbot_publisher", 1000, &Bumperbot::msgCallback, this);
  srv_server_ = nh_.advertiseService("bumperbot_service", &Bumperbot::srvCallback, this);
  srv_client_ = nh_.serviceClient<bumperbot_template::BumperbotSrv>("bumperbot_service");
  timer_ = nh_.createTimer(ros::Duration(5.0), &Bumperbot::timerCallback, this);

  as_.start();

  // Check that Service Server exists
  if(srv_client_.waitForExistence())
  {
    ROS_INFO_STREAM("Service " << srv_client_.getService() << " is ready");
  }

  // Check that Action Server exists
  if(ac_.waitForServer())
  {
    ROS_INFO_STREAM("Action bumperbot_action is ready");
  }

}


void Bumperbot::asCallback(const bumperbot_template::BumperbotGoalConstPtr &goal)
{
  ROS_INFO_STREAM("Requested to count up to " << goal->order);
  ros::Rate r(1);
  bool success = true;

  as_feedback_.sequence.reserve(abs(goal->order));
  as_result_.sequence.reserve(abs(goal->order));

  for(int i = 0; i < abs(goal->order); i++)
  {
    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("Action Server Stopped");
      as_.setPreempted();
      success = false;
      break;
    }

    ROS_INFO_STREAM("Counting ... " << i);
    as_feedback_.sequence.push_back(i);
    as_.publishFeedback(as_feedback_);
    r.sleep();
  }

  if(success)
  {
    as_result_.sequence = as_feedback_.sequence;
    ROS_INFO("Action Server Succeeded");
    as_.setSucceeded(as_result_);
  }
}


bool Bumperbot::srvCallback(bumperbot_template::BumperbotSrv::Request &req,
                            bumperbot_template::BumperbotSrv::Response &res)
{
  // Adds the two integers in the request message and returns their sum 
  ROS_INFO_STREAM("Requested to sum " << req.a << " and " << req.b);
  res.sum = req.a + req.b;
  ROS_INFO_STREAM("Returning result " << res.sum);
  return true;
}


void Bumperbot::msgCallback(const bumperbot_template::BumperbotMsg &msg)
{
  ROS_INFO_STREAM("New message received: " << msg.message);
}


void Bumperbot::timerCallback(const ros::TimerEvent &event)
{
  ROS_INFO_STREAM("Timer Triggered, Publishing new message");
  bumperbot_template::BumperbotMsg msg;
  std::stringstream ss;
  ss << "Timer message " << counter_++;
  msg.message = ss.str();
  pub_.publish(msg);
}