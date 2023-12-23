#include <string>
#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "twist_mux_msgs/action/joy_turbo.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

enum State{
  FREE = 0,
  WARNING,
  DANGER
};

class SafetyStop : public rclcpp::Node
{
public:
  SafetyStop() : Node("safety_stop_node"), state_{State::FREE}, prev_state_{State::FREE}
  {
    declare_parameter<double>("warning_distance", 0.6);
    declare_parameter<double>("danger_distance", 0.2);
    declare_parameter<std::string>("scan_topic", "scan");
    declare_parameter<std::string>("safety_stop_topic", "safety_stop");
    warning_distance_ = get_parameter("warning_distance").as_double();
    danger_distance_ = get_parameter("danger_distance").as_double();
    std::string scan_topic = get_parameter("scan_topic").as_string();
    std::string safety_stop_topic = get_parameter("safety_stop_topic").as_string();

    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, 10, std::bind(&SafetyStop::laserCallback, this, _1));
    safety_stop_pub_ = create_publisher<std_msgs::msg::Bool>(safety_stop_topic, 10);
    decrease_speed_client_ = rclcpp_action::create_client<twist_mux_msgs::action::JoyTurbo>(this, "joy_turbo_decrease");
    increase_speed_client_ = rclcpp_action::create_client<twist_mux_msgs::action::JoyTurbo>(this, "joy_turbo_increase");
    while(!decrease_speed_client_->wait_for_action_server(1s) && rclcpp::ok())
    {
      RCLCPP_WARN(get_logger(), "Action /joy_turbo_decrease not available! Waiting..");
      std::this_thread::sleep_for(2s);
    }
    while(!increase_speed_client_->wait_for_action_server(1s) && rclcpp::ok())
    {
      RCLCPP_WARN(get_logger(), "Action /joy_turbo_increase not available! Waiting..");
      std::this_thread::sleep_for(2s);
    }
  }

private:
  double warning_distance_, danger_distance_;
  State state_, prev_state_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_stop_pub_;
  rclcpp_action::Client<twist_mux_msgs::action::JoyTurbo>::SharedPtr decrease_speed_client_;
  rclcpp_action::Client<twist_mux_msgs::action::JoyTurbo>::SharedPtr increase_speed_client_;

  void laserCallback(const sensor_msgs::msg::LaserScan &msg)
  {
    state_ = State::FREE;
    for (const auto &range : msg.ranges)
    {
      if (range <= warning_distance_)
      {
        state_ = State::WARNING;
        
        if (range <= danger_distance_)
        {
          state_ = State::DANGER;
        }
      }
    }

    if(state_ != prev_state_)
    {
      std_msgs::msg::Bool is_safety_stop;
      if(state_ == State::WARNING)
      {
        is_safety_stop.data = false;
        decrease_speed_client_->async_send_goal(twist_mux_msgs::action::JoyTurbo::Goal());
      }
      else if(state_ == State::DANGER)
      {
        is_safety_stop.data = true;
      }
      else if(state_ == State::FREE)
      {
        is_safety_stop.data = false;
        increase_speed_client_->async_send_goal(twist_mux_msgs::action::JoyTurbo::Goal());
      }

      prev_state_ = state_;
      safety_stop_pub_->publish(is_safety_stop);
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyStop>());
  rclcpp::shutdown();
  return 0;
}