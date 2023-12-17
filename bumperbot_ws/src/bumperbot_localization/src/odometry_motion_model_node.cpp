#include <memory>

#include "bumperbot_localization/odometry_motion_model.hpp"
#include "rclcpp/rclcpp.hpp"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdometryMotionModel>("odometry_motion_model");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}