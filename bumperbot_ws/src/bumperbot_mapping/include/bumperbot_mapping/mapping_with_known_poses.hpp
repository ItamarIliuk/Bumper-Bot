#ifndef MAPPING_WITH_KNOWN_POSES_HPP
#define MAPPING_WITH_KNOWN_POSES_HPP

#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

struct Pose
{
    Pose() = default;
    Pose(const int px, const int py) : x(px), y(py){}

    int x;
    int y;
};

class MappingWithKnownPoses : public rclcpp::Node
{
public:
    MappingWithKnownPoses(const std::string& name);

private:
    void scanCallback(const sensor_msgs::msg::LaserScan & scan);

    double prob2logodds(double p);

    double logodds2prob(double l);

    std::vector<uint8_t> inverseSensorModel(const Pose & p_robot, const Pose & p_beam);

    std::vector<Pose> bresenham(const Pose & start, const Pose & end);

    unsigned int poseToCell(const Pose & pose);

    Pose coordinatesToPose(const double px, const double py);
    
    bool poseOnMap(const Pose & pose);

    nav_msgs::msg::OccupancyGrid map_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    
};

#endif // MAPPING_WITH_KNOWN_POSES_HPP