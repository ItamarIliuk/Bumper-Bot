#include "bumperbot_mapping/mapping_with_known_poses.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/utils.h"

using std::placeholders::_1;

MappingWithKnownPoses::MappingWithKnownPoses(const std::string &name)
    : Node(name)
{
    declare_parameter<double>("width", 100.0);
    declare_parameter<double>("height", 100.0);
    declare_parameter<double>("resolution", 0.1);

    double width = get_parameter("width").as_double();
    double height = get_parameter("height").as_double();
    map_.info.resolution = get_parameter("resolution").as_double();
    map_.info.width = width / map_.info.resolution;
    map_.info.height = height / map_.info.resolution;
    map_.info.origin.position.x = - width / 2.0;
    map_.info.origin.position.y = - height / 2.0;
    map_.header.frame_id = "odom";

    // Init map with prior probability
    map_.data = std::vector<int8_t>(map_.info.height * map_.info.width, -1);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&MappingWithKnownPoses::scanCallback, this, _1));
    rclcpp::QoS qos_profile_pub(10);
    qos_profile_pub.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos_profile_pub);
}

void MappingWithKnownPoses::poseToCell(const double px, const double py, unsigned int & c)
{
    c = map_.info.width * std::round((py - map_.info.origin.position.y) / map_.info.resolution) +
        std::round((px - map_.info.origin.position.x) / map_.info.resolution);
}

bool MappingWithKnownPoses::poseOnMap(const double px, const double py)
{
    return (px - map_.info.origin.position.x) < (map_.info.width * map_.info.resolution) && 
           (py - map_.info.origin.position.y) < (map_.info.height * map_.info.resolution);
}

void MappingWithKnownPoses::scanCallback(const sensor_msgs::msg::LaserScan &scan)
{
    geometry_msgs::msg::TransformStamped t;
    try
    {
        t = tf_buffer_->lookupTransform(map_.header.frame_id, scan.header.frame_id, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(get_logger(), "Unable to transform between /odom and /base_footprint");
        return;
    }

    // Check if pose is on the map
    if(!poseOnMap(t.transform.translation.x, t.transform.translation.y))
    {
        RCLCPP_ERROR(get_logger(), "The robot is out of the Map!");
        return;
    }

    tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    for (size_t i = 0; i < scan.ranges.size(); i++)
    {
      // Polar to cartesian coordinates
      double angle = scan.angle_min + (i * scan.angle_increment) + yaw;
      double px = scan.ranges.at(i) * std::cos(angle);
      double py = scan.ranges.at(i) * std::sin(angle);
      px += t.transform.translation.x;
      py += t.transform.translation.y;

      unsigned int cell;
      poseToCell(px, py, cell);

      map_.data.at(cell) = 100;
    }

    map_.header.stamp = get_clock()->now();
    map_pub_->publish(map_);
}

double MappingWithKnownPoses::prob2logodds(double p)
{
    return std::log(p / (1 - p));
}

double MappingWithKnownPoses::logodds2prob(double l)
{
    return 1 - (1 / (1 + std::exp(l)));
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MappingWithKnownPoses>("mapping_with_known_poses");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}