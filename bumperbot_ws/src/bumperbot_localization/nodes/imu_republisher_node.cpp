#include <ros/ros.h>
#include <sensor_msgs/Imu.h>


std::string frame_id = "";
ros::Publisher imu_pub;

void imuCallback(const sensor_msgs::Imu &imu)
{
    sensor_msgs::Imu new_imu;
    new_imu = imu;
    new_imu.header.frame_id = frame_id;
    imu_pub.publish(new_imu);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_republisher_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.getParam("frame_id", frame_id);

    imu_pub = nh.advertise<sensor_msgs::Imu>("imu_ekf", 10);
    ros::Subscriber imu_sub = nh.subscribe("imu", 1000, imuCallback);


    ros::spin();
    return 0;
}