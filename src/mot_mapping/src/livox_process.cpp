#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

/**
 * livox_process 节点
 * 作用：订阅 Mid360 点云，并为其生成具有相同时间戳的零位姿里程计，以满足后续节点的同步要求。
 */

ros::Publisher odom_pub;

void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    nav_msgs::Odometry odom;
    odom.header.stamp = msg->header.stamp; // 关键：使用与点云完全一致的时间戳
    odom.header.frame_id = "world";
    odom.child_frame_id = "camera_init";
    
    // 零位姿初始化
    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation.w = 1.0;
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;

    odom_pub.publish(odom);
    ROS_INFO(1.0, "Livox process: Published odom with stamp %f", odom.header.stamp.toSec());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "livox_process");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string lidar_topic, odom_topic;
    pnh.param<std::string>("lidar_topic", lidar_topic, "/livox/lidar");
    pnh.param<std::string>("odom_topic", odom_topic, "/odom");

    ros::Subscriber lidar_sub = nh.subscribe(lidar_topic, 10, lidarCallback);
    odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 10);

    ROS_INFO("livox_process started, syncing odom with %s", lidar_topic.c_str());
    
    ros::spin();
    return 0;
}
