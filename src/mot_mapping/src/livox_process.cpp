#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * livox_process 节点
 * 作用：
 * 1. 订阅 Mid360 的 CustomMsg 格式点云。
 * 2. 将其转换为标准的 PointCloud2 格式并发布。
 * 3. 为每一帧点云生成具有相同时间戳的零位姿里程计。
 */

ros::Publisher odom_pub;
ros::Publisher cloud_pub;

void livoxCustomMsgCallback(const livox_ros_driver2::CustomMsgConstPtr& msg) {
    // 1. 转换 CustomMsg -> PCL PointCloud
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl_cloud.header.frame_id = "camera_init"; // 或者根据实际驱动设置的 frame_id
    pcl_cloud.header.stamp = msg->header.stamp.toNSec() / 1000; // PCL 使用微秒

    for (const auto& p : msg->points) {
        pcl::PointXYZI pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;
        pt.intensity = p.reflectivity;
        pcl_cloud.points.push_back(pt);
    }

    // 2. 转换 PCL -> PointCloud2 并发布
    sensor_msgs::PointCloud2 output_cloud;
    pcl::toROSMsg(pcl_cloud, output_cloud);
    output_cloud.header.stamp = msg->header.stamp;
    output_cloud.header.frame_id = "camera_init";
    cloud_pub.publish(output_cloud);

    // 3. 构造并发布相同时间戳的里程计
    nav_msgs::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "world";
    odom.child_frame_id = "camera_init";
    
    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation.w = 1.0;
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;

    odom_pub.publish(odom);

    ROS_INFO_THROTTLE(2.0, "Livox process: Converted msg and published odom at stamp %f", odom.header.stamp.toSec());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "livox_process");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string lidar_topic_in, cloud_topic_out, odom_topic_out;
    pnh.param<std::string>("lidar_topic", lidar_topic_in, "/livox/lidar");
    pnh.param<std::string>("cloud_topic_processed", cloud_topic_out, "/livox/points_processed");
    pnh.param<std::string>("odom_topic", odom_topic_out, "/odom");

    ros::Subscriber lidar_sub = nh.subscribe(lidar_topic_in, 10, livoxCustomMsgCallback);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic_out, 10);
    odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic_out, 10);

    ROS_INFO("livox_process started, subscribing to CustomMsg: %s", lidar_topic_in.c_str());
    
    ros::spin();
    return 0;
}
