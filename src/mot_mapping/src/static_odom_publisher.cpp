#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "static_odom_publisher");
  ros::NodeHandle nh;

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
  
  ros::Rate rate(50);  // 50Hz

  while (ros::ok()) {
    nav_msgs::Odometry odom;
    
    // 关键：使用当前ROS时间作为消息时间戳
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    
    // 位置在原点
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    
    // 姿态为单位四元数（无旋转）
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    odom.pose.pose.orientation.w = 1.0;
    
    // 速度为零
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = 0.0;
    
    odom_pub.publish(odom);
    rate.sleep();
  }

  return 0;
}
