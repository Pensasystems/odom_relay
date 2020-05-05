
// ROS
#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include <tf/tf.h>

ros::Publisher odom_pub_;

geometry_msgs::Vector3 set_vector3(const double &x, const double &y, const double &z) {
  geometry_msgs::Vector3 v;
  v.x = x; v.y = y; v.z = z;
  return v;
}

geometry_msgs::Vector3 convert_to_body_frame(
    const geometry_msgs::Quaternion &quat,
    const geometry_msgs::Vector3 &vec) {
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Quaternion q_world(vec.x, vec.y, vec.z, 0.0);
  tf::Quaternion q_vec_body = q.inverse()*q_world*q;
  geometry_msgs::Vector3 vec_body = set_vector3(q_vec_body.x(), q_vec_body.y(), q_vec_body.z());
  return vec_body;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  geometry_msgs::Vector3 vel = msg->twist.twist.linear;
  geometry_msgs::Vector3 ang_vel = msg->twist.twist.angular;
  geometry_msgs::Quaternion quat = msg->pose.pose.orientation;

  // prepare the message for publishing
  nav_msgs::Odometry odom = *msg;
  odom.header.frame_id = "vislam_odom";
  odom.child_frame_id = "odom_frame";
  odom.twist.twist.linear = convert_to_body_frame(quat, vel);
  odom_pub_.publish(odom);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle n("~");

  std::string odom_topic, output_topic;
  n.getParam("odom_topic", odom_topic);
  n.getParam("output_topic", output_topic);

  ros::Subscriber sub_odom = n.subscribe(odom_topic, 5, odomCallback);
  odom_pub_ = n.advertise<nav_msgs::Odometry>(output_topic, 5);
  
  ROS_INFO("[odom_relay]: Subscribed to topic: %s", sub_odom.getTopic().c_str());
  ROS_INFO("[odom_relay]: Publishing into topic: %s", odom_pub_.getTopic().c_str());

  ros::spin();

  return 0;
} 