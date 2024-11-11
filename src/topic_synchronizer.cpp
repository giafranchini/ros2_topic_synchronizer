/*
Copyright 2024 Giacomo Franchini
*/

#include <iostream>
#include <chrono>
#include <cmath>
#include <string>
#include <functional>
#include <cstdlib>
#include <memory>
#include <cassert>

#include "topic_synchronizer/topic_synchronizer.hpp"

namespace synchronizer
{

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;
using std::placeholders::_5;
using std::placeholders::_6;
using std::placeholders::_7;
using std::chrono::steady_clock;

TopicSynchronizer::TopicSynchronizer(const rclcpp::NodeOptions & options)
: Node("topic_synchronizer", options)
{
  cmd_vel_source = declare_parameter<std::string>("topics.source.cmd_vel", "/cmd_vel");
  feedback_source = declare_parameter<std::string>("topics.source.feedback", "/feedback");
  joint_state_source = declare_parameter<std::string>("topics.source.joint_state", "/joint_states");
  imu_source = declare_parameter<std::string>("topics.source.imu", "/imu");
  odom_source = declare_parameter<std::string>("topics.source.odom", "/odometry");
  odom_filtered_source = declare_parameter<std::string>("topics.source.odom_filtered", "/odometry_filtered");
  odom_gt_source = declare_parameter<std::string>("topics.source.odom_gt", "/odometry_gt");

  cmd_vel_out = declare_parameter<std::string>("topics.output.cmd_vel", "/cmd_vel_sync");
  feedback_out = declare_parameter<std::string>("topics.output.feedback", "/feedback_sync");
  joint_state_out = declare_parameter<std::string>("topics.output.joint_state", "/joint_states_sync");
  imu_out = declare_parameter<std::string>("topics.output.imu", "/imu_sync");
  odom_out = declare_parameter<std::string>("topics.output.odom", "/odometry_sync");
  odom_filtered_out = declare_parameter<std::string>("topics.output.odom_filtered", "/odometry_filtered_sync");
  odom_gt_out = declare_parameter<std::string>("topics.output.odom_gt", "/odometry_gt_sync");

  interval_duration = declare_parameter<float>("interval_duration", 500000000.0);
  approx_policy = declare_parameter<double>("approx_policy", 10.0);

  cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_out, 10);
  feedback_pub = create_publisher<jackal_msgs::msg::Feedback>(feedback_out, 10);
  joint_state_pub = create_publisher<sensor_msgs::msg::JointState>(joint_state_out, 10);
  imu_pub = create_publisher<sensor_msgs::msg::Imu>(imu_out, 10);
  odom_pub = create_publisher<nav_msgs::msg::Odometry>(odom_out, 10);
  odom_filtered_pub = create_publisher<nav_msgs::msg::Odometry>(odom_filtered_out, 10);
  odom_gt_pub = create_publisher<nav_msgs::msg::Odometry>(odom_gt_out, 10);

  cmd_vel_subs = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::Twist>>(this, cmd_vel_source);
  feedback_subs = std::make_shared<message_filters::Subscriber<jackal_msgs::msg::Feedback>>(this, feedback_source);
  joint_state_subs = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::JointState>>(this, joint_state_source);
  imu_subs = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, imu_source);
  odom_subs = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, odom_source);
  odom_filtered_subs = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, odom_filtered_source);
  odom_gt_subs = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, odom_gt_source);

  time_sync = std::make_shared<message_filters::Synchronizer<approximate_policy>>(
    approximate_policy(approx_policy),
    *cmd_vel_subs,
    *feedback_subs,
    *joint_state_subs,
    *imu_subs,
    *odom_subs,
    *odom_filtered_subs,
    *odom_gt_subs);

  time_sync_->setMaxIntervalDuration(rclcpp::Duration(0, interval_duration));
  time_sync_->registerCallback(
    std::bind(&TopicSynchronizer::ts_callback, this, _1, _2, _3, _4, _5, _6, _7));
}

void TopicSynchronizer::ts_callback(
  const geometry_msgs::msg::Twist::ConstSharedPtr & cmd_vel_msg,
  const jackal_msgs::msg::Feedback::ConstSharedPtr & feedback_msg,
  const sensor_msgs::msg::JointState::ConstSharedPtr & joint_state_msg,
  const sensor_msgs::msg::Imu::ConstSharedPtr & imu_msg,
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_filtered_msg,
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_gt_msg)
{
  rclcpp::Time cmd_vel_time = cmd_vel_msg->header.stamp;
  rclcpp::Time feedback_time = feedback_msg->header.stamp;
  rclcpp::Time joint_state_time = joint_state_msg->header.stamp;
  rclcpp::Time imu_time = imu_msg->header.stamp;
  rclcpp::Time odom_time = odom_msg->header.stamp;
  rclcpp::Time odom_filtered_time = odom_filtered_msg->header.stamp;
  rclcpp::Time odom_gt_time = odom_gt_msg->header.stamp;
  
  RCLCPP_DEBUG(get_logger(), "Received messages");
  RCLCPP_DEBUG(get_logger(), 
    "Publishing messages at time: \n%f \n%f \n%f", 
    cmd_vel_msg->header.stamp.sec + cmd_vel_msg->header.stamp.nanosec * 1e-9,
    feedback_msg->header.stamp.sec + feedback_msg->header.stamp.nanosec * 1e-9,
    joint_state_msg->header.stamp.sec + joint_state_msg->header.stamp.nanosec * 1e-9,
    imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9,
    odom_msg->header.stamp.sec + odom_msg->header.stamp.nanosec * 1e-9,
    odom_filtered_msg->header.stamp.sec + odom_filtered_msg->header.stamp.nanosec * 1e-9,
    odom_gt_msg->header.stamp.sec + odom_gt_msg->header.stamp.nanosec * 1e-9);
 
  cmd_vel_pub_->publish(*cmd_vel_msg);
  feedback_pub_->publish(*feedback_msg);
  joint_state_pub_->publish(*joint_state_msg);
  imu_pub_->publish(*imu_msg);
  odom_pub_->publish(*odom_msg);
  odom_filtered_pub_->publish(*odom_filtered_msg);
  odom_gt_pub_->publish(*odom_gt_msg);
}
}  // namespace synchronizer