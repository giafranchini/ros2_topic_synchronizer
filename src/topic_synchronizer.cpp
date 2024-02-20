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
using std::chrono::steady_clock;

TopicSynchronizer::TopicSynchronizer(const rclcpp::NodeOptions & options)
: Node("topic_synchronizer", options)
{
  odom_source_topic_ = declare_parameter<std::string>("topics.odom_source", "/odometry");
  odom_gt_source_topic_ = declare_parameter<std::string>("topics.odom_gt_source", "/odometry_gt");
  imu_source_topic_ = declare_parameter<std::string>("topics.imu_source", "/imu");
  odom_dest_topic_ = declare_parameter<std::string>("topics.odom_output", "/odometry_sync");
  odom_gt_dest_topic_ = declare_parameter<std::string>("topics.odom_gt_output", "/odometry_gt_sync");
  imu_dest_topic_ = declare_parameter<std::string>("topics.imu_output", "/imu_sync");
  interval_duration_ = declare_parameter<float>("interval_duration", 500000000.0);
  approx_policy_ = declare_parameter<double>("approx_policy", 10.0);

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_dest_topic_, 10);
  odom_gt_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_gt_dest_topic_, 10);
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_dest_topic_, 10);
  odom_subs_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, odom_source_topic_);
  odom_gt_subs_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, odom_gt_source_topic_);
  imu_subs_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, imu_source_topic_);
  time_sync_ = std::make_shared<message_filters::Synchronizer<approximate_policy>>(
    approximate_policy(approx_policy_),
    *odom_subs_,
    *odom_gt_subs_,
    *imu_subs_);
  time_sync_->setMaxIntervalDuration(rclcpp::Duration(0, interval_duration_));
  time_sync_->registerCallback(std::bind(&TopicSynchronizer::ts_callback, this, _1, _2, _3));
}

void TopicSynchronizer::ts_callback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_gt_msg,
  const sensor_msgs::msg::Imu::ConstSharedPtr & imu_msg)
{
  rclcpp::Time odom_time = odom_msg->header.stamp;
  rclcpp::Time odom_gt_time = odom_gt_msg->header.stamp;
  rclcpp::Time imu_time = imu_msg->header.stamp;
  
  RCLCPP_DEBUG(get_logger(), "Received messages");
  RCLCPP_DEBUG(get_logger(), 
    "Publishing messages at time: \n%f \n%f \n%f", 
    odom_msg->header.stamp.sec + odom_msg->header.stamp.nanosec * 1e-9,
    odom_gt_msg->header.stamp.sec + odom_gt_msg->header.stamp.nanosec * 1e-9,
    imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9);
 
  odom_pub_->publish(*odom_msg);
  odom_gt_pub_->publish(*odom_gt_msg);
  imu_pub_->publish(*imu_msg);
}
}  // namespace synchronizer