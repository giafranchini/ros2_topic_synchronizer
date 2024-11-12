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
using std::chrono::steady_clock;

TopicSynchronizer::TopicSynchronizer(const rclcpp::NodeOptions & options)
: Node("topic_synchronizer", options)
{
  odom_filtered_source = declare_parameter<std::string>("topics.source.odom_filtered", "/odometry_filtered");
  odom_gt_source = declare_parameter<std::string>("topics.source.odom_gt", "/odometry_gt");

  odom_filtered_out = declare_parameter<std::string>("topics.output.odom_filtered", "/odometry_filtered_sync");
  odom_gt_out = declare_parameter<std::string>("topics.output.odom_gt", "/odometry_gt_sync");

  interval_duration = declare_parameter<float>("interval_duration", 500000000.0);
  approx_policy = declare_parameter<int>("approx_policy", 10);

  odom_filtered_pub = create_publisher<nav_msgs::msg::Odometry>(odom_filtered_out, 10);
  odom_gt_pub = create_publisher<nav_msgs::msg::Odometry>(odom_gt_out, 10);

  odom_filtered_subs = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, odom_filtered_source);
  odom_gt_subs = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, odom_gt_source);

  time_sync = std::make_shared<message_filters::Synchronizer<approximate_policy>>(
    approximate_policy(approx_policy),
    *odom_filtered_subs,
    *odom_gt_subs);

  time_sync->setMaxIntervalDuration(rclcpp::Duration(0, interval_duration));
  time_sync->registerCallback(
    std::bind(&TopicSynchronizer::ts_callback, this, _1, _2));
}

void TopicSynchronizer::ts_callback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_filtered_msg,
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_gt_msg)
{
  rclcpp::Time odom_filtered_time = odom_filtered_msg->header.stamp;
  rclcpp::Time odom_gt_time = odom_gt_msg->header.stamp;
  
  RCLCPP_DEBUG(get_logger(), "Received messages");
  RCLCPP_DEBUG(get_logger(), 
    "Publishing messages at time: \n%f \n%f", 
    odom_filtered_msg->header.stamp.sec + odom_filtered_msg->header.stamp.nanosec * 1e-9,
    odom_gt_msg->header.stamp.sec + odom_gt_msg->header.stamp.nanosec * 1e-9);

  odom_filtered_pub->publish(*odom_filtered_msg);
  odom_gt_pub->publish(*odom_gt_msg);
}
}  // namespace synchronizer