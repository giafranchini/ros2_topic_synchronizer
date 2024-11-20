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
  odom_source0 = declare_parameter<std::string>("topics.source.odom0", "/odometry0");
  odom_source1 = declare_parameter<std::string>("topics.source.odom1", "/odometry1");

  odom_out0 = declare_parameter<std::string>("topics.output.odom0", "/odometry0/sync");
  odom_out1 = declare_parameter<std::string>("topics.output.odom1", "/odometry1/sync");

  interval_duration = declare_parameter<float>("interval_duration", 500000000.0);
  approx_policy = declare_parameter<int>("approx_policy", 10);

  odom_pub0 = create_publisher<nav_msgs::msg::Odometry>(odom_out0, 10);
  odom_pub1 = create_publisher<nav_msgs::msg::Odometry>(odom_out1, 10);

  odom_subs0 = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, odom_source0);
  odom_subs1 = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, odom_source1);

  time_sync = std::make_shared<message_filters::Synchronizer<approximate_policy>>(
    approximate_policy(approx_policy),
    *odom_subs0,
    *odom_subs1);

  time_sync->setMaxIntervalDuration(rclcpp::Duration(0, interval_duration));
  time_sync->registerCallback(
    std::bind(&TopicSynchronizer::ts_callback, this, _1, _2));
}

void TopicSynchronizer::ts_callback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg0,
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg1)
{
  odom_pub0->publish(*odom_msg0);
  odom_pub1->publish(*odom_msg1);
}
}  // namespace synchronizer