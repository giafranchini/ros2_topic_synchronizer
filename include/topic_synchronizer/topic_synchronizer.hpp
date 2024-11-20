/*
Copyright 2024 Giacomo Franchini
*/

#ifndef TOPIC_SYNCHRONIZER__TOPIC_SYNCHRONIZER_HPP_
#define TOPIC_SYNCHRONIZER__TOPIC_SYNCHRONIZER_HPP_

#include <nav_msgs/msg/odometry.hpp>

#include <array>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

namespace synchronizer
{

class TopicSynchronizer : public rclcpp::Node
{
public:
  explicit TopicSynchronizer(const rclcpp::NodeOptions & options);

  ~TopicSynchronizer() {}

private:
  std::string odom_source0, odom_source1, odom_source2;
  std::string odom_out0, odom_out1, odom_out2;
  
  float interval_duration;
  int approx_policy;

  // Define subscribers to topics that we want to synchronize
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_subs0;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_subs1;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_subs2;

  using approximate_policy = message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry,
    nav_msgs::msg::Odometry,
    nav_msgs::msg::Odometry>;
  
  std::shared_ptr<message_filters::Synchronizer<approximate_policy>> time_sync;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub0;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub1;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub2;

  void ts_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg0,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg1,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg2);
};
}  // namespace synchronizer

#endif  // TOPIC_SYNCHRONIZER__TOPIC_SYNCHRONIZER_HPP_
