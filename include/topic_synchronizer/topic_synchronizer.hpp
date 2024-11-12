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
  std::string odom_filtered_source, odom_gt_source;
  std::string odom_filtered_out, odom_gt_out;
  
  float interval_duration;
  int approx_policy;

  // Define subscribers to topics that we want to synchronize
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_filtered_subs;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_gt_subs;

  using approximate_policy = message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry,
    nav_msgs::msg::Odometry>;
  
  std::shared_ptr<message_filters::Synchronizer<approximate_policy>> time_sync;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_filtered_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_gt_pub;

  void ts_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_filtered_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_gt_msg);
};
}  // namespace synchronizer

#endif  // TOPIC_SYNCHRONIZER__TOPIC_SYNCHRONIZER_HPP_
