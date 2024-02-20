/*
Copyright 2024 Giacomo Franchini
*/

#ifndef TOPIC_SYNCHRONIZER__TOPIC_SYNCHRONIZER_HPP_
#define TOPIC_SYNCHRONIZER__TOPIC_SYNCHRONIZER_HPP_

#include <geometry_msgs/msg/transform_stamped.h>
#include <sensor_msgs/msg/imu.hpp>
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
  std::string odom_source_topic_, odom_dest_topic_;
  std::string odom_gt_source_topic_, odom_gt_dest_topic_;
  std::string imu_source_topic_, imu_dest_topic_;
  
  float interval_duration_, approx_policy_;

  // Define subscribers to topics that we want to synchronize
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_subs_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_gt_subs_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> imu_subs_;

  using approximate_policy = message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry,
    nav_msgs::msg::Odometry,
    sensor_msgs::msg::Imu>;
  
  std::shared_ptr<message_filters::Synchronizer<approximate_policy>> time_sync_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_gt_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  void ts_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_gt_msg,
    const sensor_msgs::msg::Imu::ConstSharedPtr & imu_msg);
};
}  // namespace synchronizer

#endif  // TOPIC_SYNCHRONIZER__TOPIC_SYNCHRONIZER_HPP_
