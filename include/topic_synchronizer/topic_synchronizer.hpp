/*
Copyright 2024 Giacomo Franchini
*/

#ifndef TOPIC_SYNCHRONIZER__TOPIC_SYNCHRONIZER_HPP_
#define TOPIC_SYNCHRONIZER__TOPIC_SYNCHRONIZER_HPP_

#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
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
  std::string cmd_vel_source, imu_source, joint_state_source, odom_source, odom_filtered_source, odom_gt_source;
  std::string cmd_vel_out, imu_out, joint_state_out, odom_out, odom_filtered_out, odom_gt_out;
  
  float interval_duration, approx_policy;

  // Define subscribers to topics that we want to synchronize
  std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::TwistStamped>> cmd_vel_subs;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::JointState>> joint_state_subs;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> imu_subs;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_subs;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_filtered_subs;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_gt_subs;

  using approximate_policy = message_filters::sync_policies::ApproximateTime<
    geometry_msgs::msg::TwistStamped,
    sensor_msgs::msg::JointState,
    sensor_msgs::msg::Imu,
    nav_msgs::msg::Odometry,
    nav_msgs::msg::Odometry,
    nav_msgs::msg::Odometry>;
  
  std::shared_ptr<message_filters::Synchronizer<approximate_policy>> time_sync;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_filtered_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_gt_pub;

  void ts_callback(
    const geometry_msgs::msg::TwistStamped::ConstSharedPtr & cmd_vel_msg,
    const sensor_msgs::msg::JointState::ConstSharedPtr & joint_state_msg,
    const sensor_msgs::msg::Imu::ConstSharedPtr & imu_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_filtered_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_gt_msg);
};
}  // namespace synchronizer

#endif  // TOPIC_SYNCHRONIZER__TOPIC_SYNCHRONIZER_HPP_
