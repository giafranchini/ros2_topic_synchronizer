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
  cloud_source = declare_parameter<std::string>("topics.source.cloud", "/cloud");
  image_source = declare_parameter<std::string>("topics.source.image", "/image");
  camera_info_source = declare_parameter<std::string>("topics.source.camera_info", "/camera_info");

  cloud_out = declare_parameter<std::string>("topics.output.cloud", "/cloud_sync");
  image_out = declare_parameter<std::string>("topics.output.image", "/image_sync");
  camera_info_out = declare_parameter<std::string>("topics.output.camera_info", "/camera_info_sync");

  interval_duration = declare_parameter<float>("interval_duration", 500000000.0);
  approx_policy = declare_parameter<double>("approx_policy", 10.0);

  cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_out, 10);
  image_pub = create_publisher<sensor_msgs::msg::Image>(image_out, 10);
  camera_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_out, 10);

  cloud_subs = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, cloud_source);
  image_subs = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, image_source);
  camera_info_subs = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(this, camera_info_source);

  time_sync = std::make_shared<message_filters::Synchronizer<approximate_policy>>(
    approximate_policy(approx_policy),
    *cloud_subs,
    *image_subs,
    *camera_info_subs);

  time_sync->setMaxIntervalDuration(rclcpp::Duration(0, interval_duration));
  time_sync->registerCallback(
    std::bind(&TopicSynchronizer::ts_callback, this, _1, _2, _3));
}

void TopicSynchronizer::ts_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{ 
  RCLCPP_DEBUG(get_logger(), "Received messages");
  cloud_pub->publish(*cloud_msg);
  image_pub->publish(*image_msg);
  camera_info_pub->publish(*camera_info_msg);
}
}  // namespace synchronizer