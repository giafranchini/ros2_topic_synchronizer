/*
Copyright 2024 Giacomo Franchini
*/

#ifndef TOPIC_SYNCHRONIZER__TOPIC_SYNCHRONIZER_HPP_
#define TOPIC_SYNCHRONIZER__TOPIC_SYNCHRONIZER_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

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
  std::string cloud_source, image_source, camera_info_source;
  std::string cloud_out, image_out, camera_info_out;
  
  float interval_duration, approx_policy;

  // Define subscribers to topics that we want to synchronize
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_subs;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_subs;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> camera_info_subs;

  using approximate_policy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo>;
  
  std::shared_ptr<message_filters::Synchronizer<approximate_policy>> time_sync;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;

  void ts_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg);
};
}  // namespace synchronizer

#endif  // TOPIC_SYNCHRONIZER__TOPIC_SYNCHRONIZER_HPP_
