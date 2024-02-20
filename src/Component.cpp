/*
Copyright 2024 Giacomo Franchini
*/

#include "topic_synchronizer/topic_synchronizer.hpp"

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be
// discoverable when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(synchronizer::TopicSynchronizer)
