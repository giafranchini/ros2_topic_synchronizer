cmake_minimum_required(VERSION 3.16)
project(topic_synchronizer)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_library(topic_synchronizer_node SHARED src/topic_synchronizer.cpp)
target_include_directories(
  topic_synchronizer_node
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  PRIVATE
)
target_link_libraries(
  topic_synchronizer_node
  message_filters::message_filters
  rclcpp::rclcpp
  rcutils::rcutils
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
)

add_library(topic_synchronizer SHARED src/Component.cpp)
target_include_directories(
  topic_synchronizer
  PRIVATE
  include
)
target_link_libraries(
  topic_synchronizer
  topic_synchronizer_node
  rclcpp_components::component
)

rclcpp_components_register_node(
  topic_synchronizer
  PLUGIN "synchronizer::TopicSynchronizer"
  EXECUTABLE TopicSynchronizerExecutable
)

install(
  TARGETS topic_synchronizer topic_synchronizer_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  rcutils
  rclcpp_components
  message_filters
  geometry_msgs
  sensor_msgs
  nav_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
