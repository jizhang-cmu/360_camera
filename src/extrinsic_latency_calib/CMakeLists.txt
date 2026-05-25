cmake_minimum_required(VERSION 3.5)
project(extrinsic_latency_calib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(jsoncpp QUIET)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
)

add_executable(extrinsicCalib src/extrinsicCalib.cpp)
target_link_libraries(extrinsicCalib rclcpp::rclcpp std_msgs::std_msgs sensor_msgs::sensor_msgs nav_msgs::nav_msgs message_filters::message_filters tf2::tf2 tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs pcl_conversions::pcl_conversions cv_bridge::cv_bridge)
target_link_libraries(extrinsicCalib ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(latencyCalib src/latencyCalib.cpp)
target_link_libraries(latencyCalib rclcpp::rclcpp std_msgs::std_msgs sensor_msgs::sensor_msgs nav_msgs::nav_msgs message_filters::message_filters tf2::tf2 tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs pcl_conversions::pcl_conversions cv_bridge::cv_bridge)
target_link_libraries(latencyCalib ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS
  extrinsicCalib latencyCalib
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  launch data
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
