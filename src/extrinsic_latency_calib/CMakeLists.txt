cmake_minimum_required(VERSION 3.5)
project(extrinsic_latency_calib)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  /usr/local/include
  /usr/include
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(extrinsicCalib src/extrinsicCalib.cpp)
ament_target_dependencies(extrinsicCalib rclcpp std_msgs sensor_msgs nav_msgs pcl_ros cv_bridge)
target_link_libraries(extrinsicCalib ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(latencyCalib src/latencyCalib.cpp)
ament_target_dependencies(latencyCalib rclcpp std_msgs sensor_msgs nav_msgs pcl_ros cv_bridge)
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
