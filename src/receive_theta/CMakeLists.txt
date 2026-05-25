cmake_minimum_required(VERSION 3.5)
project(receive_theta)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
)

add_executable(receiveTheta src/receiveTheta.cpp)
target_link_libraries(receiveTheta
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  cv_bridge::cv_bridge
  ${OpenCV_LIBRARIES})

install(TARGETS
  receiveTheta
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
