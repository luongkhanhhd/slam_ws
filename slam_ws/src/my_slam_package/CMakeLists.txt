cmake_minimum_required(VERSION 3.8)
project(my_slam_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(rviz2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rplidar_ros REQUIRED)

install(DIRECTORY
  config
  launch
  rviz
  maps
  scripts
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
PROGRAMS
scripts/odom_publisher.py
scripts/vel_to_serial.py
scripts/waypoint_navigator.py
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
