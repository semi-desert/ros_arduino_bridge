cmake_minimum_required(VERSION 3.5)
project(ros_arduino_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

ament_export_dependencies(rclpy)

ament_package()