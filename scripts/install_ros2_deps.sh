#!/usr/bin/env bash
set -euo pipefail

sudo apt-get update

sudo apt-get install -y \
  python3-numpy \
  ros-humble-ament-cmake-python \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-msgs \
  ros-humble-depth-image-proc \
  ros-humble-image-proc \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-point-cloud-transport \
  ros-humble-moveit \
  ros-humble-moveit-core \
  ros-humble-moveit-ros-planning-interface \
  ros-humble-gazebo-ros2-control \
  ros-humble-franka-description \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-rclcpp-components \
  ros-humble-realsense2-camera \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-tf2-ros \
  ros-humble-xacro \
  ros-humble-topic-tools
