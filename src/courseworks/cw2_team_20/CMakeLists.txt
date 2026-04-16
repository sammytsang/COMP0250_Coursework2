cmake_minimum_required(VERSION 3.8)
project(cw2_team_20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(cw2_world_spawner REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_cw2_class_lib
  src/cw2_class.cpp
  src/cw2_task1.cpp
  src/cw2_task2.cpp
  src/cw2_task3.cpp
  src/cw2_perception.cpp
  src/cw2_motion.cpp
)

ament_target_dependencies(${PROJECT_NAME}_cw2_class_lib
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  octomap
  octomap_msgs
  shape_msgs
  gazebo_msgs
  cw2_world_spawner
)

target_link_libraries(${PROJECT_NAME}_cw2_class_lib
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${OpenCV_LIBS}
  JsonCpp::JsonCpp
)

add_executable(cw2_solution_node src/cw2_node.cpp)

ament_target_dependencies(cw2_solution_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  octomap
  octomap_msgs
  shape_msgs
  gazebo_msgs
  cw2_world_spawner
)

target_link_libraries(cw2_solution_node
  ${PROJECT_NAME}_cw2_class_lib
)

install(TARGETS
  ${PROJECT_NAME}_cw2_class_lib
  cw2_solution_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.launch" EXCLUDE
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
