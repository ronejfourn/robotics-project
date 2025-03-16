cmake_minimum_required(VERSION 3.8)
project(robotics-project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(arm_pose src/arm_pose.cpp)
ament_target_dependencies(arm_pose
    rclcpp
    moveit_ros_planning_interface
)

install(
    TARGETS arm_pose
    DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch meshes urdf config worlds maps src
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
