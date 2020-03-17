cmake_minimum_required(VERSION 2.8.3)
project(orbit_generation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometric_shapes
  interactive_markers
  moveit_msgs
  moveit_ros_perception
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  pluginlib
  roscpp
  ezgripper_cpp_driver
  fcsc_msgs
  fcsc_visual_tools
  trajectory_data_handler
  base_placement_planner
  fcsc_moveit
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roscpp
  CATKIN_DEPENDS
    geometric_shapes
    interactive_markers
    moveit_core
    moveit_msgs
    moveit_ros_perception
    moveit_ros_planning_interface
    pluginlib
    roscpp
    fcsc_msgs
    moveit_visual_tools
    ezgripper_cpp_driver
    fcsc_visual_tools
    comm_client
    trajectory_data_handler
    visualization_msgs
    base_placement_planner
    fcsc_moveit
)
add_executable(orbit_generation src/orbit_generation.cpp)
target_link_libraries(orbit_generation ${catkin_LIBRARIES})
#add_dependencies(orbit_generation ${catkin_EXPORTED_TARGETS})
add_executable(yuka src/yuka.cpp)
target_link_libraries(yuka ${catkin_LIBRARIES})
#add_dependencies(yuka ${catkin_EXPORTED_TARGETS})
add_executable(move_base src/move_base.cpp)
target_link_libraries(move_base ${catkin_LIBRARIES})
#add_dependencies(move_base ${catkin_EXPORTED_TARGETS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)