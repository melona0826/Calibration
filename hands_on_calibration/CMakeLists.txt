cmake_minimum_required(VERSION 2.8.3)
project(hands_on_calibration)


find_package(catkin REQUIRED COMPONENTS
      interactive_markers
      moveit_core
      moveit_visual_tools
      moveit_ros_planning
      moveit_ros_planning_interface
      pluginlib
      geometric_shapes
      pcl_ros
      pcl_conversions
      rosbag
      tf2_ros
      tf2_eigen
      tf2_geometry_msgs
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_definitions(-std=c++11)
add_executable(move_pose src/move_pose.cpp)
target_link_libraries(move_pose ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_pose DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
