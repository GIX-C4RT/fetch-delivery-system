cmake_minimum_required(VERSION 3.7.2)
project(fetch_delivery_system)

find_package(catkin REQUIRED 
  COMPONENTS 
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  std_msgs
  tf
  message_generation
  geometry_msgs
  shape_msgs
  visualization_msgs
  genmsg 
  simple_grasping
  pcl_conversions 
  pcl_ros
  interactive_markers
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  rviz_visual_tools
  moveit_visual_tools
  pluginlib
  geometric_shapes
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

include_directories(include ${catkin_INCLUDE_DIRS})

add_message_files(
  DIRECTORY
  msg
  FILES
  BoxTarget.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

# catkin_package()
catkin_package(CATKIN_DEPENDS message_runtime)

add_executable(segment_object src/segment_object.cpp)
target_link_libraries(segment_object ${catkin_LIBRARIES})
add_dependencies(segment_object ${catkin_EXPORTED_TARGETS})

install(
  PROGRAMS
    nodes/pick
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch maps config msg srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)