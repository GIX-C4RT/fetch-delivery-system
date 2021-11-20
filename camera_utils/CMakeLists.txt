cmake_minimum_required(VERSION 3.7.2)
project(camera_utils)

find_package(catkin REQUIRED 
  COMPONENTS 
  roscpp 
  rospy 
  sensor_msgs 
  cv_bridge
  std_msgs
  tf
  message_generation)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(OpenCV REQUIRED PATHS /usr/local/share/OpenCV NO_DEFAULT_PATH )
find_package(aruco REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

include_directories(include ${catkin_INCLUDE_DIRS} /usr/local/include/aruco)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

# catkin_package()
catkin_package(CATKIN_DEPENDS message_runtime)

add_executable(camera_pose src/camera_pose.cpp)
target_link_libraries(camera_pose ${catkin_LIBRARIES} ${OpenCV_LIBS} ${aruco_LIBS})
add_dependencies(camera_pose ${catkin_EXPORTED_TARGETS})