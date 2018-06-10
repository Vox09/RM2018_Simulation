cmake_minimum_required(VERSION 2.8.3)
project(RM_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  gazebo_msgs 
  roscpp 
  rospy 
  nodelet 
  angles 
  std_srvs 
  geometry_msgs 
  sensor_msgs 
  nav_msgs 
  urdf 
  tf 
  tf2_ros 
  dynamic_reconfigure  
  rosgraph_msgs 
  trajectory_msgs 
  image_transport 
  rosconsole
  cv_bridge
  polled_camera
  diagnostic_updater
  camera_info_manager
  std_msgs
)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
include_directories(include)

link_directories(${GAZEBO_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(RMDrive SHARED src/RMDrive.cpp)
target_link_libraries(RMDrive ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})