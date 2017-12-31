cmake_minimum_required(VERSION 2.8.3)
project(hope)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#Comment this if don't want debug
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  pcl_ros
  rospy
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

find_package(Boost COMPONENTS system filesystem regex REQUIRED)

catkin_package(

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/lib/fetch_rgbd.cpp
  src/lib/get_cloud.cpp
  src/lib/high_res_timer.cpp
  src/lib/transform.cpp
  src/lib/plane_segment.cpp
  src/lib/utilities.cpp

  src/lib/fetch_rgbd.h
  src/lib/get_cloud.h
  src/lib/high_res_timer.h
  src/lib/transform.h
  src/lib/plane_segment.h
  src/lib/utilities.h
)

# Declare a C++ executable
# With catkin_make all packages are built within a single CMake context
# The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/hope_node.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES} ${PROJECT_NAME} ${GLOG_LIB}
)
