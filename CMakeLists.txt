cmake_minimum_required(VERSION 2.8.3)
project(hope)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

#Comment this if don't want debug
set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1")

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

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

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
  src/lib/z_growing.cpp
  src/lib/transform.cpp
  src/lib/plane_segment.cpp
  src/lib/utilities.cpp

  src/lib/fetch_rgbd.h
  src/lib/get_cloud.h
  src/lib/high_res_timer.h
  src/lib/z_growing.h
  src/lib/transform.h
  src/lib/plane_segment.h
  src/lib/utilities.h
)

add_library(fmc
  src/lib/fmc/findClique.cpp
  src/lib/fmc/findCliqueHeu.cpp
  src/lib/fmc/graphIO.cpp
  src/lib/fmc/utils.cpp

  src/lib/fmc/findClique.h
  src/lib/fmc/graphIO.h
)

add_executable(test_fmc src/lib/fmc/main_fmc.cpp)
target_link_libraries(test_fmc fmc)

# Notice that the customized lib needs to link to ROS too
target_link_libraries(${PROJECT_NAME} fmc ${catkin_LIBRARIES})

# Declare a C++ executable
# With catkin_make all packages are built within a single CMake context
# The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/hope_node.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${GLOG_LIB})

add_executable(view_cloud src/view_cloud.cpp)
target_link_libraries(view_cloud ${PCL_LIBRARIES})
