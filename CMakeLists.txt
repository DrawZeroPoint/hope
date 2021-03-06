cmake_minimum_required(VERSION 2.8.3)
project(hope)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#Comment this if don't want debug
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  actionlib_msgs
  image_transport
  pcl_ros
  rospy
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

# Set BOOST_LIBRARYDIR accordingly in case that PCL could not find Boost
set(BOOST_LIBRARYDIR /usr/lib/x86_64-linux-gnu)

find_package(Boost COMPONENTS system filesystem regex REQUIRED)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
message("Found OpenCV ${OpenCV_VERSION}")

add_service_files(
  FILES
  ExtractObjectOnTop.srv
  GetObjectPose.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
)

# Add dynamic reconfigure API
generate_dynamic_reconfigure_options(cfg/hope.cfg)

catkin_package(
  CATKIN_DEPENDS 
  cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/lib/utilities.cpp
  src/lib/utilities.h
  src/lib/pose_estimation.cpp
  src/lib/pose_estimation.h
  src/lib/palletization.cpp
  src/lib/palletization.h

  src/lib/fetch_rgbd.cpp
  src/lib/get_cloud.cpp
  src/lib/high_res_timer.cpp
  src/lib/z_growing.cpp
  src/lib/transform.cpp
  src/lib/plane_segment.cpp

  src/lib/fetch_rgbd.h
  src/lib/get_cloud.h
  src/lib/high_res_timer.h
  src/lib/z_growing.h
  src/lib/transform.h
  src/lib/plane_segment.h
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Declare C++ executables

add_executable(${PROJECT_NAME}_node src/hope_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${GLOG_LIB}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_ros src/hope_ros.cpp)
target_link_libraries(${PROJECT_NAME}_ros
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${GLOG_LIB}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

add_executable(${PROJECT_NAME}_palletization src/hope_palletization.cpp)
target_link_libraries(${PROJECT_NAME}_palletization
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${GLOG_LIB}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

add_executable(pub_cloud src/pub_cloud.cpp)
target_link_libraries(pub_cloud
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${GLOG_LIB}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

add_executable(view_cloud src/view_cloud.cpp)
target_link_libraries(view_cloud ${PCL_LIBRARIES})

add_executable(hope_test src/hope_test.cpp)
target_link_libraries(hope_test
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${PROJECT_NAME}
)
