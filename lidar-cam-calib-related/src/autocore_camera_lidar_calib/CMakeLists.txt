cmake_minimum_required(VERSION 3.0.2)
project(autocore_camera_lidar_calib)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
#SET(CMAKE_BUILD_TYPE "Debug")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set(Boost_LIBRARY_DIR /usr/lib/x86_64-linux-gnu/)

find_package(PCL REQUIRED)
find_package(OpenCV 3.2.0 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Yaml-cpp REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_executable(feature_extract src/feature_extract.cpp)
add_executable(visual_validate src/visual_validate.cpp)
add_executable(caculate_r_t src/caculate_r_t.cpp)

target_link_libraries(feature_extract 
  ${catkin_LIBRARIES}  ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES} yaml-cpp
)

target_link_libraries(visual_validate 
  ${catkin_LIBRARIES}  ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES} yaml-cpp
)

target_link_libraries(caculate_r_t 
  ${catkin_LIBRARIES}  ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES} yaml-cpp
)
