cmake_minimum_required(VERSION 3.8)
project(multi_lidar_calib)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(rviz2_capture_plugin_interface REQUIRED)
find_package(FAST_GICP REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs sensor_msgs autoware_auto_vehicle_msgs rviz2_capture_plugin_interface)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES}  ${FAST_GICP_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${FAST_GICP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

  install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
