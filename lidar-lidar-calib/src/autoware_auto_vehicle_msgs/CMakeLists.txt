# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(autoware_auto_vehicle_msgs)

# Generate messages
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VelocityReport.idl"
  DEPENDENCIES
    "std_msgs"
)



ament_package()
