cmake_minimum_required(VERSION 3.5)
project(wit-imu-driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake  REQUIRED)
find_package(rclcpp       REQUIRED)
find_package(std_msgs     REQUIRED)
find_package(serial       REQUIRED)
find_package(sensor_msgs  REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

###########
## Build ##
###########
include_directories(include)

add_executable(${PROJECT_NAME}_node 
  src/wit_imu_node.cpp 
  src/JY901.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  serial
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()