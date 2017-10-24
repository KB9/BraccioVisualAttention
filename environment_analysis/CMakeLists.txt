cmake_minimum_required(VERSION 2.8.3)
project(environment_analysis)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

## Declare ROS messages and services
#add_message_files(FILES sensor_msgs/PointCloud2.msg)

## Generate added messages and services
#generate_messages(DEPENDENCIES sensor_msgs)

## Declare a catkin package
catkin_package()

## Build
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(map_listener map_listener.cpp)
target_link_libraries(map_listener ${catkin_LIBRARIES})
#add_dependencies(map_listener map_listener_generate_messages_cpp)