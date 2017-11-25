cmake_minimum_required(VERSION 2.8.3)
project(environment_analysis)

set(CMAKE_CXX_STANDARD 14)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge)

find_package(OpenCV)

## Declare a catkin package
catkin_package()

## Build
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(rtabmap_ros_listener MapListener.cpp SalientPoint.cpp kinematics/MatrixMath.cpp kinematics/BraccioKinematics.cpp)
target_link_libraries(rtabmap_ros_listener ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})