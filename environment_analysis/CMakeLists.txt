cmake_minimum_required(VERSION 2.8.3)
project(environment_analysis)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge tf_object_detection zed_wrapper)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

## Declare a catkin package
catkin_package()

## Build
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(rtabmap_ros_listener MapListener.cpp SalientPoint.cpp Braccio.cpp kinematics/MatrixMath.cpp kinematics/BraccioKinematics.cpp GaussianMap.cpp GazeVisualizer.cpp MeshAnalyser.cpp GazeSolver.cpp)
target_link_libraries(rtabmap_ros_listener ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})