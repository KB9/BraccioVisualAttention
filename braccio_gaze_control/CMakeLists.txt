cmake_minimum_required(VERSION 2.8.3)
project(braccio_gaze_control)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rospy std_msgs)

## Declare a catkin package
catkin_package()
catkin_python_setup()

catkin_install_python(PROGRAMS braccio_gaze_controller.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})