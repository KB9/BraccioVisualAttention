cmake_minimum_required(VERSION 2.8.3)
project(tf_object_detection)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs message_generation)

# Add object detection service file
add_service_files(FILES ObjectDetection.srv)

# Declare a catkin package
catkin_python_setup()
generate_messages(DEPENDENCIES std_msgs sensor_msgs)
catkin_package()

catkin_install_python(PROGRAMS src/object_detection/object_detection_demo.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})