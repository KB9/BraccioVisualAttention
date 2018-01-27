# BraccioVisualAttention ![CI status](https://img.shields.io/badge/build-passing-brightgreen.svg)

BraccioVisualAttention 

## Installation
Create a catkin workspace:
```
mkdir -p ~/catkin_ws/src
```
Navigate to the `src` directory in your catkin workspace and clone this repository:
```
cd ~/catkin_ws/src
git clone https://github.com/KB9/BraccioVisualAttention.git
```
Build the project using catkin:
```
cd ~/catkin_ws
catkin_make install
```

### Requirements
* Ubuntu 16.04 LTS
* ROS Kinetic

## Usage

There are 4 parts to this project:
* **braccio_gaze_control**: This is responsible for the communication between your computer and the Braccio's Arduino.
* **environment_analysis**: Performs analysis on the data from the camera feed, and selects the most important features to look at.
* **tf_object_detection**: A ROS wrapper for the [TensorFlow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection), allowing objects to be detected in the camera feed.
* **zed-ros-wrapper**: A modified version [ROS wrapper for the ZED stereo camera](http://wiki.ros.org/zed-ros-wrapper), whose modification allows it to generate and save an environment mesh.

### Communication with the Braccio
To send movement commands to the Braccio, the Braccio's Arduino Yun must be first connected to the same Wi-Fi hotspot as your computer:
* Connect to the Wi-Fi hotspot created by the Yun.
* Visit http://arduino3.local
* Follow the instructions to connect the Yun to the Wi-Fi hotspot your computer is connected to.

Once the Yun is connected to the same Wi-Fi hotspot, the JSON RPC server must be set up on the Braccio's Arduino Yun. The server code can be sent to the Arduino as follows:
```
roscd braccio_gaze_control/
scp braccio_gaze_server.py root@arduino3.local:~
```
This will copy the server code into root's home directory on the Arduino. To launch the RPC server, SSH into the Arduino, using **arduino** as the password:
```
ssh root@arduino3.local
```
Then launch the server:
```
python braccio_gaze_server.py
```
If this was successful, you should see a confirmation message stating that the server is running at a specified IP address and port.

Now you must connect a client to the server, in order to send commands from your computer to the running server:
```
cd ~/catkin_ws/
source devel/setup.bash
rosrun braccio_gaze_control braccio_gaze_controller.py
```

### Starting the ZED Camera
The ZED stereo camera must be started before any environment analysis can be performed:
```
cd ~/catkin_ws
roslaunch zed_wrapper zed.launch
```
If successful, this should report that data is being published on several ROS topics.

### Starting the TensorFlow Object Detection Service
*This is optional, as the environment analysis can be performed without object detection.*

To start the object detection service:
```
cd ~/catkin_ws
rosrun tf_object_detection object_detection_demo.py
```
If successful, a message stating that the service is ready for object detection will be displayed.

### Starting the Environment Analyser
To start the environment analyser:
```
cd ~/catkin_ws
rosrun environment_analysis rtabmap_ros_listener
```
A window displaying the camera feed and various debug-related data should be displayed, indicating that the environment analysis is currently active.
