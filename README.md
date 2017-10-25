# BraccioVisualAttention ![CI status](https://img.shields.io/badge/build-passing-brightgreen.svg)

BraccioVisualAttention 

## Installation



### Requirements
* Ubuntu 16.04 LTS
* ROS Kinetic
* libfreenect2
* IAI Kinect2

## Usage

Open a new terminal. Execute the following command to launch the bridge between libfreenect2 and ROS:
```
roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true
```

The above will eventually halt, stating that it is "waiting for clients to connect".
Once this occurs, open another terminal and execute the following command to launch the rtabmap_ros package:
```
roslaunch rtabmap_ros rgbd_mapping_kinect2.launch resolution:=qhd
```
rtabmap_ros will now be open, and will allow you to visualize the point cloud that is generated using the feed from the Kinect One.

To run the environment analysis package, open another terminal and execute the following command:
```
rosrun environment_analysis rtabmap_ros_listener
```
