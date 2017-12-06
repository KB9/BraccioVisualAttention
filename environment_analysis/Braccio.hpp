#ifndef _BRACCIO_H_
#define _BRACCIO_H_

// ROS
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Bool.h"

// Kinematics
#include "kinematics/BraccioKinematics.hpp"

class Braccio
{
public:
	Braccio(float base_angle = 0.0f, float shoulder_angle = 90.0f,
	        float elbow_angle = 90.0f, float wrist_angle = 90.0f,
	        float wrist_rot_angle = 90.0f);

	void initGazeFeedback(ros::NodeHandle &nh,
	                      void (*subscriberCallback)(std_msgs::Bool));

	bool lookAt(float x, float y, float z);
	bool lookAt_BR(float x, float y, float z);

	// TESTING
	float getEffectorX();
	float getEffectorY();
	float getEffectorZ();

	void setJointAngles(const BraccioJointAngles &angles);
	BraccioJointAngles getJointAngles();

	bool isFeedbackInitialized();

private:
	BraccioKinematics kinematics;
	BraccioJointAngles joint_angles;

	ros::Publisher gaze_point_publisher;
	ros::Subscriber gaze_point_subscriber;
	bool is_gaze_feedback_started;

	void sendJointAngles();
};

#endif // _BRACCIO_H_