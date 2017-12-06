#include "Braccio.hpp"

Braccio::Braccio(float base_angle, float shoulder_angle, float elbow_angle,
                 float wrist_angle, float wrist_rot_angle)
{
	joint_angles.base = base_angle;
	joint_angles.shoulder = shoulder_angle;
	joint_angles.elbow = elbow_angle;
	joint_angles.wrist = wrist_angle;
	joint_angles.wrist_rot = wrist_rot_angle;

	is_gaze_feedback_started = false;
}

void Braccio::initGazeFeedback(ros::NodeHandle &nh,
                               void (*subscriberCallback)(std_msgs::Bool))
{
	if (!is_gaze_feedback_started)
	{
		gaze_point_publisher = nh.advertise<std_msgs::Int32MultiArray>("braccio_gaze_focus_setter", 1);
		gaze_point_subscriber = nh.subscribe("/braccio_gaze_focus_callback", 1, subscriberCallback);
		ROS_INFO("Waiting for connection to gaze server...");
		while (gaze_point_publisher.getNumSubscribers() == 0) {}
		ROS_INFO("Connection to gaze server established!");
		is_gaze_feedback_started = true;

		// sendJointAngles();
	}
}

bool Braccio::lookAt(float x, float y, float z)
{
	BraccioJointAngles new_angles;
	bool ok = kinematics.lookAt(x, y, z, new_angles);
	if (ok)
		setJointAngles(new_angles);
	else
		ROS_WARN("IK could not solve for (%f,%f,%f)", x, y, z);
	return ok;
}

bool Braccio::lookAt_BR(float x, float y, float z)
{
	ROS_INFO("Camera relative: (%.3f,%.3f,%.3f)", x, y, z);
	Pos3d base_relative_pos = kinematics.toBaseRelative(x, y, z);
	ROS_INFO("Base relative: (%.3f,%.3f,%.3f)", base_relative_pos.x, base_relative_pos.y, base_relative_pos.z);

	// TODO: Make this move the the Braccio
	lookAt(base_relative_pos.x, base_relative_pos.y, base_relative_pos.z);
}

float Braccio::getEffectorX()
{
	return kinematics.getEffectorPos3d().x;
}

float Braccio::getEffectorY()
{
	return kinematics.getEffectorPos3d().y;
}

float Braccio::getEffectorZ()
{
	return kinematics.getEffectorPos3d().z;
}

void Braccio::setJointAngles(const BraccioJointAngles &angles)
{
	joint_angles = angles;
	kinematics.setJointAngles(angles);
	sendJointAngles();
}

BraccioJointAngles Braccio::getJointAngles()
{
	return joint_angles;
}

bool Braccio::isFeedbackInitialized()
{
	return is_gaze_feedback_started;
}

void Braccio::sendJointAngles()
{
	if (!isFeedbackInitialized())
	{
		ROS_WARN("Gaze feedback has not been initialized. Braccio pose will not be updated.");
		return;
	}

	if (gaze_point_publisher.getNumSubscribers() > 0)
	{
		std_msgs::Int32MultiArray angles_msg;
		angles_msg.data.push_back(joint_angles.base);
		angles_msg.data.push_back(joint_angles.shoulder);
		angles_msg.data.push_back(joint_angles.elbow);
		angles_msg.data.push_back(joint_angles.wrist);
		angles_msg.data.push_back(joint_angles.wrist_rot);

		gaze_point_publisher.publish(angles_msg);
	}
	else
	{
		ROS_WARN("Gaze server not active. Braccio pose will not be updated.");
	}
}