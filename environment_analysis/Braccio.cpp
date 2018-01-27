#include "Braccio.hpp"

// Eigen
#include <Eigen/Core>

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

bool Braccio::lookAt(float x, float y, float z, ReferenceFrame frame)
{
	BraccioJointAngles new_angles;

	// Convert to the world reference frame if necessary
	Pos3d world_coords;
	if (frame == ReferenceFrame::Effector)
	{
		world_coords = toBaseRelative(x, y, z);
	}
	else
	{
		world_coords.x = x;
		world_coords.y = y;
		world_coords.z = z;
	}

	// Send the joint angles to the Braccio if they are accepted by the IK
	bool ok = kinematics.lookAt(world_coords.x, world_coords.y, world_coords.z,
	                            new_angles);
	if (ok)
		setJointAngles(new_angles);
	else
		ROS_WARN("IK could not solve for (%.3f,%.3f,%.3f)",
		         world_coords.x, world_coords.y, world_coords.z);
	return ok;
}

Pos3d Braccio::toBaseRelative(float x, float y, float z)
{
	// Get the angle of the base and effector in radians
	BraccioJointAngles angles = getJointAngles();
	float angle_eff_deg = angles.shoulder + (angles.elbow - 90) + (angles.wrist - 90);
	float angle_eff = toRadians(angle_eff_deg);
	float angle_base = toRadians(angles.base);

	// Initialize the two angles of rotation that are possible for the Braccio
	// effector
	float effector_y_angle = angle_eff;
	float effector_z_angle = -angle_base;

	Eigen::MatrixXf y_rotate(3,3);
	y_rotate(0,0) = cosf(effector_y_angle);
	y_rotate(0,1) = 0;
	y_rotate(0,2) = -sinf(effector_y_angle);
	y_rotate(1,0) = 0;
	y_rotate(1,1) = 1;
	y_rotate(1,2) = 0;
	y_rotate(2,0) = sinf(effector_y_angle);
	y_rotate(2,1) = 0;
	y_rotate(2,2) = cosf(effector_y_angle);

	Eigen::MatrixXf z_rotate(3,3);
	z_rotate(0,0) = cosf(effector_z_angle);
	z_rotate(0,1) = sinf(effector_z_angle);
	z_rotate(0,2) = 0;
	z_rotate(1,0) = -sinf(effector_z_angle);
	z_rotate(1,1) = cosf(effector_z_angle);
	z_rotate(1,2) = 0;
	z_rotate(2,0) = 0;
	z_rotate(2,1) = 0;
	z_rotate(2,2) = 1;

	Eigen::MatrixXf pos(3,1);
	pos(0,0) = x;
	pos(1,0) = y;
	pos(2,0) = z;

	// First rotate around Y then around Z, then add the effector positions to
	// make the base center the origin
	Eigen::MatrixXf result = z_rotate * (y_rotate * pos);
	float rx = result(0,0) + getEffectorX();
	float ry = result(1,0) + getEffectorY();
	float rz = result(2,0) + getEffectorZ();
	return {rx, ry, rz};
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