#include "BraccioKinematics.hpp"
#include <cmath>
#include <random>
#include "MatrixMath.h"

JointAngles &operator +=(JointAngles &angles, AngleDeltas deltas)
{
	angles.q1 += deltas.q1;
	angles.q2 += deltas.q2;
	angles.q3 += deltas.q3;

	return angles;
}

JointAngles operator +(JointAngles angles, AngleDeltas deltas)
{
	return angles += deltas;
}

JointAngles normalize(JointAngles angles)
{
	auto norm = [](float theta)-> float {return theta - (2 * PI) * floor((theta + PI) / (2 * PI) ); };
	return {norm(angles.q1), norm(angles.q2), norm(angles.q3)};

}

Pos2d fk_pos(Lengths lengths, JointAngles angles)
{
	float q1 = angles.q1;
	float q2 = angles.q2;
	float q3 = angles.q3;

	float l1 = lengths.l1;
	float l2 = lengths.l2;
	float l3 = lengths.l3;

	float x = l1 * cos(q1) + l2 * cos(q1 + q2) + l3 * cos(q1 + q2 + q3);
	float y = l1 * sin(q1) + l2 * sin(q1 + q2) + l3 * sin(q1 + q2 + q3);

	return {x, y};
}

AngleDeltas calculate(Lengths lengths, JointAngles angles, float *distance)
{
	float q1 = angles.q1;
	float q2 = angles.q2;
	float q3 = angles.q3;

	float l1 = lengths.l1;
	float l2 = lengths.l2;
	float l3 = lengths.l3;

	float jacobian [2 * 3] = { -l1 * sinf(q1) - l2 * sinf(q1 + q2) - l3 * sinf(q1 + q2 + q3), -l2 * sinf(q1 + q2) - l3 * sinf(q1 + q2 + q3), -l3 * sinf(q1 + q2 + q3),
	                            l1 * cosf(q1) + l2 * cosf(q1 + q2) + l3 * cosf(q1 + q2 + q3),  l1 * cosf(q1 + q2) + l3 * cosf(q1 + q2 + q3),  l3 * cosf(q1 + q2 + q3) };                                                                             
	float jacobian_transpose[3 * 2];
	Matrix.Transpose(jacobian, 2, 3, jacobian_transpose);
	
	float mult[3 * 3];
	Matrix.Multiply(jacobian_transpose, jacobian, 3, 2, 3, mult);
	Matrix.Invert(mult, 3);

	float jacobian_pseudoinverse[3 * 2];
	Matrix.Multiply(mult, jacobian_transpose, 3, 3, 2, jacobian_pseudoinverse);

	float delta_angles[3 * 1];
	Matrix.Multiply(jacobian_pseudoinverse, distance, 3, 2, 1, delta_angles);
	return AngleDeltas {delta_angles[0], delta_angles[1], delta_angles[2]};
}


PosDeltas distance_delta(Lengths lengths, JointAngles angles, Pos2d tgt)
{
	Pos2d curr_pos = fk_pos(lengths, angles);

	float dx = tgt.x - curr_pos.x;
	float dy = tgt.y - curr_pos.y;

	return {dx,dy};
}

struct Constraint
{
	float min;
	float max;
};


struct JointConstraints
{
	Constraint c1, c2, c3;
};

bool satisfies(Constraint c, float angle)
{
	return angle >= c.min && angle <= c.max;
}

bool satisfies(JointConstraints constraints, JointAngles angles)
{
	return satisfies(constraints.c1, angles.q1) && satisfies(constraints.c2, angles.q2) && satisfies(constraints.c3, angles.q3);
}

std::pair<JointAngles, bool> solve2d(Pos2d tgt, Lengths lengths, JointAngles start, float delta = 0.01f, int limit = 200)
{
	float dist = std::hypot(tgt.x, tgt.y);
	float max_dist = lengths.l1 + lengths.l2 + lengths.l3;
	if (dist >= max_dist) {
		float base_angle = std::atan2(tgt.y, tgt.x);
		return {{base_angle, 0, 0}, true};
	}

	JointAngles angles = start;
	while (limit-- > 0)
	{
		PosDeltas pos_delta = distance_delta(lengths, angles, tgt);
		float distance[] = {pos_delta.dx, pos_delta.dy};

		auto deltas = calculate(lengths, angles, distance);

		angles += deltas;

		auto b1 = (fabs(distance[0]) + fabs(distance[1]));


		if (b1 < delta) { break;}
	}

	return { normalize(angles), limit > 0};
}

std::pair<JointAngles, bool> solve2d_constrained(Pos2d tgt, Lengths lengths, JointConstraints constraints, float delta = 0.01f, int limit = 200)
{
	std::random_device rd;
	std::default_random_engine dre(rd());
	std::uniform_real_distribution<float> real_dist(-PI, PI);

	JointAngles result;
	do {
		--limit;
		JointAngles angles = {real_dist(dre), real_dist(dre), real_dist(dre) };
		auto res = solve2d(tgt, lengths, angles, delta, limit);
		if (!res.second) continue;
		result = res.first;

	} while(!satisfies(constraints, result) && limit-- > 0);	

	return {result, limit > 0};
}

std::pair<Angles, bool> solve3d_constrained(Pos3d tgt, Lengths lengths, JointConstraints constraints, Constraint base_constraint = {0, 2 * PI}, float delta = 0.01f, int limit = 200)
{
	std::pair<Angles, bool> empty = { {}, false };

	auto norm = [](float theta)-> float {return theta - (2 * PI) * floor((theta + PI) / (2 * PI) ); };

	// Convert the 3d representation to a 2d one.
	float base_angle = atan2(tgt.y, tgt.x);

	if (base_angle < 0.0f) base_angle = 2 * PI + base_angle;

	if (!satisfies(base_constraint, base_angle)) {
		return empty;
	}
	float new_x = std::hypot(tgt.x, tgt.y);
	float new_y = tgt.z;
	Pos2d new_tgt = {new_x, new_y};


	auto solution_2d = solve2d_constrained(new_tgt, lengths, constraints, delta, limit);
	if (!solution_2d.second) {
		return empty;
	}

	return {{base_angle, solution_2d.first.q1, solution_2d.first.q2, solution_2d.first.q3}, true};
}

bool BraccioKinematics::lookAt(float x, float y, float z, BraccioJointAngles &braccio_angles)
{
	Lengths lengths{SHOULDER_LENGTH, ELBOW_LENGTH, WRIST_LENGTH};
	JointConstraints constraints{{SHOULDER_CONSTRAINT_MIN, SHOULDER_CONSTRAINT_MAX},
								 {ELBOW_CONSTRAINT_MIN, ELBOW_CONSTRAINT_MAX},
								 {WRIST_CONSTRAINT_MIN, WRIST_CONSTRAINT_MAX}};
	Pos3d target{x, y, z};

	auto result = solve3d_constrained(target, lengths, constraints, {0, PI});
	bool success = result.second;
	if (success)
	{
		Angles angles = result.first;
		braccio_angles.base = toDegrees(angles.base);
		braccio_angles.shoulder = toDegrees(angles.q1);
		braccio_angles.elbow = toDegrees(angles.q2) + 90.0f;
		braccio_angles.wrist = toDegrees(angles.q3) + 90.0f;
		braccio_angles.wrist_rot = 90.0f;

		current_angles = braccio_angles;
	}

	return success;
}

Pos3d BraccioKinematics::getEffectorPos3d()
{
	JointAngles angles;
	angles.q1 = toRadians(current_angles.shoulder);
	angles.q2 = toRadians(current_angles.elbow - 90.0f);
	angles.q3 = toRadians(current_angles.wrist - 90.0f);

	Lengths lengths;
	lengths.l1 = SHOULDER_LENGTH;
	lengths.l2 = ELBOW_LENGTH;
	lengths.l3 = WRIST_LENGTH;

	Pos2d fk_pos = ::fk_pos(lengths, angles);
	return {fk_pos.x * cosf(toRadians(current_angles.base)), fk_pos.x * sinf(toRadians(current_angles.base)), fk_pos.y};
}

Pos3d BraccioKinematics::toBaseRelativeAxis(float x, float y, float z)
{
	float a = toRadians(current_angles.base);
	float base_rel_x = x * cosf(a) + z * -sinf(a);
	float base_rel_y = y;
	float base_rel_z = x * sinf(a) + z * cosf(a);

	return {base_rel_x, base_rel_y, base_rel_z};
}

Pos3d BraccioKinematics::toBaseRelative(float x, float y, float z)
{
	// Get the position of the effector relative to the arm's base
	Pos3d effector_pos = getEffectorPos3d();

	// Rotate the camera's axes to that of the base
	Pos3d base_rel_pos = toBaseRelativeAxis(x, y, z);

	// Translate the base relative positions to the base origin
	return {base_rel_pos.x + effector_pos.x, base_rel_pos.y + effector_pos.y, base_rel_pos.z + effector_pos.z};
}