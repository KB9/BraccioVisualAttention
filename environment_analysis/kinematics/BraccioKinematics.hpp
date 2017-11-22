// #include <cmath>
// #include "MatrixMath.h"

static constexpr float PI = 3.141592;

// struct Lengths
// {
// 	float l1, l2, l3;
// };

// struct Pos2d
// {
// 	float x;
// 	float y;
// };

// struct Pos3d
// {
// 	float x, y, z;
// };

// struct JointAngles
// {
// 	float q1, q2, q3;
// };

// struct Angles
// {
// 	float base;
// 	float q1, q2, q3;
// };

// struct AngleDeltas
// {
// 	float q1, q2, q3;
// };

// struct PosDeltas
// {
// 	float dx, dy;
// };

// JointAngles &operator +=(JointAngles &angles, AngleDeltas deltas)
// {
// 	angles.q1 += deltas.q1;
// 	angles.q2 += deltas.q2;
// 	angles.q3 += deltas.q3;

// 	return angles;
// }

// JointAngles operator +(JointAngles angles, AngleDeltas deltas)
// {
// 	return angles += deltas;
// }

// JointAngles normalize(JointAngles angles)
// {
// 	auto norm = [](float theta)-> float {return theta - (2 * PI) * floor((theta + PI) / (2 * PI) ); };
// 	return {norm(angles.q1), norm(angles.q2), norm(angles.q3)};

// }

// Pos2d fk_pos(Lengths lengths, JointAngles angles)
// {
// 	float q1 = angles.q1;
// 	float q2 = angles.q2;
// 	float q3 = angles.q3;

// 	float l1 = lengths.l1;
// 	float l2 = lengths.l2;
// 	float l3 = lengths.l3;

// 	float x = l1 * cos(q1) + l2 * cos(q1 + q2) + l3 * cos(q1 + q2 + q3);
// 	float y = l1 * sin(q1) + l2 * sin(q1 + q2) + l3 * sin(q1 + q2 + q3);

// 	return {x, y};
// }

// AngleDeltas calculate(Lengths lengths, JointAngles angles, float *distance)
// {
// 	float q1 = angles.q1;
// 	float q2 = angles.q2;
// 	float q3 = angles.q3;

// 	float l1 = lengths.l1;
// 	float l2 = lengths.l2;
// 	float l3 = lengths.l3;

// 	float jacobian [2 * 3] = { -l1 * sinf(q1) - l2 * sinf(q1 + q2) - l3 * sinf(q1 + q2 + q3), -l2 * sinf(q1 + q2) - l3 * sinf(q1 + q2 + q3), -l3 * sinf(q1 + q2 + q3),
// 	                            l1 * cosf(q1) + l2 * cosf(q1 + q2) + l3 * cosf(q1 + q2 + q3),  l1 * cosf(q1 + q2) + l3 * cosf(q1 + q2 + q3),  l3 * cosf(q1 + q2 + q3) };                                                                             
// 	float jacobian_transpose[3 * 2];
// 	Matrix.Transpose(jacobian, 2, 3, jacobian_transpose);
	
// 	float mult[3 * 3];
// 	Matrix.Multiply(jacobian_transpose, jacobian, 3, 2, 3, mult);
// 	Matrix.Invert(mult, 3);

// 	float jacobian_pseudoinverse[3 * 2];
// 	Matrix.Multiply(mult, jacobian_transpose, 3, 3, 2, jacobian_pseudoinverse);

// 	float delta_angles[3 * 1];
// 	Matrix.Multiply(jacobian_pseudoinverse, distance, 3, 2, 1, delta_angles);
// 	return AngleDeltas {delta_angles[0], delta_angles[1], delta_angles[2]};
// }


// PosDeltas distance_delta(Lengths lengths, JointAngles angles, Pos2d tgt)
// {
// 	Pos2d curr_pos = fk_pos(lengths, angles);

// 	float dx = tgt.x - curr_pos.x;
// 	float dy = tgt.y - curr_pos.y;

// 	return {dx,dy};
// }

// struct Constraint
// {
// 	float min;
// 	float max;
// };


// struct JointConstraints
// {
// 	Constraint c1, c2, c3;
// };

// bool satisfies(Constraint c, float angle)
// {
// 	return angle >= c.min && angle <= c.max;
// }

// bool satisfies(JointConstraints constraints, JointAngles angles)
// {
// 	return satisfies(constraints.c1, angles.q1) && satisfies(constraints.c2, angles.q2) && satisfies(constraints.c3, angles.q3);
// }

// std::pair<JointAngles, bool> solve2d(Pos2d tgt, Lengths lengths, JointAngles start, float delta = 0.01f, int limit = 200)
// {
// 	float dist = std::hypot(tgt.x, tgt.y);
// 	float max_dist = lengths.l1 + lengths.l2 + lengths.l3;
// 	if (dist >= max_dist) {
// 		float base_angle = std::atan2(tgt.y, tgt.x);
// 		return {{base_angle, 0, 0}, true};
// 	}

// 	JointAngles angles = start;
// 	while (limit-- > 0)
// 	{
// 		PosDeltas pos_delta = distance_delta(lengths, angles, tgt);
// 		float distance[] = {pos_delta.dx, pos_delta.dy};

// 		auto deltas = calculate(lengths, angles, distance);

// 		angles += deltas;

// 		auto b1 = (fabs(distance[0]) + fabs(distance[1]));


// 		if (b1 < delta) { break;}
// 	}

// 	return { normalize(angles), limit > 0};
// }

// std::pair<JointAngles, bool> solve2d_constrained(Pos2d tgt, Lengths lengths, JointConstraints constraints, float delta = 0.01f, int limit = 200)
// {
// 	std::random_device rd;
// 	std::default_random_engine dre(rd());
// 	std::uniform_real_distribution<float> real_dist(-PI, PI);

// 	JointAngles result;
// 	do {
// 		--limit;
// 		JointAngles angles = {real_dist(dre), real_dist(dre), real_dist(dre) };
// 		auto res = solve2d(tgt, lengths, angles, delta, limit);
// 		if (!res.second) continue;
// 		result = res.first;

// 	} while(!satisfies(constraints, result) && limit-- > 0);

// 	return {result, limit > 0};
// }

// std::pair<Angles, bool> solve3d_constrained(Pos3d tgt, Lengths lengths, JointConstraints constraints, float delta = 0.01f, int limit = 200)
// {
// 	// Convert the 3d representation to a 2d one.
// 	float new_x = std::hypot(tgt.x, tgt.y);
// 	float new_y = tgt.z;
// 	Pos2d new_tgt = {new_x, new_y};

// 	float base_angle = atan2(tgt.y, tgt.x);

// 	auto solution_2d = solve2d_constrained(new_tgt, lengths, constraints, delta, limit);
// 	if (!solution_2d.second) {
// 		return { {}, false };
// 	}

// 	return {{base_angle, solution_2d.first.q1, solution_2d.first.q2, solution_2d.first.q3}, true};
// }

constexpr float toRadians(float degrees)
{
	return degrees * (PI / 180.0f);
}

constexpr float toDegrees(float radians)
{
	return radians * (180.0f / PI);
}

struct BraccioJointAngles
{
	float base;
	float shoulder;
	float elbow;
	float wrist;
	float wrist_rot;
};

class BraccioKinematics
{
public:
	bool lookAt(float x, float y, float z, BraccioJointAngles &braccio_angles);

	// NOTE: These are probably named wrong, and I can't find any segment on the arm which is 19cm long
	static constexpr float SHOULDER_LENGTH = 12.0f;
	static constexpr float ELBOW_LENGTH = 12.0f;
	static constexpr float WRIST_LENGTH = 19.0f;

	static constexpr float SHOULDER_CONSTRAINT_MIN = toRadians(15.0f);
	static constexpr float SHOULDER_CONSTRAINT_MAX = toRadians(165.0f);
	static constexpr float ELBOW_CONSTRAINT_MIN = toRadians(-90.0f); // 0.0f
	static constexpr float ELBOW_CONSTRAINT_MAX = toRadians(90.0f); // 180.0f
	static constexpr float WRIST_CONSTRAINT_MIN = toRadians(-90.0f); // 0.0f
	static constexpr float WRIST_CONSTRAINT_MAX = toRadians(90.0f); // 180.0f
	static constexpr float BASE_CONSTRAINT_MIN = toRadians(0.0f); // 0.0f
	static constexpr float BASE_CONSTRAINT_MAX = toRadians(180.0f); // 0.0f
};