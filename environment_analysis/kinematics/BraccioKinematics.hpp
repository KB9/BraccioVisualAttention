static constexpr float PI = 3.141592;

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

struct Lengths
{
	float l1, l2, l3;
};

struct Pos2d
{
	float x;
	float y;
};

struct Pos3d
{
	float x, y, z;
};

struct JointAngles
{
	float q1, q2, q3;
};

struct Angles
{
	float base;
	float q1, q2, q3;
};

struct AngleDeltas
{
	float q1, q2, q3;
};

struct PosDeltas
{
	float dx, dy;
};

struct Constraint
{
	float min;
	float max;
};

struct JointConstraints
{
	Constraint c1, c2, c3;
};

class BraccioKinematics
{
public:
	bool lookAt(float x, float y, float z, BraccioJointAngles &braccio_angles);
	Pos3d getEffectorPos3d();

	void setJointAngles(const BraccioJointAngles &angles);

	static constexpr float SHOULDER_LENGTH = 12.0f;
	static constexpr float ELBOW_LENGTH = 12.0f;
	static constexpr float WRIST_LENGTH = 9.0f;

	static constexpr float SHOULDER_CONSTRAINT_MIN = toRadians(15.0f);
	static constexpr float SHOULDER_CONSTRAINT_MAX = toRadians(165.0f);
	static constexpr float ELBOW_CONSTRAINT_MIN = toRadians(-90.0f); // 0.0f
	static constexpr float ELBOW_CONSTRAINT_MAX = toRadians(90.0f); // 180.0f
	static constexpr float WRIST_CONSTRAINT_MIN = toRadians(-90.0f); // 0.0f
	static constexpr float WRIST_CONSTRAINT_MAX = toRadians(90.0f); // 180.0f
	static constexpr float BASE_CONSTRAINT_MIN = toRadians(0.0f); // 0.0f
	static constexpr float BASE_CONSTRAINT_MAX = toRadians(180.0f); // 0.0f

private:
	Angles current_angles; // in radians
};