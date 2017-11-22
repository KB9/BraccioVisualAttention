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