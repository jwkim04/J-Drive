#ifndef AS5047_HPP_
#define AS5047_HPP_

#define _USE_MATH_DEFINES

#include <Lowlevel/lowlevel.hpp>
#include <math.h>

class AS5047 {
public:
	void UpdateEncoder();
	void UpdateEncoderPool();
	uint16_t GetRawData();
	float GetJointPosition();
	float GetRotorPosition();
	float GetExtendedJointPosition();
	float GetJointVelocity();

	uint32_t polePair;
	float encoderOffset = 0.0f;

private:
	void Update();

	uint16_t encoderRawData = 0;

	float jointPosition = 0.0f;
	float rotorPosition = 0.0f;
	float extendedJointPosition = 0.0f;
};

#endif
