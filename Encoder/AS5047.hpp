#ifndef AS5047_HPP_
#define AS5047_HPP_

#define _USE_MATH_DEFINES

#include <Lowlevel/lowlevel.hpp>
#include <math.h>
#include <BoardConfig/board_config.h>
#include <DigitalFilter/lowpass.hpp>

class AS5047
{
public:
	LowPass filter_vel = LowPass();
	LowPass filter_pos = LowPass();

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

	float jointVelocity = 0.0f;
	float jointPosition = 0.0f;
	float rotorPosition = 0.0f;
	float extendedJointPosition = 0.0f;
	float extendedJointPositionPrev[10] = { 0, };
	uint8_t positionBufferIdx = 0;
};

#endif
