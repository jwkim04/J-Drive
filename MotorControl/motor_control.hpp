#ifndef MOTOR_CONTROL_HPP_
#define MOTOR_CONTROL_HPP_

#include <Lowlevel/lowlevel.hpp>
#include <FastMath/fast_math.hpp>
#include <cstdint>
#include <Encoder/AS5047.hpp>

enum MotorControlMode
{
	VOLTAGE_CONTROL_MODE,
	CURRENT_CONTROL_MODE,
	VELOCITY_CONTROL_MODE,
	POSITION_CONTROL_MODE
//DAMPED_OSCILLATION_POSITION_CONTROL_MODE
};

struct ControlParam
{
	uint8_t controlMode;

	float volatgeLimit;
	float goalVoltage;

	float currentLimit;
	float goalCurrent;

	float velocityLimit;
	float goalVelocity;

	float goalPosition;
};

struct PIDParam
{
	float Kp;
	float Ki;
	float Kd;
	float Ka;
	float iLimit;
};

struct MotorParam
{
	uint32_t polePair;
	float encoderOffset;
	float phaseResistance;
	float phaseInductance;
};

class MotorControl
{
public:
	void Init();
	void ControlUpdate();

	ControlParam controlParam;

	PIDParam currentPIDParam;
	PIDParam velocityPIDParam;
	PIDParam positionPIDParam;
	MotorParam motorParam;

	float supplyVoltage;

	int32_t ADC1Offset;
	int32_t ADC2Offset;


	float ia, ib, ic;
	float id, iq;

private:
	AS5047 Encoder = AS5047();

	void DQZTrans(float a, float b, float c, float theta, float *d, float *q);
	void DQZTransInv(float d, float q, float theta, float *a, float *b, float *c);
};

#endif
