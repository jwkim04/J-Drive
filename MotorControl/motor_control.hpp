#ifndef MOTOR_CONTROL_HPP_
#define MOTOR_CONTROL_HPP_

#include <Lowlevel/lowlevel.hpp>
#include <FastMath/fast_math.hpp>
#include <cstdint>
#include <Encoder/AS5047.hpp>
#include <Util/util.hpp>
#include <BoardConfig/board_config.h>
#include <DigitalFilter/lowpass.hpp>
#include <Protocol/controlTable.hpp>

enum MotorControlMode
{
	VOLTAGE_CONTROL_MODE,
	CURRENT_CONTROL_MODE,
	VELOCITY_CONTROL_MODE,
	POSITION_CONTROL_MODE,
	DAMPED_OSCILLATION_POSITION_CONTROL_MODE
};

struct ControlParam
{
	uint8_t controlMode;

	float goalVoltage;

	float goalCurrent;

	float goalVelocity;

	float goalPosition;
};

struct DampedOscillationParam
{
	float k;
	float x;
	float b;
	float xdot;
	float m;
	float _xddotBuff[10] = {0, };
	uint8_t _buffIdx = 0;
	float xddot;
	float F;
};

struct PIDParam
{
	float error;
	float p;
	float i;
	float d;
	float a;
	float Kp;
	float Ki;
	float Kd;
	float Ka;
};

struct MotorParam
{
	uint32_t polePair;
	float encoderOffset = 0.0f;
};

class MotorControl
{
public:
	void Init();
	void SetControlTable(ControlTable *_controlTable);
	void ControlUpdate();

	AS5047 Encoder = AS5047();

	LowPass filter_d = LowPass();
	LowPass filter_q = LowPass();
	LowPass filter_acceleration = LowPass();

	uint8_t controlMode = VOLTAGE_CONTROL_MODE;

	ControlParam controlParam;

	DampedOscillationParam dampedOscillationParam;
	PIDParam currentPIDParam_d;
	PIDParam currentPIDParam_q;
	PIDParam velocityPIDParam;
	PIDParam positionPIDParam;
	MotorParam motorParam;

	float supplyVoltage;

	int32_t ADC1Offset = 2047;
	int32_t ADC2Offset = 2047;

	float presentCurrent;
	float presentVoltage;

	float jointPosition;
	float rotorPosition;
	float extendedJointPosition;
	float jointVelocity;

	float velocityCommand;
	float currentCommand;
	float ia, ib, ic;
	float id, iq;
	float vd;
	float vq;
	float va, vb, vc;
	uint16_t aPWM, bPWM, cPWM;

private:
	ControlTable *controlTable;

	void DampedOscillationPositionControl();
	void PositionControl();
	void VelocityControl();
	void CurrentControl();
	void VoltageControl();

	void DQZTrans(float a, float b, float c, float theta, float *d, float *q);
	void DQZTransInv(float d, float q, float theta, float *a, float *b, float *c);
};

#endif
