#include <MotorControl/motor_control.hpp>
#include <BoardConfig/board_config.h>

float theta = 0.0f;

float a, b, c;

float d = 0.015f;
float q = 0.0f;

uint16_t aPWM, bPWM, cPWM;

float adc1, adc2;

float jointPosition;
float rotorPosition;
float extendedJointPosition;

float error = 0.0f;

void MotorControl::Init()
{
	Encoder.encoderOffset = motorParam.encoderOffset;
	Encoder.polePair = motorParam.polePair;

	Encoder.UpdateEncoderPool();
}

void MotorControl::ControlUpdate()
{
	Encoder.UpdateEncoder();

	jointPosition = Encoder.GetJointPosition();
	rotorPosition = Encoder.GetRotorPosition();
	extendedJointPosition = Encoder.GetExtendedJointPosition();

	adc1 = (float) ((int32_t) GetSO1() - 0x7FF);
	adc2 = (float) ((int32_t) GetSO2() - 0x7FF);

	error = theta - rotorPosition;
	DQZTransInv(d, q, rotorPosition + (M_PI / 2.0f), &a, &b, &c);

	a = a + 0.5f;
	b = b + 0.5f;
	c = c + 0.5f;

	aPWM = (uint16_t) (a * ((float) (0xFFF)));
	bPWM = (uint16_t) (b * ((float) (0xFFF)));
	cPWM = (uint16_t) (c * ((float) (0xFFF)));

	SetInverterPWMDuty(aPWM, bPWM, cPWM);

	theta += 0.0001f;
	if (theta > 2 * M_PI)
		theta = 0.0f;
}

void MotorControl::DQZTrans(float a, float b, float c, float theta, float *d, float *q)
{
	float cf = FastCos(theta);
	float sf = FastSin(theta);

	*d = 0.6666667f * (cf * a + (0.86602540378f * sf - 0.5f * cf) * b + (-0.86602540378f * sf - 0.5f * cf) * c);
	*q = 0.6666667f * (-sf * a - (-0.86602540378f * cf - 0.5f * sf) * b - (0.86602540378f * cf - 0.5f * sf) * c);
}

void MotorControl::DQZTransInv(float d, float q, float theta, float *a, float *b, float *c)
{
	float cf = FastCos(theta);
	float sf = FastSin(theta);

	*a = cf * d - sf * q;
	*b = (0.86602540378f * sf - 0.5f * cf) * d - (-0.86602540378f * cf - 0.5f * sf) * q;
	*c = (-0.86602540378f * sf - 0.5f * cf) * d - (0.86602540378f * cf - 0.5f * sf) * q;
}
