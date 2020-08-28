#include <MotorControl/motor_control.hpp>
#include <BoardConfig/board_config.h>
#include <stdio.h>

float theta = 0.0f;

float a, b, c;

float d = 0.02f;
float q = 0.0f;

uint16_t aPWM, bPWM, cPWM;

float jointPosition;
float rotorPosition;
float extendedJointPosition;

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

	ia = (float) ((int32_t) GetSO1() - ADC1Offset) * (1.0f / (float) 0x7FF);
	ib = (float) ((int32_t) GetSO2() - ADC2Offset) * (1.0f / (float) 0x7FF);
	ic = (-ia - ib);

	DQZTrans(ic, ib, ia, theta, &id, &iq);
	DQZTransInv(d, q, theta, &a, &b, &c);

	a = a + 0.5f;
	b = b + 0.5f;
	c = c + 0.5f;

	aPWM = (uint16_t) (a * ((float) (0xFFF)));
	bPWM = (uint16_t) (b * ((float) (0xFFF)));
	cPWM = (uint16_t) (c * ((float) (0xFFF)));

	SetInverterPWMDuty(aPWM, bPWM, cPWM);

	theta += 0.0003f;
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
