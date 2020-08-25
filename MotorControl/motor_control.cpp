#include <MotorControl/motor_control.hpp>
#include <BoardConfig/board_config.h>

float theta = 0.0f;

float a, b, c;

float d = 0.05f;
float q = 0.0f;

uint16_t aPWM, bPWM, cPWM;

float adc1, adc2;

void MotorControl::ControlUpdate()
{
	adc1 = (float) ((int32_t) GetSO1() - 0x7FF);
	adc2 = (float) ((int32_t) GetSO2() - 0x7FF);

	DQZTransInv(d, q, theta, &a, &b, &c);

	a = a + .5f;
	b = b + .5f;
	c = c + .5f;

	aPWM = (uint16_t) (a * ((float) (0xFFF)));
	bPWM = (uint16_t) (b * ((float) (0xFFF)));
	cPWM = (uint16_t) (c * ((float) (0xFFF)));

	SetInverterPWMDuty(aPWM, bPWM, cPWM);

	theta += 0.002f;
	if (theta > 2 * M_PI)
		theta = 0.0f;
}

void MotorControl::DQZTrans(float a, float b, float c, float theta, float *d, float *q)
{
	float cf = FastCos(theta);
	float sf = FastSin(theta);

	*d = 0.6666667f * (cf * a + (0.86602540378f * sf - .5f * cf) * b + (-0.86602540378f * sf - .5f * cf) * c);
	*q = 0.6666667f * (-sf * a - (-0.86602540378f * cf - .5f * sf) * b - (0.86602540378f * cf - .5f * sf) * c);
}

void MotorControl::DQZTransInv(float d, float q, float theta, float *a, float *b, float *c)
{
	float cf = FastCos(theta);
	float sf = FastSin(theta);

	*a = cf * d - sf * q;
	*b = (0.86602540378f * sf - .5f * cf) * d - (-0.86602540378f * cf - .5f * sf) * q;
	*c = (-0.86602540378f * sf - .5f * cf) * d - (0.86602540378f * cf - .5f * sf) * q;
}
