#include <Calibration/calibration.hpp>

void Calibration::Init()
{
	done = 0;
	startUpCounter = 0;
	avgCounter = 0;
}

void Calibration::CalibrationUpdate()
{
	if (!done)
	{
		float a, b, c;

		DQZTransInv(calibrationVoltage, 0.0f, 0.0f, &a, &b, &c);

		a = a + 0.5f;
		b = b + 0.5f;
		c = c + 0.5f;

		uint16_t aDuty = (uint16_t) (a * ((float) (0xFFF)));
		uint16_t bDuty = (uint16_t) (b * ((float) (0xFFF)));
		uint16_t cDuty = (uint16_t) (c * ((float) (0xFFF)));

		SetInverterPWMDuty(aDuty, bDuty, cDuty);

		if (startUpCounter == 10000)
		{
			Encoder.UpdateEncoderPool();
			encoderOffset += Encoder.GetJointPosition();
			avgCounter++;

			if (avgCounter == 100)
			{
				done = 1;
				encoderOffset /= 100.0f;
				SetInverterPWMDuty(0x0, 0x0, 0x0);
				return;
			}
		}
		else
		{
			startUpCounter++;
		}
	}

}

void Calibration::DQZTrans(float a, float b, float c, float theta, float *d, float *q)
{
	float cf = FastCos(theta);
	float sf = FastSin(theta);

	*d = 0.6666667f * (cf * a + (0.86602540378f * sf - 0.5f * cf) * b + (-0.86602540378f * sf - 0.5f * cf) * c);
	*q = 0.6666667f * (-sf * a - (-0.86602540378f * cf - 0.5f * sf) * b - (0.86602540378f * cf - 0.5f * sf) * c);
}

void Calibration::DQZTransInv(float d, float q, float theta, float *a, float *b, float *c)
{
	float cf = FastCos(theta);
	float sf = FastSin(theta);

	*a = cf * d - sf * q;
	*b = (0.86602540378f * sf - 0.5f * cf) * d - (-0.86602540378f * cf - 0.5f * sf) * q;
	*c = (-0.86602540378f * sf - 0.5f * cf) * d - (0.86602540378f * cf - 0.5f * sf) * q;
}
