#include "foc.hpp"

void FOC::FOCUpdate()
{

}

void FOC::DQZTrans(float a, float b, float c, float theta, float *d, float *q)
{
	float cf = FastCos(theta);
	float sf = FastSin(theta);

	*d = 0.6666667f * (cf * a + (0.86602540378f * sf - .5f * cf) * b + (-0.86602540378f * sf - .5f * cf) * c);
	*q = 0.6666667f * (-sf * a - (-0.86602540378f * cf - .5f * sf) * b - (0.86602540378f * cf - .5f * sf) * c);
}

void FOC::DQZTransInv(float d, float q, float theta, float *a, float *b, float *c)
{
	float cf = FastCos(theta);
	float sf = FastSin(theta);

	*a = cf * d - sf * q;
	*b = (0.86602540378f * sf - .5f * cf) * d - (-0.86602540378f * cf - .5f * sf) * q;
	*c = (-0.86602540378f * sf - .5f * cf) * d - (0.86602540378f * cf - .5f * sf) * q;
}
