#ifndef FOC_HPP_
#define FOC_HPP_

#include <FastMath/fast_math.hpp>

class FOC
{
public:
	float iq = 0.0f;
	float id = 0.0f;

	float kp = 0.0f;
	float ki = 0.0f;
	float iLimit = 0.0f;

	void FOCUpdate();

private:
	void DQZTrans(float a, float b, float c, float theta, float *d, float *q);
	void DQZTransInv(float d, float q, float theta, float *a, float *b, float *c);
};

#endif
