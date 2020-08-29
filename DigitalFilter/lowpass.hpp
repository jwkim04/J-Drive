#ifndef LOWPASS_HPP_
#define LOWPASS_HPP_

#include <math.h>

class LowPass
{
public:
	void SetParam(float cutOffFreq, float delta);
	float Update(float input);

private:
	float e;
	float output = 0.0f;
};

#endif
