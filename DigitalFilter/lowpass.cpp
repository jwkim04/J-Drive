#include <DigitalFilter/lowpass.hpp>

void LowPass::SetParam(float cutOffFreq, float delta)
{
	e = 1.0f - exp(-delta * 2.0f * (float) M_PI * cutOffFreq);
}

float LowPass::Update(float input)
{
	output += (input - output) * e;

	return output;
}
