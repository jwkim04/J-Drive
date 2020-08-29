#include <Util/util.hpp>

void Delaymillis(uint32_t ms)
{
	HAL_Delay(ms);
}

float Limiter(float value, float limit)
{
	if (value > limit)
	{
		return limit;
	}
	else if (value < -limit)
	{
		return -limit;
	}

	return value;
}
