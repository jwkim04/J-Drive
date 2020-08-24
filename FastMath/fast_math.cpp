#include "fast_math.hpp"
#include <cstdint>

float sineTable[FAST_MATH_TABLE_SIZE];

void FastMathInit()
{
	double x = 0.0f;

	for (uint32_t i = 0; i < FAST_MATH_TABLE_SIZE; i++)
	{
		sineTable[i] = (float) sin(x);

		x += (M_PI * 2.0) / (double) FAST_MATH_TABLE_SIZE;
	}
}

float FastSin(float x)
{
	x -= (float) (M_PI * 2.0) * (float) ((int32_t) (x / (float) (M_PI * 2.0)));

	uint32_t idx = (uint32_t) (x * ((double) FAST_MATH_TABLE_SIZE / (M_PI * 2.0)));

	return sineTable[idx];
}

float FastCos(float x)
{
	x += (float) M_PI / 2.0f;
	x -= (float) (M_PI * 2.0) * (float) ((int32_t) (x / (float) (M_PI * 2.0)));

	uint32_t idx = (uint32_t) (x * ((double) FAST_MATH_TABLE_SIZE / (M_PI * 2.0)));

	return sineTable[idx];
}
