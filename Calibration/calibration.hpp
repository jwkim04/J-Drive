#ifndef CALIBRATION_HPP_
#define CALIBRATION_HPP_

#define _USE_MATH_DEFINES
#include <math.h>
#include <Lowlevel/lowlevel.hpp>
#include <cstdint>
#include <Encoder/AS5047.hpp>
#include <FastMath/fast_math.hpp>

class Calibration {
public:
	void Init();
	void CalibrationUpdate();

	float calibrationVoltage = 0.05f;
	uint8_t done = 0;
	float encoderOffset = 0.0f;
	uint32_t polePair = 1;

private:
	AS5047 Encoder = AS5047();

	float theta = 0.0f;
	uint32_t startUpCounter = 0;

	void DQZTrans(float a, float b, float c, float theta, float *d, float *q);
	void DQZTransInv(float d, float q, float theta, float *a, float *b, float *c);
};


#endif
