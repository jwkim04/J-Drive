#include <Encoder/AS5047.hpp>

uint8_t dataTx[2] = { 0xFF, 0xFF };
uint8_t *dataRx;

int32_t incrementalCounter = 0;
uint8_t EncoderAPrev = 0;

void AS5047::UpdateEncoderPool()
{
	SPITransmitPool(dataTx, 2);
	dataRx = SPIReceive();

	SPITransmitPool(dataTx, 2);
	dataRx = SPIReceive();

	Update();
}

void AS5047::UpdateEncoder()
{
	SPITransmit(dataTx, 2);
	dataRx = SPIReceive();

	Update();
}

void AS5047::Update()
{
	encoderRawData = (((dataRx[0] & 0xFF) << 8) | (dataRx[1] & 0xFF)) & ~0xC000;

	jointPosition = (float) encoderRawData * ((float) M_PI * 2.0f) / 16383.0f;
	jointPosition -= encoderOffset;

	if (jointPosition > (float) M_PI * 2.0f)
	{
		jointPosition -= (float) M_PI * 2.0f;
	}
	else if (jointPosition < 0.0f)
	{
		jointPosition += (float) M_PI * 2.0f;
	}

	float rotorPositionRaw = jointPosition * (float) polePair;

	while (rotorPositionRaw > (float) M_PI * 2.0f)
	{
		rotorPositionRaw -= (float) M_PI * 2.0f;
	}

	rotorPosition = rotorPositionRaw;

	uint8_t EncoderA, EncoderB, ARising, AFalling;

	if (encoderRawData >= 0 && encoderRawData < (16383 / 8 * 2))
	{
		EncoderA = 1;
	}
	else
	{
		EncoderA = 0;
	}

	if (encoderRawData >= (16383 / 8 * 1) && encoderRawData < (16383 / 8 * 3))
	{
		EncoderB = 1;
	}
	else
	{
		EncoderB = 0;
	}

	if (EncoderAPrev == 0 && EncoderA == 1)
	{
		ARising = 1;
	}
	else
	{
		ARising = 0;
	}

	if (EncoderAPrev == 1 && EncoderA == 0)
	{
		AFalling = 1;
	}
	else
	{
		AFalling = 0;
	}

	if (ARising == 1 && EncoderB == 0)
	{
		incrementalCounter++;
	}
	else if (AFalling == 1 && EncoderB == 0)
	{
		incrementalCounter--;
	}

	EncoderAPrev = EncoderA;

	extendedJointPosition = (float) incrementalCounter * (float) (M_PI * 2.0) + (float) encoderRawData * ((float) (M_PI * 2.0) / 16383.0f);
	extendedJointPosition -= encoderOffset;

	jointVelocity = (extendedJointPosition - extendedJointPositionPrev[positionBufferIdx]) / (DELTA_T * 10.0f);
	jointVelocity = filter.Update(jointVelocity);
	extendedJointPositionPrev[positionBufferIdx++] = extendedJointPosition;

	if (positionBufferIdx >= 10)
	{
		positionBufferIdx = 0;
	}
}

uint16_t AS5047::GetRawData()
{
	return encoderRawData;
}

float AS5047::GetJointPosition()
{
	return jointPosition;
}

float AS5047::GetRotorPosition()
{
	return rotorPosition;
}

float AS5047::GetExtendedJointPosition()
{
	return extendedJointPosition;
}

float AS5047::GetJointVelocity()
{
	return jointVelocity;
}
