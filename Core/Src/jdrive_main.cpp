#include <jdrive_main.hpp>
#include <Lowlevel/lowlevel.hpp>
#include <Util/util.hpp>
#include <cstdint>
#include <FastMath/fast_math.hpp>

void JDriveMain()
{
	StartOnBoardLED();
	for (uint8_t k = 0; k < 2; k++)
	{
		for (int16_t i = 0x0; i <= 0xFFF; i = i + 45)
		{
			SetOnBoardLED(i);
			Delaymillis(1);
		}
		for (int16_t i = 0xFFF; i >= 0x0; i = i - 45)
		{
			SetOnBoardLED(i);
			Delaymillis(1);
		}
	}

	FastMathInit();
	StartADC();
	StartInverterPWM();
	OffGateDriver();
	StartControlTimer();
}
