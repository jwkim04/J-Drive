#include <jdrive_main.hpp>
#include <Lowlevel/lowlevel.hpp>
#include <Util/util.hpp>
#include <cstdint>

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

	StartADC();
	TIM1->CCR1 = 0x7FF;
	TIM1->CCR2 = 0x7FF;
	TIM1->CCR3 = 0x7FF;
	StartInverterPWM();
	OffGateDriver();
	StartControlTimer();
}
