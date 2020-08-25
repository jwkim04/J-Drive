#include <jdrive_main.hpp>
#include <Lowlevel/lowlevel.hpp>
#include <Util/util.hpp>
#include <cstdint>
#include <FastMath/fast_math.hpp>
#include <MotorControl/motor_control.hpp>

MotorControl Motor = MotorControl();

void Control();

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
	SetControlFunc(Control);
	StartControlTimer();

	for (uint8_t i = 0; i < 100; i++)
	{
		Delaymillis(1);
		Motor.supplyVoltage += GetDCVoltageRaw() * DC_VOLTAGE_COEFF;
	}
	Motor.supplyVoltage /= 100.0f;

	if (Motor.supplyVoltage >= OVERVOLTAGE_PROTECTION || Motor.supplyVoltage <= UNDERVOLTAGE_PROTECTION)
	{
		//Supply voltage error
		while (1)
		{
			SetOnBoardLED(0xFFF);
			Delaymillis(100);
			SetOnBoardLED(0x0);
			Delaymillis(100);
		}
	}

	//Startup success
	SetOnBoardLED(0xFFF);
	Delaymillis(500);
	SetOnBoardLED(0x0);

	OffGateDriver();
	StartInverterPWM();
	ControlStart();

	while(1);
}

void Control()
{
	Motor.ControlUpdate();
}
