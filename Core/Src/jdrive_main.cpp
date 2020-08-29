#include <jdrive_main.hpp>
#include <Lowlevel/lowlevel.hpp>
#include <Util/util.hpp>
#include <cstdint>
#include <FastMath/fast_math.hpp>
#include <MotorControl/motor_control.hpp>
#include <Calibration/calibration.hpp>
#include <stdio.h>
#include <Protection/protection.hpp>

MotorControl motorControl = MotorControl();
Calibration calibration = Calibration();
Protection protection = Protection();

uint8_t controlStatus = STATUS_NONE;

void Control();

//TODO make error controller

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

	Delaymillis(1);
	for (uint8_t i = 0; i < 100; i++)
	{
		Delaymillis(1);
		motorControl.supplyVoltage += GetDCVoltageRaw() * DC_VOLTAGE_COEFF;
	}
	motorControl.supplyVoltage /= 100.0f;
	protection.supplyVoltage = motorControl.supplyVoltage;

	if (motorControl.supplyVoltage >= OVERVOLTAGE_PROTECTION || motorControl.supplyVoltage <= UNDERVOLTAGE_PROTECTION)
	{
		while (1)
		{
			SetOnBoardLED(0xFFF);
			Delaymillis(100);
			SetOnBoardLED(0x0);
			Delaymillis(100);
		}
	}

	SetOnBoardLED(0xFFF);
	Delaymillis(500);
	SetOnBoardLED(0x0);

	calibration.calibrationVoltage = 0.05f;
	motorControl.motorParam.polePair = 7;
	SetPhaseOrder(1);

	motorControl.controlParam.goalCurrent = 0.02f;
	motorControl.currentPIDParam_d.Kp = 5.0f;
	motorControl.currentPIDParam_q.Kp = 5.0f;
	motorControl.currentPIDParam_d.Ki = 20.0f;
	motorControl.currentPIDParam_q.Ki = 20.0f;
	motorControl.currentPIDParam_d.Ka = 1.0f / motorControl.currentPIDParam_d.Kp;
	motorControl.currentPIDParam_q.Ka = 1.0f / motorControl.currentPIDParam_q.Kp;

	OnGateDriver();
	StartInverterPWM();
	motorControl.Init();
	calibration.Init();
	protection.Init();

	controlStatus = STATUS_CALIBRATION;

	while (1)
	{
		printf("$ %f %f %f %f;\n", motorControl.controlParam.goalCurrent * 100, motorControl.iq * 100, motorControl.id * 100, motorControl.currentPIDParam_q.i * 100);
	};
}

void Control()
{
	if (controlStatus != STATUS_NONE)
	{
		protection.Update();

		if (controlStatus == STATUS_MOTORCONTROL)
		{
			motorControl.ControlUpdate();
		}
		else if (controlStatus == STATUS_CALIBRATION)
		{
			calibration.CalibrationUpdate();

			if (calibration.done)
			{
				motorControl.motorParam.encoderOffset = calibration.encoderOffset;
				motorControl.ADC1Offset = calibration.ADC1Offset;
				motorControl.ADC2Offset = calibration.ADC2Offset;
				motorControl.Init();
				calibration.Init();
				controlStatus = STATUS_MOTORCONTROL;
			}
		}
	}
}
