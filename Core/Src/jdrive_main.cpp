#include <jdrive_main.hpp>
#include <Lowlevel/lowlevel.hpp>
#include <Util/util.hpp>
#include <cstdint>
#include <FastMath/fast_math.hpp>
#include <MotorControl/motor_control.hpp>
#include <Calibration/calibration.hpp>
#include <stdio.h>
#include <Protection/protection.hpp>
#include <Protocol/protocol.hpp>

MotorControl motorControl = MotorControl();
Calibration calibration = Calibration();
Protection protection = Protection();
Protocol protocol = Protocol();

ControlTable *controlTable;

uint8_t controlStatus = STATUS_NONE;

float presentSupplyVoltage;
float presentBoardTemp;

void Control();

//TODO make error control code
//TODO add TX FIFO and delay

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

	protocol.controlTable.Init();
	controlTable = &protocol.controlTable;

	if (ReadEEPROM(2) > (uint32_t) 0xFE)
	{
		controlTable->FactoryReset();
	}

	controlTable->LoadControlTableFromEEPROM();

	switch (controlTable->controlTableData[3].data)
	{
	case 0:
		SetUartBuadRate(9600);
		break;
	case 1:
		SetUartBuadRate(57600);
		break;
	case 2:
		SetUartBuadRate(115200);
		break;
	case 3:
		SetUartBuadRate(1000000);
		break;
	case 4:
		SetUartBuadRate(2000000);
		break;
	default:
		SetUartBuadRate(9600);
		break;
	}

	FastMathInit();
	StartADC();
	SetControlFunc(Control);
	SetUartFIFO(&protocol.uartFIFO);
	StartControlTimer();
	StartUartInterrupt();

	motorControl.SetControlTable(&protocol.controlTable);
	calibration.SetControlTable(&protocol.controlTable);
	protection.SetControlTable(&protocol.controlTable);

	Delaymillis(1);
	for (uint8_t i = 0; i < 100; i++)
	{
		Delaymillis(1);
		motorControl.supplyVoltage += GetDCVoltageRaw() * DC_VOLTAGE_COEFF;
	}
	motorControl.supplyVoltage /= 100.0f;
	protection.supplyVoltage = motorControl.supplyVoltage;

	if (motorControl.supplyVoltage >= *(float*) &controlTable->controlTableData[12].data || motorControl.supplyVoltage <= *(float*) &controlTable->controlTableData[13].data)
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

	SetPhaseOrder(controlTable->controlTableData[23].data);

	OffGateDriver();
	StartInverterPWM();
	motorControl.Init();
	calibration.Init();
	protection.Init();

	StartTimer();

	controlStatus = STATUS_NONE;

	while (1)
	{
		protocol.Update();
	}
}

uint32_t pos;

void Control()
{
	TimerUpdate();

	presentSupplyVoltage = (float) GetDCVoltageRaw() * DC_VOLTAGE_COEFF;
	presentBoardTemp = (float) GetFETTempRaw() * ONBOARD_TEMP_COEFF;
	controlTable->controlTableData[47].data = *(uint32_t*) &presentSupplyVoltage;
	controlTable->controlTableData[48].data = *(uint32_t*) &presentBoardTemp;
	controlTable->controlTableData[49].data = GetMotorTempRaw();

	controlTable->controlTableData[40].data = GetTimerTick();

	SetPhaseOrder(controlTable->controlTableData[23].data);

	motorControl.Encoder.polePair = controlTable->controlTableData[27].data;
	motorControl.motorParam.polePair = controlTable->controlTableData[27].data;

	if (controlTable->controlTableData[50].data == 1)
	{
		controlStatus = STATUS_CALIBRATION;
	}

	if (controlTable->controlTableData[28].data == 1)
	{
		OnGateDriver();
	}
	else
	{
		OffGateDriver();
		SetInverterPWMDuty(0x7FF, 0x7FF, 0x7FF);
		controlTable->controlTableData[29].data = 0;
		controlStatus = STATUS_NONE;
	}

	if (controlTable->controlTableData[29].data == 1 && controlStatus != STATUS_CALIBRATION && controlTable->controlTableData[28].data == 1)
	{
		controlStatus = STATUS_MOTORCONTROL;
	}
	else
	{
		if (controlStatus != STATUS_CALIBRATION)
		{
			SetInverterPWMDuty(0x7FF, 0x7FF, 0x7FF);
			controlStatus = STATUS_NONE;
		}
	}

	if (controlStatus != STATUS_NONE)
	{
		protection.Update();

		if (controlStatus == STATUS_MOTORCONTROL)
		{
			motorControl.ControlUpdate();
		}
		else if (controlStatus == STATUS_CALIBRATION)
		{
			SetOnBoardLED(0xFFF);
			calibration.CalibrationUpdate();

			if (calibration.done)
			{
				protocol.controlTable.SetTable(28, 0, 1);
				protocol.controlTable.SetTable(26, *(uint32_t*) &calibration.encoderOffset, 4);
				protocol.controlTable.SetTable(24, calibration.ADC1Offset, 4);
				protocol.controlTable.SetTable(25, calibration.ADC2Offset, 4);
				protocol.controlTable.SetTable(28, 1, 1);

				motorControl.Init();
				calibration.Init();

				protocol.controlTable.SetTable(50, 0, 1);

				controlStatus = STATUS_NONE;
				SetOnBoardLED(0x0);
			}
		}
	}
}
