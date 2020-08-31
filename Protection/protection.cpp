#include <Protection/protection.hpp>

void Protection::Init()
{
	voltageErrorHigh = supplyVoltage + SUPPLY_PROTECTION;
	voltageErrorLow = supplyVoltage - SUPPLY_PROTECTION;
}

void Protection::Update()
{
	if (controlStatus != STATUS_NONE)
		supplyVoltage = GetDCVoltageRaw() * DC_VOLTAGE_COEFF;

	if (useBattery == 1)
	{
		if (supplyVoltage <= batteryCutoffVoltage)
		{
			OffGateDriver();
			controlStatus = STATUS_NONE;
			SetOnBoardLED(0xFFF);
		}
	}
	else
	{
		if (supplyVoltage <= voltageErrorLow || supplyVoltage >= voltageErrorHigh)
		{
			OffGateDriver();
			controlStatus = STATUS_NONE;
			SetOnBoardLED(0xFFF);
		}
	}

	if(GateFault())
	{
		OffGateDriver();
		controlStatus = STATUS_NONE;
		SetOnBoardLED(0xFFF);
	}

	//TODO Add MOSFET and motor over temperature protection
	//TODO Add gate fault Control
}
