#include <Protection/protection.hpp>

void Protection::Init()
{
	voltageErrorHigh = supplyVoltage + SUPPLY_PROTECTION;
	voltageErrorLow = supplyVoltage - SUPPLY_PROTECTION;
}

void Protection::Update()
{
	if(controlStatus != STATUS_NONE)
			supplyVoltage = GetDCVoltageRaw() * DC_VOLTAGE_COEFF;

	if(supplyVoltage <= voltageErrorLow)
	{
		//voltage error
		OffGateDriver();
		controlStatus = STATUS_NONE;
		SetOnBoardLED(0xFFF);
	}

	//TODO Add MOSFET and motor over temperature protection
	//TODO Add gate fault Control
}
