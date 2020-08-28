#include <Protection/protection.hpp>

void Protection::Init()
{
	voltageErrorHigh = supplyVoltage + SUPPLY_PROTECTION;
	voltageErrorLow = supplyVoltage - SUPPLY_PROTECTION;
}

void Protection::Update()
{
	supplyVoltage = GetDCVoltageRaw() * DC_VOLTAGE_COEFF;

	if(supplyVoltage <= voltageErrorLow || supplyVoltage >= voltageErrorHigh)
	{
		//voltage error
		OffGateDriver();
		controlStatus = STATUS_NONE;
	}
}
