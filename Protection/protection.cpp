#include <Protection/protection.hpp>

void Protection::Init()
{
	voltageErrorHigh = supplyVoltage + SUPPLY_PROTECTION;
	voltageErrorLow = supplyVoltage - SUPPLY_PROTECTION;
}

void Protection::Update()
{
	supplyVoltage = GetDCVoltageRaw() * DC_VOLTAGE_COEFF;

	if (controlTable->controlTableData[15].data >= 1)
	{
		if (supplyVoltage <= *(float*) &controlTable->controlTableData[16].data)
		{
			controlTable->controlTableData[31].data |= BATTERY_VOLTAGE_ERROR_MASK;
			controlTable->SetTable(29, 0, 1);
			//Battery Voltage Error
		}
		else
		{
			controlTable->controlTableData[31].data &= BATTERY_VOLTAGE_ERROR_MASK;
		}
	}
	else
	{
		if (controlTable->controlTableData[5].data == 1)
		{
			if (supplyVoltage <= voltageErrorLow)
			{
				controlTable->controlTableData[31].data |= VOLTAGE_ERROR_MASK;
				controlTable->SetTable(29, 0, 1);
				//Voltage Error
			}
			else
			{
				controlTable->controlTableData[31].data &= VOLTAGE_ERROR_MASK;
			}
		}
		else if (controlTable->controlTableData[5].data == 2)
		{
			if (supplyVoltage <= voltageErrorLow || supplyVoltage >= voltageErrorHigh)
			{
				controlTable->controlTableData[31].data |= VOLTAGE_ERROR_MASK;
				controlTable->SetTable(29, 0, 1);
				//Voltage Error
			}
			else
			{
				controlTable->controlTableData[31].data &= VOLTAGE_ERROR_MASK;
			}
		}
	}

	if (GateFault())
	{
		controlTable->controlTableData[31].data |= ELECTRICAL_SHOCK_ERROR_MASK;
		controlTable->SetTable(29, 0, 1);
		//Electrical Shock Error
	}
	else
	{
		controlTable->controlTableData[31].data &= ELECTRICAL_SHOCK_ERROR_MASK;
	}

	if (GetFETTempRaw() * ONBOARD_TEMP_COEFF >= *(float*) &controlTable->controlTableData[10].data)
	{
		controlTable->controlTableData[31].data |= BOARD_TEMPERATURE_ERROR_MASK;
		//Board Temperature Error
	}
	else
	{
		controlTable->controlTableData[31].data &= BOARD_TEMPERATURE_ERROR_MASK;
	}

	if (GetMotorTempRaw() >= controlTable->controlTableData[11].data)
	{
		controlTable->controlTableData[31].data |= MOTOR_TEMPERATURE_ERROR_MASK;
		//Motor Temperature Error
	}
	else
	{
		controlTable->controlTableData[31].data &= MOTOR_TEMPERATURE_ERROR_MASK;
	}
}

void Protection::SetControlTable(ControlTable *_controlTable)
{
	controlTable = _controlTable;
}
