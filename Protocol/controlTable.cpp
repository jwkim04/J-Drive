#include <Protocol/controlTable.hpp>

void ControlTable::Init()
{
	uint16_t address = 0;

	InitTable(address++, MODEL_NUMBER, R, 2, INT, FIXED); //Model Number
	InitTable(address++, ((FW_VER_MAJOR << 4) | FW_VER_MINOR), R, 1, INT, FIXED); //Firmware Version
	InitTable(address++, 1, RW, 1, INT, EEPROM); //ID
	InitTable(address++, 3, RW, 1, INT, EEPROM); //Baud Rate
	InitTable(address++, 250, RW, 1, INT, EEPROM); //Return Delay Time
	InitTable(address++, 1, RW, 1, INT, EEPROM); //Voltage Protection Mode
	InitTable(address++, 4, RW, 1, INT, EEPROM); //Operating Mode
	InitTable(address++, 255, RW, 1, INT, EEPROM); //Secondary ID
	InitTable(address++, 0x00000000, RW, 4, FLOAT, EEPROM); //Homing Offset
	InitTable(address++, 0x3F000000, RW, 4, FLOAT, EEPROM); //Moving Threshold
	InitTable(address++, 0x42A00000, RW, 4, FLOAT, EEPROM); //Board Temperature Limit
	InitTable(address++, 0x00000000, RW, 4, FLOAT, EEPROM); //Motor Temperature Limit
	InitTable(address++, 0x41F00000, RW, 4, FLOAT, EEPROM); //Max Voltage Limit
	InitTable(address++, 0x41000000, RW, 4, FLOAT, EEPROM); //Min Voltage Limit
	InitTable(address++, 0, RW, 1, INT, EEPROM); //Shutdown
	InitTable(address++, 0, RW, 1, INT, EEPROM); //Battery Mode
	InitTable(address++, 0x41A80000, RW, 4, FLOAT, EEPROM); //Battery Cutoff Voltage
	InitTable(address++, 0x41A00000, RW, 4, FLOAT, EEPROM); //Current I Gain
	InitTable(address++, 0x40C00000, RW, 4, FLOAT, EEPROM); //Current P Gain
	InitTable(address++, 0x3C23D70A, RW, 4, FLOAT, EEPROM); //Velocity I Gain
	InitTable(address++, 0x3BA3D70A, RW, 4, FLOAT, EEPROM); //Velocity P Gain
	InitTable(address++, 0x00000000, RW, 4, FLOAT, EEPROM); //Position I Gain
	InitTable(address++, 0x41200000, RW, 4, FLOAT, EEPROM); //Position P Gain
	InitTable(address++, 0x00000000, RW, 1, INT, EEPROM); //Phase Order
	InitTable(address++, 0, RW, 4, INT, EEPROM); //ADC1 Offset
	InitTable(address++, 0, RW, 4, INT, EEPROM); //ADC2 Offset
	InitTable(address++, 0x00000000, RW, 4, FLOAT, EEPROM); //Encoder Offset
	InitTable(address++, 7, RW, 1, INT, EEPROM); //Pole Pair

	InitTable(address++, 0, RW, 1, INT, RAM); //Inverter Enable
	InitTable(address++, 0, RW, 1, INT, RAM); //Torque Enable
	InitTable(address++, 0, R, 1, INT, RAM); //Registered Instruction
	InitTable(address++, 0, R, 1, INT, RAM); //Hardware Error Status
	InitTable(address++, 0, RW, 1, INT, RAM); //Bus Watchdog
	InitTable(address++, 0x3DCCCCCD, RW, 4, FLOAT, RAM); //Goal PWM
	InitTable(address++, 0x3F000000, RW, 4, FLOAT, RAM); //Goal Current
	InitTable(address++, 0x42200000, RW, 4, FLOAT, RAM); //Goal Velocity
	InitTable(address++, 0x00000000, RW, 4, FLOAT, RAM); //Goal Position
	InitTable(address++, 0x3C23D70A, RW, 4, FLOAT, RAM); //Stiffness
	InitTable(address++, 0x00000000, RW, 4, FLOAT, RAM); //Friction
	InitTable(address++, 0x00000000, RW, 4, FLOAT, RAM); //Inertia
	InitTable(address++, 0, R, 2, INT, RAM); //Realtime Tick
	InitTable(address++, 0, R, 1, INT, RAM); //Moving
	InitTable(address++, 0, R, 1, INT, RAM); //Moving Status
	InitTable(address++, 0x00000000, R, 4, FLOAT, RAM); //Present PWM
	InitTable(address++, 0x00000000, R, 4, FLOAT, RAM); //Present Current
	InitTable(address++, 0x00000000, R, 4, FLOAT, RAM); //Present Velocity
	InitTable(address++, 0x00000000, R, 4, FLOAT, RAM); //Present Position
	InitTable(address++, 0x00000000, R, 4, FLOAT, RAM); //Present Input Voltage
	InitTable(address++, 0x00000000, R, 4, FLOAT, RAM); //Present Board Temperature
	InitTable(address++, 0, R, 4, INT, RAM); //Present Motor Temperature
	InitTable(address++, 0, RW, 1, INT, RAM); //Calibration
	InitTable(address++, 0x3D4CCCCD, RW, 4, FLOAT, RAM); //Calibration PWM
}

void ControlTable::LoadControlTableFromEEPROM()
{
	for (uint32_t address = 0; address < CONTROLTABLE_EEPROM_END_ADDRESS + 1; address++)
	{
		controlTableData[address].data = ReadEEPROM(address);
	}
}

void ControlTable::FactoryReset()
{
	UnLockEEPROM();
	for (uint32_t address = 0; address < CONTROLTABLE_EEPROM_END_ADDRESS + 1; address++)
	{
		WriteEEPROM((uint32_t) address, controlTableData[address].initialValue);
	}
	LockEEPROM();
}

uint8_t ControlTable::SetTable(uint16_t address, uint32_t data, uint8_t len)
{
	if (controlTableData[29].data != 0 && controlTableData[28].data != 0 && controlTableData[address].location == EEPROM)
	{
		return 1;
	}

	if (address > CONTROLTABLE_END_ADDRESS)
	{
		return 1;
	}

	if (len < controlTableData[address].len)
	{
		return 2;
	}
	else if (controlTableData[address].access == R)
	{
		return 1;
	}
	else
	{
		controlTableData[address].data = data;

		if (controlTableData[address].location == EEPROM)
		{
			UnLockEEPROM();
			for (uint32_t _address = 0; _address < CONTROLTABLE_EEPROM_END_ADDRESS + 1; _address++)
			{
				WriteEEPROM((uint32_t) _address, controlTableData[_address].data);
			}
			LockEEPROM();

			if (address == 3)
			{
				HAL_NVIC_SystemReset();
			}
		}
		return 0;
	}
}

uint8_t ControlTable::GetTable(uint16_t address, uint32_t *data, uint8_t len)
{
	if (address > CONTROLTABLE_END_ADDRESS)
	{
		return 1;
	}
	else
	{
		*data = controlTableData[address].data;
		return 0;
	}
}

void ControlTable::InitTable(uint16_t address, uint32_t initialValue, uint8_t access, uint8_t len, uint8_t type, uint8_t location)
{
	controlTableData[address].initialValue = initialValue;
	controlTableData[address].data = initialValue;
	controlTableData[address].access = access;
	controlTableData[address].len = len;
	controlTableData[address].location = location;
	controlTableData[address].type = type;
}
