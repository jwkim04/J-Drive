#include <Protocol/controlTable.hpp>

void ControlTable::Init()
{
	uint16_t address = 0;

	InitTable(address++ , MODEL_NUMBER, R, 2, INT, FIXED); //Model Number
	InitTable(address++ , ((FW_VER_MAJOR << 4) | FW_VER_MINOR), R, 1, INT, FIXED); //Firmware Version
	InitTable(address++ , 1, RW, 2, INT, EEPROM); //ID
	InitTable(address++ , 0, RW, 1, INT, EEPROM); //Baud Rate
	InitTable(address++ , 250, RW, 1, INT, EEPROM); //Return Delay Time
	InitTable(address++ , 0, RW, 1, INT, EEPROM); //Drive Mode
	InitTable(address++ , 0, RW, 1, INT, EEPROM); //Operating Mode
	InitTable(address++ , 0, RW, 1, INT, EEPROM); //Secondary ID
	InitTable(address++ , 0x00000000, RW, 4, FLOAT, EEPROM); //Homing Offset
	InitTable(address++ , 0x3F000000, RW, 4, FLOAT, EEPROM); //Moving Threshold
	InitTable(address++ , 0x42A00000, RW, 4, FLOAT, EEPROM); //Board Temperature Limit
	InitTable(address++ , 0x00000000, RW, 4, FLOAT, EEPROM); //Motor Temperature Limit
	InitTable(address++ , 0x41F00000, RW, 4, FLOAT, EEPROM); //Max Voltage Limit
	InitTable(address++ , 0x41000000, RW, 4, FLOAT, EEPROM); //Min Voltage Limit
	InitTable(address++ , 0, RW, 1, INT, EEPROM); //Shutdown
	InitTable(address++ , 0, RW, 1, INT, EEPROM); //Battery Mode
	InitTable(address++ , 0x41A80000, RW, 4, FLOAT, EEPROM); //Battery Cutoff Voltage
	InitTable(address++ , 0x41A00000, RW, 4, FLOAT, EEPROM); //Current I Gain
	InitTable(address++ , 0x40C00000, RW, 4, FLOAT, EEPROM); //Current P Gain
	InitTable(address++ , 0x3C23D70A, RW, 4, FLOAT, EEPROM); //Velocity I Gain
	InitTable(address++ , 0x3BA3D70A, RW, 4, FLOAT, EEPROM); //Velocity P Gain
	InitTable(address++ , 0x00000000, RW, 4, FLOAT, EEPROM); //Position I Gain
	InitTable(address++ , 0x41200000, RW, 4, FLOAT, EEPROM); //Position P Gain
	InitTable(address++ , 0x00000000, RW, 4, FLOAT, EEPROM); //Encoder Offset
	InitTable(address++ , 7, RW, 1, INT, EEPROM); //Pole Pair

	InitTable(address++ , 0, RW, 1, INT, RAM); //Inverter Enable
	InitTable(address++ , 0, RW, 2, INT, RAM); //LED
	InitTable(address++ , 0, RW, 1, INT, RAM); //Status Return Level
	InitTable(address++ , 0, R, 1, INT, RAM); //Registered Instruction
	InitTable(address++ , 0, R, 1, INT, RAM); //Hardware Error Status
	InitTable(address++ , 0, RW, 1, INT, RAM); //Bus Watchdog
	InitTable(address++ , 0x00000000, RW, 4, FLOAT, RAM); //Goal PWM
	InitTable(address++ , 0x00000000, RW, 4, FLOAT, RAM); //Goal Current
	InitTable(address++ , 0x00000000, RW, 4, FLOAT, RAM); //Goal Velocity
	InitTable(address++ , 0x00000000, RW, 4, FLOAT, RAM); //Goal Position
	InitTable(address++ , 0x00000000, RW, 4, FLOAT, RAM); //Stiffness
	InitTable(address++ , 0x00000000, RW, 4, FLOAT, RAM); //Friction
	InitTable(address++ , 0x00000000, RW, 4, FLOAT, RAM); //Inertia
	InitTable(address++ , 0, R, 2, INT, RAM); //Realtime Tick
	InitTable(address++ , 0, R, 1, INT, RAM); //Moving
	InitTable(address++ , 0, R, 1, INT, RAM); //Moving Status
	InitTable(address++ , 0x00000000, R, 4, FLOAT, RAM); //Present PWM
	InitTable(address++ , 0x00000000, R, 4, FLOAT, RAM); //Present Current
	InitTable(address++ , 0x00000000, R, 4, FLOAT, RAM); //Present Velocity
	InitTable(address++ , 0x00000000, R, 4, FLOAT, RAM); //Present Position
	InitTable(address++ , 0x00000000, R, 4, FLOAT, RAM); //Present Input Voltage
	InitTable(address++ , 0x00000000, R, 4, FLOAT, RAM); //Present Board Temperature
	InitTable(address++ , 0, R, 4, INT, RAM); //Present Motor Temperature
	InitTable(address++ , 0, RW, 1, INT, RAM); //Calibration
}

void ControlTable::LoadTableFromEEPROM()
{

}

void ControlTable::InitTable(uint16_t address, uint32_t initalValue, uint8_t access, uint8_t len, uint8_t type, uint8_t location)
{

}
