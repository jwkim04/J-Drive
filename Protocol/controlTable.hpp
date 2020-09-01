#ifndef CONTROLTABLE_HPP_
#define CONTROLTABLE_HPP_

#include <BoardConfig/board_config.h>
#include <cstdint>

enum Access
{
	R,
	RW
};

enum DataType
{
	INT,
	FLOAT
};

enum DataLocation
{
	EEPROM,
	RAM,
	FIXED
};

struct ControlTableData
{
	uint32_t data = 0x0000;
	uint32_t initialValue = 0x0000;
	uint8_t access = R;
	uint8_t len = 1;
	uint8_t type = INT;
	uint8_t location = EEPROM;
};

class ControlTable
{
public:
	ControlTableData controlTableData[49];

	void Init();
	void LoadTableFromEEPROM();
	uint8_t SetTable(uint16_t address, uint32_t data, uint8_t len);
	uint8_t GetTable(uint16_t address, uint32_t *data, uint8_t len);

private:
	void InitTable(uint16_t address, uint32_t initialValue, uint8_t access, uint8_t len, uint8_t type, uint8_t location);
};

#endif
