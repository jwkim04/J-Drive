#ifndef PROTECTION_HPP_
#define PROTECTION_HPP_

#include <BoardConfig/board_config.h>
#include <cstdint>
#include <Lowlevel/lowlevel.hpp>
#include <Core/Inc/jdrive_main.hpp>
#include <Protocol/controlTable.hpp>
#include <ErrorControl/errorControl.hpp>

extern uint8_t controlStatus;

class Protection
{
public:
	void Update();
	void Init();

	float supplyVoltage = 0.0f;

	void SetControlTable(ControlTable *_controlTable);

private:
	ControlTable *controlTable;

	float voltageErrorHigh;
	float voltageErrorLow;
};

#endif
