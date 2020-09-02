#ifndef PROTOCOL_HPP_
#define PROTOCOL_HPP_

#include <Protocol/fifo.hpp>
#include <Lowlevel/lowlevel.hpp>
#include <Protocol/controlTable.hpp>

class Protocol
{
public:
	FIFO uartFIFO = FIFO();
	ControlTable controlTable = ControlTable();
	void Update();

private:
	void ParserUpdate(uint8_t nextData);
	uint8_t HeaderParser(uint8_t nextData);
	uint8_t CheckID(uint8_t ID);
	uint16_t CRC16(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);

	void RunInstruction();

	void PingInstruction();

	void InitSendPacket();
	void SetPacketID(uint8_t ID);
	void SetPacketErrorCode(uint8_t errorCode);
	void AddParam(uint8_t data);
	void SendPacket();
	uint8_t sendPacket[0xFF] = { 0xFF, 0xFF, 0xFD, 0x00, 0x00, 0x00, 0x00, 0x55, 0x00 };
	uint16_t sendPacketParamIdx = 9;
	uint16_t sendPacketLenght = 4;

	uint8_t recvPacket[0xFF];
	uint16_t recvPacketIdx = 0;
	uint16_t recvCounter = 0;

	uint8_t parserStatus = 0;

	uint8_t IDResult = 0;

	uint8_t headerParserStatus = 0;
	uint8_t headerParserID;
	uint16_t headerParserLength;
	uint8_t headrParserInstruction;
};

#endif
