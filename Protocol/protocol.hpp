#ifndef PROTOCOL_HPP_
#define PROTOCOL_HPP_

#include <Protocol/fifo.hpp>
#include <Lowlevel/lowlevel.hpp>

class Protocol
{
public:
	FIFO uartFIFO = FIFO();
	void Update();

private:
	void ParserUpdate(uint8_t nextData);
	uint8_t HeaderParser(uint8_t nextData);
	uint16_t CRC16(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
	uint8_t recvPacket[0xFF];
	uint16_t recvPacketIdx = 0;

	uint8_t parserStatus = 0;

	uint8_t headerParserStatus = 0;
	uint8_t headerParserID;
	uint16_t headerParserLength;
	uint8_t headrParserInstruction;
};

#endif
