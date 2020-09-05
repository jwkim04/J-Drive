#include <cstdint>
#include <BoardConfig/board_config.h>

#ifndef FIFO_HPP_
#define FIFO_HPP_

class FIFO
{
public:
	uint8_t pop();

	uint32_t topIdx = 0;
	uint32_t bottomIdx = 0;

	uint8_t buffer[UART_FIFO_BUFFER_SIZE];
private:

};

#endif
