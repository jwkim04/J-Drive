#include <cstdint>
#include <BoardConfig/board_config.h>

#ifndef FIFO_HPP_
#define FIFO_HPP_

class FIFO
{
public:
	void push(uint8_t data);
	uint8_t pop();

	uint32_t size = 0;
private:
	uint32_t topIdx = 0;
	uint32_t bottomIdx = 0;
	uint8_t buffer[UART_FIFO_BUFFER_SIZE];
};

#endif
