#include <Protocol/fifo.hpp>
#include <Lowlevel/lowlevel.hpp>

uint8_t FIFO::pop()
{
	uint8_t data = buffer[bottomIdx++];

	if (bottomIdx >= UART_FIFO_BUFFER_SIZE)
	{
		bottomIdx = 0;
	}

	return data;
}
