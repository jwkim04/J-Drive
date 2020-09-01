#include <Protocol/fifo.hpp>

void FIFO::push(uint8_t data)
{
	size++;
	buffer[topIdx++] = data;

	if(topIdx == UART_FIFO_BUFFER_SIZE)
	{
		topIdx = 0;
	}
}

uint8_t FIFO::pop()
{
	size--;
	uint8_t data = buffer[bottomIdx++];

	if(bottomIdx == UART_FIFO_BUFFER_SIZE)
	{
		bottomIdx = 0;
	}

	return data;
}
