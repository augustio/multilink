#include <stdint.h>

#include "simple_uart.h"

int main(void)
{
	simple_uart_config(12, 11, 12, 12, false);

	simple_uart_putstring((const uint8_t *)"TX goes main loop\r\n");

	while (1)
		continue;

	return 0;
}
