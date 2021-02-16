#include "serial_irq.h"
#include "modbus/rtu.h"

#include "stm32l1xx.h"

void USART2_IRQHandler(void) {
	// RXNEIE : Data register not empty.
	if (USART2->SR & USART_CR1_RXNEIE) {
		unsigned char data = USART2->DR;
		// Modbus handle.
		modbus_read(data);
		// Echo for now.
		USART_write(data);
	}
}

