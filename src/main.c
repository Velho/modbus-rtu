/*
 ******************************************************************************
 File:     main.c
 Info:     Generated by Atollic TrueSTUDIO(R) 9.3.0   2021-02-07

 The MIT License (MIT)
 Copyright (c) 2019 STMicroelectronics

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 ******************************************************************************
 */

/* Includes */
#include <stddef.h>
#include "stm32l1xx.h"

/* Private typedef */
/* Private define  */
/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */
void SetSysClock(void);
/* USART */
void USART_init(void);
void USART_write(char data);
void delay_ms(int delay);

int usart_sent = 0;


/**
 * Modbus frame management interface.
 */

#define MODBUS_MAX_APDU_SZ 8

typedef struct modbus_buffer {
	uint8_t buffer[80];
} mb_buf_t;

typedef struct modbus_com {
	uint8_t buffer[80];
	uint16_t size;
	int completed;
	int rx_handled;
} mb_com_t;

static mb_com_t com = { 0 };

/**
 **===========================================================================
 **
 **  Abstract: main program
 **
 **===========================================================================
 */
int main(void) {
	/**
	 *  IMPORTANT NOTE!
	 *  See the <system_*.c> file and how/if the SystemInit() function updates
	 *  SCB->VTOR register. Sometimes the symbol VECT_TAB_SRAM needs to be defined
	 *  when building the project if code has been located to RAM and interrupts
	 *  are used. Otherwise the interrupt table located in flash will be used.
	 *  E.g.  SCB->VTOR = 0x20000000;
	 */

	__disable_irq();
	USART_init();

	/* Configure the system clock to 32 MHz and update SystemCoreClock */
	SetSysClock();
	SystemCoreClockUpdate();

	/* TODO - Add your application code here */

	NVIC_EnableIRQ(USART2_IRQn);
	__enable_irq();

	RCC->AHBENR |= 1; // Enable ABH bus clock.

	// Blink out LD5.
	GPIOA->MODER &= ~0x00000C00;	//clear (input reset state for PA5). p184
	GPIOA->MODER |= 0x400; 			//GPIOA pin 5 to output. p184
	GPIOA->ODR ^= 0x20;				//0010 0000 xor bit 5. p186
	delay_ms(1000);
	GPIOA->ODR ^= 0x20;				//0010 0000 xor bit 5. p186
	delay_ms(1000);

	/* Infinite loop */
	while (1) {

		if (com.completed) {
			// Let's send indication of completed modbus frame.
			char text[] = "Frame assembled";
			int i;
			for (i = 0; i < sizeof(text); i++) {
				USART_write(text[i]);
			}

			com.rx_handled = 1; // Just a control flag.
			com.size = 0;
			com.completed = 0;
		}

	}
	return 0;
}

void delay_ms(int delay) {
	int i = 0;
	for (; delay > 0; delay--)
		for (i = 0; i < 2460; i++)
			; 	//measured with oscilloscope
}

/**
 * @brief  Configures the System clock frequency, AHB/APBx prescalers and Flash
 *         settings.
 * @note   This function should be called only once the RCC clock configuration
 *         is reset to the default reset state (done in SystemInit() function).
 * @param  None
 * @retval None
 */
#define HSI_STARTUP_TIMEOUT   ((uint16_t)0x0500) /* Time out for HSI start up */

void SetSysClock(void) {
	__IO uint32_t StartUpCounter = 0, HSIStatus = 0;

	/* Enable HSI */
	RCC->CR |= ((uint32_t) RCC_CR_HSION);

	/* Wait till HSI is ready and if Time out is reached exit */
	do {
		HSIStatus = RCC->CR & RCC_CR_HSIRDY;
	} while ((HSIStatus == 0) && (StartUpCounter != HSI_STARTUP_TIMEOUT));

	if ((RCC->CR & RCC_CR_HSIRDY) != RESET) {
		HSIStatus = (uint32_t) 0x01;
	} else {
		HSIStatus = (uint32_t) 0x00;
	}

	if (HSIStatus == (uint32_t) 0x01) {
		/*  PLL configuration: PLLCLK = (HSI * 6)/3 = 32 MHz */
		RCC->CFGR &= (uint32_t) ((uint32_t) ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL
				| RCC_CFGR_PLLDIV));
		RCC->CFGR |= (uint32_t) (RCC_CFGR_PLLSRC_HSI | RCC_CFGR_PLLMUL4
				| RCC_CFGR_PLLDIV2);
	}

	else {
		/* If HSI fails to start-up, the application will have wrong clock
		 configuration. User can add here some code to deal with this error */
	}

	/* Enable 64-bit access */
	FLASH->ACR |= FLASH_ACR_ACC64;

	/* Enable Prefetch Buffer */
	FLASH->ACR |= FLASH_ACR_PRFTEN;

	/* Flash 1 wait state */
	FLASH->ACR |= FLASH_ACR_LATENCY;

	/* Power enable */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	/* Select the Voltage Range 1 (1.8 V) */
	PWR->CR = PWR_CR_VOS_0;

	/* Wait Until the Voltage Regulator is ready */
	while ((PWR->CSR & PWR_CSR_VOSF) != RESET)
		;

	/* HCLK = SYSCLK /1*/
	RCC->CFGR |= (uint32_t) RCC_CFGR_HPRE_DIV1;

	/* PCLK2 = HCLK /1*/
	RCC->CFGR |= (uint32_t) RCC_CFGR_PPRE2_DIV1;

	/* PCLK1 = HCLK /1*/
	RCC->CFGR |= (uint32_t) RCC_CFGR_PPRE1_DIV1;

	/* Enable PLL */
	RCC->CR |= RCC_CR_PLLON;

	/* Wait till PLL is ready */
	while ((RCC->CR & RCC_CR_PLLRDY) == 0)
		;

	/* Select PLL as system clock source */
	RCC->CFGR &= (uint32_t) ((uint32_t) ~(RCC_CFGR_SW));
	RCC->CFGR |= (uint32_t) RCC_CFGR_SW_PLL;

	/* Wait till PLL is used as system clock source */
	while ((RCC->CFGR & (uint32_t) RCC_CFGR_SWS) != (uint32_t) RCC_CFGR_SWS_PLL)
		;
}


void USART_init(void) {
	RCC->APB1ENR|=0x00020000; 	//set bit 17 (USART2 EN)
	RCC->AHBENR|=0x00000001; 	//enable GPIOA port clock bit 0 (GPIOA EN)
	GPIOA->AFR[0]=0x00000700;	//GPIOx_AFRL p.188,AF7 p.177
	GPIOA->AFR[0]|=0x00007000;	//GPIOx_AFRL p.188,AF7 p.177
	GPIOA->MODER|=0x00000020; 	//MODER2=PA2(TX) to mode 10=alternate function mode. p184
	GPIOA->MODER|=0x00000080; 	//MODER2=PA3(RX) to mode 10=alternate function mode. p184

	// Configure USART2.
	USART2->BRR = 0x00000116;	// 115200 BAUD and crystal 32MHz. p710, D05

	USART2->CR1 = 0x00000008;	// TE bit. p739-740. Enable transmit
	USART2->CR1 |= 0x00000004;	// RE bit. p739-740. Enable receiver
	USART2->CR1 |= 0x00002000;	// UE bit. p739-740. Uart enable

//	USART2->CR1 |= (1 << 6); // TC : Transmission complete.
	// USART2->CR1 |= 0x80; // TXE : Transmit data register empty.
//	USART2->CR1 |= 0x0020;		// RXNE bit : Read data register not empty

	// See page, 739 (CR1).
	USART2->CR1 |= USART_CR1_RXNEIE;
}

void USART_write(char data) {
	// Wait for TX buffer.
	while (!(USART2->SR & 0x0080));
	USART2->DR = (data);
}

static void append(uint8_t data) {
	com.buffer[com.size] = data;
	com.size++;

	// Mark the frame done.
	if (com.size >= MODBUS_MAX_APDU_SZ)
		com.completed = 1;
}

void USART2_IRQHandler(void) {
	// uint8_t * rx = modbus_read();
	volatile unsigned char data; // We do not lift.

	// RXNEIE : Data register not empty.
	if (USART2->SR & USART_CR1_RXNEIE) {
		data = (unsigned char)USART2->DR;
		append(data);

		// Echo for now.
		USART_write(data);
	}
}

