#include "bme280-i2c.h"
#include "stm32l1xx.h"

static BME280_I2C_t bme280_i2c = { 0 };

void BME280_set_address(uint8_t address)
{
	bme280_i2c.address = address;
}

void BME280_I2C_init(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;	// Enable GPIOB clock PB8(D15)=SCL,PB9(D14)=SDA.
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // (??)

	GPIOB->AFR[1] &= ~0xFF;	  // PB8,PB9 I2C1 SCL, SDA. AFRH8 and AFRH9. clear
	GPIOB->AFR[1] |= 0x44;	  // GPIOx_AFRL p.189,AF4=I2C1(0100 BIN) p.177
	GPIOB->MODER &= ~0xF0000; // PB8 and PB9 clear

	GPIOB->MODER |= 0xA0000; // Alternate function mode PB8,PB9
	GPIOB->OTYPER |= 0x300;	 // output open-drain. p.184

	GPIOB->OSPEEDR |= 0xF0000; // High-speed selection for PB8-9.

	// Driver circuit includes the pull-up resistors.
	GPIOB->PUPDR &= ~0xF0000; // No pull-up resistors for PB8 and PB9 p.185

	/**
	 * Following is the required sequence in master mode.
	 * - Program the peripheral input clock in I2C_CR2 Register in
	 * 	 order generate correct timings.
	 * - Configure the clock control registers.
	 * - Configure the rise time register.
	 * - Program the I2C_CR1 register to enable the peripheral.
	 * -> Send the start bit when beginning the transmit.
	 * - Set the START bit in the I2C_CR1 register to generate a Start condition (During the init or write?).
	 * Peripheral input clock frequency must be at least:
	 * 	- 2 MHz in SM mode
	 *  - 4 MHz in FM mode
	 */

	// Software reset i2c1 SWRST p 682
	I2C1->CR1 |= I2C_CR1_SWRST;
	// Clear reset
	I2C1->CR1 &= ~I2C_CR1_SWRST;
	// CR2 Peripheral clock frequency.
	I2C1->CR2 = 0x20; // FREQ[5:0] = 32 MHz.

	// CCR and TRISE calculated against the peripheral clock.
	// CCR, Clock Control register.
	// Calculated CCR = T(scl) + T(sclh) / Tpclk1
	I2C1->CCR = 160;
	// TRISE,
	// TRISE = Tr(scl) / T(pclk1) + 1
	I2C1->TRISE = 33;

	// PERIPHERAL ENABLE ==>
	I2C1->CR1 |= 0x1; // Enable peripheral.
}

/**
 * [S][EV5][HEADER][A][EV9][ADDR][A][EV6][EV8_1][EV8][A][EV8][A][EV8_2][P]
 * S = Start, A = Ack, P = Stop
 * EV5: SB=1, cleared by reading SR1 register followed by writing DR register with Address.
 * EV6: ADDR=1, cleared by reading SR1 register followed by reading SR2.
 * EV8_1:TxE=1, shift register empty, data register empty, write Data1 in DR.
 * EV8:TxE=1, shift register not empty, data register empty, cleared by writing DR register.
 * EV8_2:TxE=1, BTF=1, Program Stop request. TxE and BTF are cleared by hardware by the Stop condition.
 * EV9:ADD10=1, cleared by reading SR1 register followed by writing DR register.
 */
size_t BME280_write(uint8_t reg, uint8_t *data, uint16_t len)
{
	int i;
	__IO int t;

	// Wait until the bus is ready.
	while (I2C1->SR2 & I2C_SR2_BUSY)
		;

	// Start the communication by writing the START COND.
	// Then the master waits for a read SR1 register followed by a write in the DR register
	// with the slave address.

	// [S]
	// Is this required as we are not in the reception mode?
	// Disable Acknowledge/PEC position (for data reception).
	I2C1->CR1 |= I2C_CR1_ACK;	// ACK (POS at first)
	I2C1->CR1 |= I2C_CR1_START; // Start condition

	// Test without the following.

	// Try the same as in the read function.
	// while (!(I2C1->SR1 & I2C_SR1_SB));
	// LSB set?
	I2C1->DR = bme280_i2c.address << 1; // Shift to enter Transmitter mode.(Required? as x76 lsb is not set.)
	while (!(I2C1->SR1 & I2C_SR1_ADDR))
		;

	//    while(!(I2C1->SR1 & I2C_SR1_BTF)); // Byte transfer finished.

	/**
	 * Reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag, even if the ADDR flag was
	 * set after reading I2C_SR1. Consequently, I2C_SR2 must be read only when ADDR is found
	 * set in I2C_SR1 or when the STOPF bit is cleared.
	 */
	t = I2C1->SR1 | I2C1->SR2;

	while (!(I2C1->SR1 & I2C_SR1_TXE))
		; // TxE: Data register empty (transmitters)

	I2C1->DR = reg;

	for (i = 0; i < len; i++)
	{
		// TxE or BTF
		while (!(I2C1->SR1 & I2C_SR1_TXE))
			; // TxE: Data register empty (tansmitters)
		// Should be writing the txbuffer.
		I2C1->DR = *data++;
		// BTF
		while (!(I2C1->SR1 & I2C_SR1_BTF))
			;
	}

	// Wait, this seems very odd.
	// STOPF ; Stop detection (slave mode)
	// Set by hardware when a Stop condition is detected on the bus
	// by the slave after an ack.
	// Cleared by software reading the SR1 register followed by a write in the CR1 register,
	// or by hardware PE=0.
	while (!(I2C1->SR1 & I2C_SR1_STOPF))
		; // STOPF ; Stop detection (slave mode)

	I2C1->CR1 |= I2C_CR1_STOP; // Stop condition

	return 0;
}

/**
 * [S][EV5][Header][A][EV9][Address][A][EV6][Sr][EV5][Header][A][EV6][Data1][A1][Data2][EV7][A][DataN][EV7_1][NA][P][EV7]
 * S=Start,Sr=Repeated Start,P=Stop,A=Acknowledge,NA=Non-acknowledge
 * EV5: SB=1, cleared by reading SR1 register followed by writing DR register.
 * EV6: ADDR=1, cleared by reading SR1 register followed by reading SR2. In 10-bit master received mode, 
 * this sequence should be followed by writing CR2 with START=1.
 * EV7: RxNE=1 cleared by reading DR register.
 * EV7_1: RxNE=1, cleared by reading DR register, program ACK=0 and STOP request.
 * EV9: ADD10=1, cleared by reading SR1 register followed by writing DR register.
 */
uint8_t *BME280_read(uint8_t reg, uint8_t *data, uint16_t len)
{
	__IO int t;

	/**
	 * Sensor reading described on BME280.
	 * Reading any register from the device,
	 * register address must be sent in write mode (1110 11X0).
	 * After this the slave is addressed in read mode (RW=1)
	 * at address 1110 11X1 until NOACKM and stop condition is sent.
	 */

	//	I2C1->CR1 |= I2C_CR1_ACK;
	I2C1->CR1 &= ~I2C_CR1_POS;	// Acknowledge clear p.682
	I2C1->CR1 |= I2C_CR1_START; // Generate start p.694

	// Wait for EV5
	// while (!(I2C1->SR1 & 1)); // Wait until start condition generated
	while (!(I2C1->SR1 & I2C_SR1_SB))
		; // Wait until start condition generated

	I2C1->DR = bme280_i2c.address << 1; // Last bit set (receiver mode)

	// Wait for EV6
	// while (!(I2C1->SR1 & 2)); // Wait until end of address transmission p.690
	while (!(I2C1->SR1 & I2C_SR1_ADDR))
		; // Wait until end of address transmission p.690

	t = I2C1->SR1 | I2C1->SR2; // Reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag p691
	// while (!(I2C1->SR1 & (1 << 7))); // Wait until data register empty p.689
	while (!(I2C1->SR1 & I2C_SR1_TXE))
		; // Wait until data register empty p.689

	I2C1->DR = reg; // Send command
	while (!(I2C1->SR1 & I2C_SR1_TXE))
		; //wait until data register empty p.689

	I2C1->CR1 |= 0x100; // Generate repeated start p.694
	// while (!(I2C1->SR1 & 1)); // Wait until start condition generated
	while (!(I2C1->SR1 & I2C_SR1_SB))
		; // Wait until start condition generated

	I2C1->DR = bme280_i2c.address << 1 | 1; // Transmit slave address
	while (!(I2C1->SR1 & I2C_SR1_ADDR))
		; // Wait until end of address transmission p.690

	(void)I2C1->SR2;
	I2C1->CR1 |= I2C_SR1_AF; // x400

	// uint8_t *data = data // bme280_i2c.rxbuffer;
	while (len > 0)
	{
		while (!(I2C1->SR1 & I2C_SR1_RXNE))
			;
		*data++ = I2C1->DR;
		len--;
	}

	I2C1->CR1 |= I2C_CR1_STOP; // Stop condition.
	I2C1->CR1 &= ~I2C_CR1_ACK; // Disable Ack.

	return data;
}


uint8_t BME280_read_u8(uint8_t reg)
{
	__IO int t;
	uint8_t value;

	/**
	 * Sensor reading described on BME280.
	 * Reading any register from the device,
	 * register address must be sent in write mode (1110 11X0).
	 * After this the slave is addressed in read mode (RW=1)
	 * at address 1110 11X1 until NOACKM and stop condition is sent.
	 */

	//	I2C1->CR1 |= I2C_CR1_ACK;
	I2C1->CR1 &= ~I2C_CR1_POS;	// Acknowledge clear p.682
	I2C1->CR1 |= I2C_CR1_START; // Generate start p.694

	// Wait for EV5
	// while (!(I2C1->SR1 & 1)); // Wait until start condition generated
	while (!(I2C1->SR1 & I2C_SR1_SB))
		; // Wait until start condition generated

	I2C1->DR = bme280_i2c.address << 1; // Last bit set (receiver mode)

	// Wait for EV6
	// while (!(I2C1->SR1 & 2)); // Wait until end of address transmission p.690
	while (!(I2C1->SR1 & I2C_SR1_ADDR))
		; // Wait until end of address transmission p.690

	t = I2C1->SR1 | I2C1->SR2; // Reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag p691
	// while (!(I2C1->SR1 & (1 << 7))); // Wait until data register empty p.689
	while (!(I2C1->SR1 & I2C_SR1_TXE))
		; // Wait until data register empty p.689

	I2C1->DR = reg; // Send command
	while (!(I2C1->SR1 & I2C_SR1_TXE))
		; //wait until data register empty p.689

	I2C1->CR1 |= 0x100; // Generate repeated start p.694
	// while (!(I2C1->SR1 & 1)); // Wait until start condition generated
	while (!(I2C1->SR1 & I2C_SR1_SB))
		; // Wait until start condition generated

	I2C1->DR = bme280_i2c.address << 1 | 1; // Transmit slave address
	while (!(I2C1->SR1 & I2C_SR1_ADDR))
		; // Wait until end of address transmission p.690

	(void)I2C1->SR2;
	I2C1->CR1 |= I2C_SR1_AF; // x400

	// uint8_t *data = data // bme280_i2c.rxbuffer;
	while (len > 0)
	{
		while (!(I2C1->SR1 & I2C_SR1_RXNE))
			;
		value = I2C1->DR;
		len--;
	}

	I2C1->CR1 |= I2C_CR1_STOP; // Stop condition.
	I2C1->CR1 &= ~I2C_CR1_ACK; // Disable Ack.

	return value;
}


uint8_t BME280_write_u8(uint8_t reg, uint8_t value)
{
	BME280_write(reg, &value, sizeof(value));
	return value;
}

uint8_t BME280_read_u8(uint8_t reg)
{
	uint8_t value;
	//	BME280_write_u8(reg, value);
	BME280_read(reg, &value, sizeof(value));
	return value;
}

uint16_t BME280_read_u16(uint8_t reg)
{
	uint16_t value;
	// BME280_read(reg, &value, sizeof(value));

	value = (BME280_read_u8(reg) << 8) | BME280_read_u8(reg);

	return value;
}

int16_t BME280_read_s16(uint8_t reg)
{
	int16_t value;
	value = BME280_read_u16(reg);

	/*
	 value = (BME280_read_u8(reg) << 8) | BME280_read_u8(reg);
	 */

	return value;
}

uint32_t BME280_read_u24(uint8_t reg)
{
	uint32_t value;
	// Can be done without shifting through the values?
	value = BME280_read(reg, &value, sizeof(value));

	return value;
}
