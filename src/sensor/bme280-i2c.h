#ifndef BME280_I2C_H
#define BME280_I2C_H

#include <stdint.h>

#define I2C_BUFFER_LEN	32

typedef struct bme280_i2c {
	int transmitting;

	uint8_t txbuffer[I2C_BUFFER_LEN];
	size_t txindex;
	size_t txlength;
	
	uint8_t rxbuffer[I2C_BUFFER_LEN];
	size_t rxindex;
	size_t rxlength;
} bme280_i2c_t;


// Start the communication ?
void bme280_begin(uint8_t reg);
// End the communication ?
void bme280_end(uint8_t reg);

/**
 * Writes to the given register with the value.
 * \return Length of data written.
 */
size_t bme280_write(uint8_t reg, uint8_t value);

/**
 *
 */
uint8_t bme280_read(uint8_t reg);

#endif
