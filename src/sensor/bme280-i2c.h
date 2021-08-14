#ifndef BME280_I2C_H
#define BME280_I2C_H

#include <stddef.h>
#include <stdint.h>

#include "bme280-reg.h"

#define I2C_BUFFER_LEN 32

typedef struct BME280_I2C {
    uint8_t address;
} BME280_I2C_t;

void BME280_I2C_init(void);

/**
 * Sets the BME280 address.
 */
void BME280_set_address(uint8_t);

/**
 * Writes to the given register with the value.
 * \param uint8_t Register to be written.
 * \param uint8_t* Pointer to the data to be written.
 * \param uint16_t Length of the data to be written.
 * \return Length of data written.
 */
size_t BME280_write(uint8_t reg, uint8_t *data, uint16_t len);

/**
 * 
 */
uint8_t BME280_write_u8(uint8_t reg, uint8_t value);

/**
 * Blocking read from the i2c channel.
 *
 * \param uint8_t register.
 * \param uint8_t* pointer to the rx buffer.
 * \param uint16_t size of the rx buffer.
 * \return uint8_t value.
 */
uint8_t *BME280_read(uint8_t reg, uint8_t *data, uint16_t len);

/**
 * Reads unsigned byte value.
 * \param reg
 * \return uint8_t
 */
uint8_t BME280_read_u8(uint8_t reg);

/**
 * \brief 
 * 
 * \param reg 
 * \return uint16_t 
 */
uint16_t BME280_read_u16(uint8_t reg);
int16_t BME280_read_s16(uint8_t reg);

uint32_t BME280_read_u24(uint8_t reg);

/**
 * \brief Reads byte from the i2c bus.
 * \param address
 * \param command
 * \
 */
void I2C1_Read(uint8_t address, uint8_t command, int len, uint8_t *data);

/**
 * Writes len-bytes to the bus (over 2 bytes).
 */
void I2C1_Write(uint8_t address, uint8_t command, int len, uint8_t *data);

#endif
