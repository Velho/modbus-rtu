/*
 * bme280-reg.h
 *
 *  Created on: Feb 9, 2021
 *      Author: Velho
 */

#ifndef BME280_REG_H_
#define BME280_REG_H_

// Median of 32 different measurements.
// With or without the internal iir filter.

/**
 * Slave address LSB (GND = 0, Vddio = 1)
 */
#define BME280_I2C_ADDRESS  0x77


#define BME280_REGISTER_T1	0x88
#define BME280_REGISTER_T2	0x8A
#define BME280_REGISTER_T3	0x8C

#define BME280_REGISTER_P1   0x8E
#define BME280_REGISTER_P2   0x90
#define BME280_REGISTER_P3   0x92
#define BME280_REGISTER_P4   0x94
#define BME280_REGISTER_P5   0x96
#define BME280_REGISTER_P6   0x98
#define BME280_REGISTER_P7   0x9A
#define BME280_REGISTER_P8   0x9C
#define BME280_REGISTER_P8   0x9E

#define BME280_REGISTER_H1	0xA1
#define BME280_REGISTER_H2	0xE1
#define BME280_REGISTER_H3	0xE3
#define BME280_REGISTER_H4	0xE4
#define BME280_REGISTER_H5	0xE5
#define BME280_REGISTER_H6	0xE6
#define BME280_REGISTER_H7	0xE7

#define BME280_REGISTER_CHIP_ID  0xD0
#define BME280_REGISTER_RESET    0xE0

#define BME280_REGISTER_HUM_LSB      0xFE
#define BME280_REGISTER_HUM_MSB      0xFD
#define BME280_REGISTER_TEMP_XLSB    0xFC
#define BME280_REGISTER_TEMP_LSB     0xFB
#define BME280_REGISTER_TEMP_MSB     0xFA
#define BME280_REGISTER_PRESS_XLSB   0xF9
#define BME280_REGISTER_PRESS_LSB    0xF8
#define BME280_REGISTER_PRESS_MSB    0xF7

/**
 * \brief osrs_h[2:0] : Controls oversampling of humidity data.
 * Note
 * The “ctrl_hum” register sets the humidity data acquisition options of the device. Changes to 
 * this REGISTERister only become effective after a write operation to “ctrl_meas”.
 */
#define BME280_REGISTER_CONF         0xF5
#define BME280_REGISTER_CTRL_MEAS    0xF4
#define BME280_REGISTER_STATUS       0xF3
#define BME280_REGISTER_CTRL_HUM     0xF2

typedef signed int 		BME280_S32_t;
typedef signed long     BME280_S64_t;
typedef unsigned int 	BME280_U32_t;
typedef unsigned long   BME280_U64_t;

#endif /* BME280_REG_H_ */
