/*
 * bme280-reg.h
 *
 *  Created on: Feb 9, 2021
 *      Author: Velho
 */

#ifndef BME280_REG_H_
#define BME280_REG_H_

// Measurement median of 32 different measurements.

#define BME280_REG_T1	0x88
#define BME280_REG_T2	0x8A
#define BME280_REG_T3	0x8C

#define BME280_REG_P1   0x8E
#define BME280_REG_P2   0x90
#define BME280_REG_P3   0x92
#define BME280_REG_P4   0x94
#define BME280_REG_P5   0x96
#define BME280_REG_P6   0x98
#define BME280_REG_P7   0x9A
#define BME280_REG_P8   0x9C
#define BME280_REG_P8   0x9E

#define BME280_REG_H1	0xA1
#define BME280_REG_H2	0xE1
#define BME280_REG_H3	0xE3
#define BME280_REG_H4	0xE4
#define BME280_REG_H5	0xE5
#define BME280_REG_H6	0xE6
#define BME280_REG_H7	0xE7

#define BME280_REG_CHIP_ID  0xD0
#define BME280_REG_RESET    0xE0

#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_STATUS       0xF3
#define BME280_REG_MEAS         0xF4
#define BME280_REG_CONF         0xF5


typedef signed int BME280_S32_T;


#endif /* BME280_REG_H_ */
