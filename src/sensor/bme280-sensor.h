/*
 * sensor.h
 *
 *  Created on: Feb 9, 2021
 *      Author: Velho
 */

#ifndef BME280_SENSOR_H_
#define BME280_SENSOR_H_


typedef struct config {

	uint8_t p_mode;
	uint8_t t_mode;
	uint8_t h_mode;

	uint8_t filter;

} config_t;

typedef enum operating_mode {
	BME280_OSAMPLE_1 = 1,
	BME280_OSAMPLE_2,
	BME280_OSAMPLE_3,
	BME280_OSAMPLE_8,
	BME280_OSAMPLE_16
} op_mode_t;

typedef enum filter_mode {
	BME280_FILTER_OFF,
	BME280_FILTER_2,
	BME280_FILTER_4,
	BME280_FILTER_8,
	BME280_FILTER_16,
} f_mode_t;

// Data readout is done by starting a burst read from
// 0xF7 to 0xFC (temp and pres) or from
// 0xF7 to 0xFE (temp, pres, humid).
// 20 bits per value for press, temp
// unsigned 16 bit format for humid.

// ut, up, uh
// Output compensation


#endif /* BME280_SENSOR_H_ */
