/*
 * sensor.h
 *
 *  Created on: Feb 9, 2021
 *      Author: Velho
 */

#ifndef BME280_SENSOR_H_
#define BME280_SENSOR_H_


#include "bme280-reg.h"

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



/**
 * Returns temperature in DegC, resolution is 0.01 DegC.
 * t_fine carries fine temperature as global value.
 * \return Output value of “5123” equals 51.23 DegC.
 */
BME280_S32_t t_fine;
BME280_S32_t BME280_compensate_T_int32(BME280_S32_t adc_T);

/** 
 * Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
 * \return Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
 */
BME280_U32_t BME280_compensate_P_int64(BME280_S32_t adc_P);

/**
 * Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
 * \return Output value of “47445” represents 47445/1024 = 46.333 %RH
 */
BME280_U32_t bme280_compensate_H_int32(BME280_S32_t adc_H);

#endif /* BME280_SENSOR_H_ */
