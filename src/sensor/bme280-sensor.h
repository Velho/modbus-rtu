/*
 * sensor.h
 *
 *  Created on: Feb 9, 2021
 *      Author: Velho
 */

#ifndef BME280_SENSOR_H_
#define BME280_SENSOR_H_

#include <stdint.h>

#include "bme280-reg.h"

typedef enum oversampling_settings {
    BME280_OSAMPLING_NONE = 0b000,
    BME280_OSAMPLING_X1 = 0b001,
    BME280_OSAMPLING_X2 = 0b010,
    BME280_OSAMPLING_X4 = 0b011,
    BME280_OSAMPLING_X8 = 0b100,
    BME280_OSAMPLING_16 = 0b101
} oversampling_t;

typedef enum filter_mode {
    BME280_FILTER_OFF = 0b000,
    BME280_FILTER_X2 = 0b001,
    BME280_FILTER_X4 = 0b010,
    BME280_FILTER_X8 = 0b011,
    BME280_FILTER_X16 = 0b100,
} filter_t;

typedef enum sensor_mode {
    BME280_SLEEP_MODE = 0b00,
    BME280_FORCED_MODE = 0b01,
    BME280_NORMAL_MODE = 0b11
} sensor_mode_t;

typedef enum standby_mode {
    BME280_STANDBY_MS_0_5 = 0b000,
    BME280_STANDBY_MS_10 = 0b110,
    BME280_STANDBY_MS_20 = 0b111,
    BME280_STANDBY_MS_62_5 = 0b001,
    BME280_STANDBY_MS_125 = 0b010,
    BME280_STANDBY_MS_250 = 0b011,
    BME280_STANDBY_MS_500 = 0b100,
    BME280_STANDBY_MS_1000 = 0b101
} standby_mode_t;


typedef struct config {
    uint8_t p_mode;
    uint8_t t_mode;
    uint8_t h_mode;

    uint8_t filter;
} config_t;

/**
 * The “config” register sets the rate, filter and interface options of the device. Writes to the “config” 
 * register in normal mode may be ignored. In sleep mode writes are not ignored.
*/
struct bme280_config {
    uint8_t t_sb : 2;   ///< Controls inactive duration tstandby in normal mode.
    uint8_t filter : 2; ///< Controls the time constant of the IIR filter.
};

/**
 * Control measurement struct.
 */
typedef struct control_measurement {
    // inactive duration => in normal mode.
    BME280_U32_t filter : 3; ///< Filter Settings.

    BME280_U32_t osrs_t : 3; ///< Temperature Oversampling.
    BME280_U32_t osrs_p : 3; ///< Pressure Oversampling.
    BME280_U32_t osrs_h : 3; ///< Humidity Oversampling.

    BME280_U32_t devmode : 2; ///< Device mode.
} ctrl_meas_t;

/**
 * Calibration structures.
 */
typedef struct pressure_calib {
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} pressure_calib_t;

typedef struct humidity_calib {
    uint8_t dig_H1;
    uint16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
} humidity_data_t;


typedef struct temperature_calib {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
} temp_calib_t;

typedef struct calibration_data {
    /* TODO : Seperate these into their own structs? */
    pressure_calib_t pres_data;
    humidity_data_t hum_data;
    temp_calib_t temp_data;

    /* Represent the ADC values. */
    BME280_S32_t adc_H;
    BME280_S32_t adc_T;
    BME280_S32_t adc_P;
} calib_data_t;

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
