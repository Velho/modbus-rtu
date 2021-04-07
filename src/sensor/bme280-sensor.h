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
#include "bme280-i2c.h"


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


/**
 * Sensor configuration options.
 */
typedef struct sensor_config {
    uint8_t t_sb : 3;   ///< Inactive duration.
    uint8_t filter : 3; ///< Filter settings.
} config_t;

/**
 * Control measurement struct.
 * Oversampling settings
 * 000 - skipped
 * 001 - x1
 * 010 - x2
 * 011 - x4
 * 100 - x8
 * 101 - x16
 */
typedef struct control_measurement {
    BME280_U32_t osrs_t : 3; ///< Temperature Oversampling.
    BME280_U32_t osrs_p : 3; ///< Pressure Oversampling.

    /**
     * 00       - sleep
     * 01 or 10 - forced
     * 11       - normal
     */
    BME280_U32_t mode : 2; ///< Device mode.
} ctrl_meas_t;

/**
 * Control humidity register sets the humidity data acquisition
 * options of the device. Changes to this register only become
 * effective after a write operation to "ctrl_meas".
 */
typedef struct control_humidity {
    BME280_U32_t osrs_h : 3; ///< Humidity Oversampling.
} ctrl_hum_t;
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
} humidity_calib_t;


typedef struct temperature_calib {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
} temp_calib_t;

typedef struct calibration_data {
    pressure_calib_t pres_data;
    humidity_calib_t hum_data;
    temp_calib_t temp_data;
} calib_data_t;

/**
 * Are these three raw structs what they suppose to be or
 * what the data_t structs implement?
 */
typedef struct pressure {
    uint8_t press_msb; ///< Contains the MSB part up[19:12] of the raw pressure measurement output data.
    uint8_t press_lsb; ///< Contains the LSB part up[11:4] of the raw pressure measurement output data
    uint8_t press_xlsb : 4; ///< Contains the XLSB part up[3:0] of the raw pressure measurement output data. Contents depend on temperature resolution. 
} press_sensor_t;

/**
 * The temperature register contains the raw tempereature measurement output data ut[19:0].
 */
typedef struct temperature {
    uint8_t temp_msb; ///<
    uint8_t temp_lsb; ///<
    uint8_t temp_xlsb : 4; ///<
} temp_sensor_t;

typedef struct humidity {
    uint8_t hum_msb; ///<
    uint8_t temp_lsb; ///<
} hum_sensor_t;

typedef struct bme280_sensor_temp {
    BME280_S32_t t_fine;
    calib_data_t *calib;

    BME280_S32_t adc_T;
    temp_sensor_t temp;
} sensor_temp_t;

typedef struct bme280_sensor {
    bme280_i2c_t *if_channel;
    uint8_t chip_id;
    uint8_t status;

    BME280_S32_t t_fine;

    calib_data_t calib;
    
    /* Represent the ADC values. */
    BME280_S32_t adc_H;
    BME280_S32_t adc_T;
    BME280_S32_t adc_P;

    /* Represents the raw sensor data. */
    press_sensor_t press;
    temp_sensor_t temp;
    hum_sensor_t hum;
} sensor_t;

    /* Represenets the calibration data. */
    pressure_calib_t pres_data;
    humidity_calib_t hum_data;
    temp_calib_t temp_data;

} sensor_t;

/**
 * Initializes the global bme280 sensor context.
 * \returns boolean condition if initializaion failed.
 */
int bme280_init();

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
