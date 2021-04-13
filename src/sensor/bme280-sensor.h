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

/**
 * Oversampling options for humidity, pressure, temperature measurement.
 * With the oversampling is used to reduce the noise and the data resolution
 * can depend on the IIR filter.
 */
typedef enum oversampling_settings
{
    BME280_OSAMPLING_NONE = 0b000,
    BME280_OSAMPLING_X1 = 0b001,
    BME280_OSAMPLING_X2 = 0b010,
    BME280_OSAMPLING_X4 = 0b011,
    BME280_OSAMPLING_X8 = 0b100,
    BME280_OSAMPLING_16 = 0b101
} oversampling_t;

/**
 * IIR Filter settings, coefficient.
 * Humidity value inside the sensor does not fluctuate rapidly and does not require low pass filtering.
 * However the environmental pressure is subject to many short-term changes, caused e.g. by slamming of a door
 * or a window or wind blogin into the sensor.
 */
typedef enum filter_mode
{
    BME280_FILTER_OFF = 0b000,
    BME280_FILTER_X2 = 0b001,
    BME280_FILTER_X4 = 0b010,
    BME280_FILTER_X8 = 0b011,
    BME280_FILTER_X16 = 0b100,
} filter_t;

/**
 * BME280 offers three sensor modes, sleep, forced and normal.
 * - Sleep mode: no operation, all registers accessible, lowest power, selected after startup.
 * - Forced mode: perform one measurement, store results and return to sleep mode.
 * - Normal mode:perpetual cycling of measurements and inactive periods.
 */
typedef enum sensor_mode
{
    BME280_SLEEP_MODE = 0b00,
    BME280_FORCED_MODE = 0b01,
    BME280_NORMAL_MODE = 0b11
} sensor_mode_t;

/**
 * Controls inactive duration Tstandby in normal mode.
 * Normal mode comprises an automated perpetual cycling between an
 * (active) measurement period and an (inactive) standby period.
 * Where Cycle Time equals Tmeasure + Tstandby.
 */
typedef enum standby_mode
{
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
typedef struct sensor_config
{
    uint8_t t_sb : 3;   ///< Inactive duration.
    uint8_t filter : 3; ///< Filter settings.
} config_t;

/**
 * Control humidity register sets the humidity data acquisition
 * options of the device. Changes to this register only become
 * effective after a write operation to "ctrl_meas". 0xF2
 */
typedef struct control_humidity
{
    BME280_U32_t osrs_h : 3; ///< Humidity Oversampling.
} ctrl_hum_t;

/**
 * Control measurement struct, 0xF4.
 * Oversampling settings
 *  000 - skipped
 *  001 - x1
 *  010 - x2
 *  011 - x4
 *  100 - x8
 *  101 - x16
 * Different sensor modes
 *  00       - sleep
 *  01 or 10 - forced
 *  11       - normal
 */
typedef struct control_measurement
{
    BME280_U32_t osrs_t : 3; ///< Temperature Oversampling.
    BME280_U32_t osrs_p : 3; ///< Pressure Oversampling.

    ctrl_hum_t osrs_h; ///< Humidity oversampling type.

    BME280_U32_t mode : 2; ///< Device mode.
} ctrl_meas_t;

/**
 * Calibration data
 */
typedef struct calibration_data
{
    uint16_t dig_P1; ///< Pressure compensation.
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;

    uint8_t dig_H1; ///< Humididty compensation.
    uint16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;

    uint16_t dig_T1; ///< Temperature compensation.
    int16_t dig_T2;
    int16_t dig_T3;
} calib_data_t;

/**
 * Are these three raw structs what they suppose to be or
 * what the data_t structs implement?
 */
typedef struct pressure
{
    uint8_t press_msb;      ///< Contains the MSB part up[19:12] of the raw pressure measurement output data.
    uint8_t press_lsb;      ///< Contains the LSB part up[11:4] of the raw pressure measurement output data
    uint8_t press_xlsb : 4; ///< Contains the XLSB part up[3:0] of the raw pressure measurement output data. Contents depend on temperature resolution.
} press_sensor_t;

/**
 * The temperature register contains the raw tempereature measurement output data ut[19:0].
 */
typedef struct temperature
{
    uint8_t temp_msb;      ///<
    uint8_t temp_lsb;      ///<
    uint8_t temp_xlsb : 4; ///<
} temp_sensor_t;

typedef struct humidity
{
    uint8_t hum_msb;  ///<
    uint8_t temp_lsb; ///<
} hum_sensor_t;

/**
 *
 */
typedef struct bme280_sensor_temp
{
    BME280_S32_t t_fine;
    calib_data_t *calib;

    BME280_S32_t adc_T;
    temp_sensor_t temp;
} sensor_temp_t;

/**
 * BME280 context structure.
 * Includes the ADC values as well as the parameter
 * values read from the sensor.
 */
typedef struct bme280_sensor
{
    uint8_t chip_id;
    uint8_t status;

    BME280_S32_t t_fine;

    calib_data_t calib;
    ctrl_meas_t ctrl_meas;

    /* Represent the ADC values. */
    BME280_S32_t adc_H;
    BME280_S32_t adc_T;
    BME280_S32_t adc_P;

    /* Represents the raw sensor data. */
    press_sensor_t press;
    temp_sensor_t temp;
    hum_sensor_t hum;
} sensor_t;

sensor_t read_sensor(/* Specify the device. */);
int add_measurement(BME280_S32_t value);

/**
 * Initializes the global bme280 sensor context.
 * \returns boolean condition if initializaion failed.
 */
int BME280_init();

// Data readout is done by starting a burst read from
// 0xF7 to 0xFC (temp and pres) or from
// 0xF7 to 0xFE (temp, pres, humid).
// 20 bits per value for press, temp
// unsigned 16 bit format for humid.

/**
 * Define Compenstation Functions.
 * Temperature. TODO : Address the calibration data.
 *    BME280_S32_t t_fine; // BME280_S32_t adc_T
 *    BME280_S32_t BME280_compensate_T_int32(sensor_t *sensor)
 *    {
 *       BME280_S32_t var1, var2, T;
 *       var1 = ((((adc_T >> 3) â€“ ((BME280_S32_t)dig_T1 << 1))) * ((BME280_S32_t)dig_T2)) >> 11;
 *       var2 = (((((adc_T>>4) â€“ ((BME280_S32_t)sensor->calib.temp_calib.dig_T1)) * ((adc_T>>4) â€“ ((BME280_S32_t)dig_T1))) >> 12) *
 *       ((BME280_S32_t)dig_T3)) >> 14;
 *       t_fine = var1 + var2;
 *       T = (t_fine * 5 + 128) >> 8;
 *       return T;
 *   }
 */

/**
 * BME280 calibration parameters. BME280 output consists of the ADC output values.
 * Actualy pressure and temperature must be calculated using a set of calibration parameters.
 * \param calib_data_t * Structure to the trimming values.
 */
void BME280_read_calibration_values(calib_data_t *);

/**
 * Oversampling of the different sensors, pressure, temp and humidity.
 * \param sensor_mode_t
 * \param oversampling_t
 */
void BME280_set_sampling(sensor_mode_t, oversampling_t);

/**
 * Reads the humidity.
 */
float BME280_read_humidity();

/**
 * Reads the temperature.
 */
float BME280_read_temperature();

/**
 * Reads the pressure.
 */
float BME280_read_pressure();

#endif /* BME280_SENSOR_H_ */
