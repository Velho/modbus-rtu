/**
 * sensor.c
 *
 *  Created on: Feb 9, 2021
 *      Author: Velho
 */

#include "bme280-sensor.h"
#include "stm32l1xx.h"

#include <string.h>
#include <math.h>

static sensor_t sensor = {0};

static void i2c_delay_ms(int delay)
{
	int i = 0;
	for (; delay > 0; delay--)
		for (i = 0; i < 2460; i++)
			; //measured with oscilloscope
}

static int is_calibrating(void)
{
	sensor.status = BME280_read_u8(BME280_REGISTER_STATUS);

	return (sensor.status & (1 << 0)) != 0;
}

/**
 * Read the factory set calibration values from the device.
 */
void BME280_read_calibration_values(calib_data_t *calib)
{
	// TODO Implement the missing functions.
	calib->dig_P1 = BME280_read_u16(BME280_REGISTER_P1);
	calib->dig_P2 = BME280_read_s16(BME280_REGISTER_P2);
	calib->dig_P3 = BME280_read_s16(BME280_REGISTER_P3);
	calib->dig_P4 = BME280_read_s16(BME280_REGISTER_P4);
	calib->dig_P5 = BME280_read_s16(BME280_REGISTER_P5);
	calib->dig_P6 = BME280_read_s16(BME280_REGISTER_P6);
	calib->dig_P7 = BME280_read_s16(BME280_REGISTER_P7);
	calib->dig_P8 = BME280_read_s16(BME280_REGISTER_P8);
	calib->dig_P9 = BME280_read_s16(BME280_REGISTER_P9);

	calib->dig_T1 = BME280_read_u16(BME280_REGISTER_T1);
	calib->dig_T2 = BME280_read_s16(BME280_REGISTER_T2);
	calib->dig_T3 = BME280_read_s16(BME280_REGISTER_T3);

	calib->dig_H1 = BME280_read_u8(BME280_REGISTER_H1);
	calib->dig_H2 = BME280_read_s16(BME280_REGISTER_H2);
	calib->dig_H3 = BME280_read_u8(BME280_REGISTER_H3);

	calib->dig_H4 = ((int8_t)BME280_read_u8(BME280_REGISTER_H4) << 4) | (BME280_read_u8(BME280_REGISTER_H4 + 1) & 0xF);
	calib->dig_H5 = ((int8_t)BME280_read_u8(BME280_REGISTER_H5 + 1) << 4) |
					(BME280_read_u8(BME280_REGISTER_H5) >> 4);
	calib->dig_H6 = (int8_t)BME280_read_u8(BME280_REGISTER_H6);
}

int BME280_init()
{
	__disable_irq();
	// Initialize the sensor.
	BME280_set_address(BME280_I2C_ADDRESS);

	// Read the chip id and compare it.
	sensor.chip_id = BME280_read_u8(BME280_REGISTER_CHIP_ID);

	// Read the ID to make sure we have working interface.
	if (sensor.chip_id != 0x60)
		return 1;

	// Reset the device.
	BME280_write_u8(BME280_REGISTER_RESET, 0xB6);
	i2c_delay_ms(5000); // Reset timeout.

	while (is_calibrating())
		i2c_delay_ms(10000);

	BME280_read_calibration_values(&sensor.calib);

	BME280_set_sampling(BME280_NORMAL_MODE, BME280_OSAMPLING_NONE);

	__enable_irq();

	i2c_delay_ms(100);

	return 0;
}

void BME280_set_sampling(sensor_mode_t mode, oversampling_t sample)
{
	BME280_write_u8(BME280_REGISTER_CTRL_MEAS, BME280_SLEEP_MODE);

	BME280_write_u8(BME280_REGISTER_CTRL_HUM, sample);
	BME280_write_u8(BME280_REGISTER_CONF, sample);
	BME280_write_u8(BME280_REGISTER_CTRL_MEAS, sample);
}

float BME280_read_humidity()
{
	BME280_read_temperature(); // Read the temperature to get the t_fine.

	BME280_S32_t adc_h;
	adc_h = BME280_read_s16(BME280_REGISTER_HUM_MSB);

	// If the measurement is disabled.
	if (sensor.adc_H == 0x8000)
		return NAN;

	sensor.adc_H = adc_h;
	BME280_S32_t v_x1_u32r;

	v_x1_u32r = (sensor.t_fine - ((int32_t)76800));
	v_x1_u32r = (((((sensor.adc_H << 14) - (((int32_t)sensor.calib.dig_H4) << 20) -
					(((int32_t)sensor.calib.dig_H5) * v_x1_u32r)) +
				   ((int32_t)16384)) >>
				  15) *
				 (((((((v_x1_u32r * ((int32_t)sensor.calib.dig_H6)) >> 10) *
					  (((v_x1_u32r * ((int32_t)sensor.calib.dig_H3)) >> 11) +
					   ((int32_t)32768))) >>
					 10) +
					((int32_t)2097152)) *
					   ((int32_t)sensor.calib.dig_H2) +
				   8192) >>
				  14));

	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
							   ((int32_t)sensor.calib.dig_H1)) >>
							  4));

	v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
	v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
	float h = (v_x1_u32r >> 12);
	return h / 1024.0;
}

float BME280_read_temperature()
{
	BME280_S32_t var1, var2, t;
	BME280_S32_t adc_T;

	adc_T = BME280_read_u24(BME280_REGISTER_TEMP_MSB);

	if (adc_T == 0x800000)
		return NAN;

	adc_T >>= 4;

	var1 = ((((adc_T >> 3) - ((int32_t)sensor.calib.dig_T1 << 1))) *
			((int32_t)sensor.calib.dig_T2)) >>
		   11;

	var2 = (((((adc_T >> 4) - ((int32_t)sensor.calib.dig_T1)) *
			  ((adc_T >> 4) - ((int32_t)sensor.calib.dig_T1))) >>
			 12) *
			((int32_t)sensor.calib.dig_T3)) >>
		   14;

	sensor.t_fine = var1 + var2;

	t = (sensor.t_fine * 5 + 128) >> 8;
	return t / 100;
}

float BME280_read_pressure()
{
	BME280_S64_t var1, var2, p;

	BME280_read_temperature();

	BME280_S32_t adc_P = BME280_read_u24(BME280_REGISTER_PRESS_MSB);
	if (adc_P == 0x800000)
		return NAN;

	adc_P >>= 4;

	var1 = ((int64_t)sensor.t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)sensor.calib.dig_P6;
	var2 = var2 + ((var1 * (int64_t)sensor.calib.dig_P5) << 17);
	var2 = var2 + (((int64_t)sensor.calib.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)sensor.calib.dig_P3) >> 8) +
		   ((var1 * (int64_t)sensor.calib.dig_P2) << 12);
	var1 =
		(((((int64_t)1) << 47) + var1)) * ((int64_t)sensor.calib.dig_P1) >> 33;

	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}

	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)sensor.calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)sensor.calib.dig_P8) * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (((int64_t)sensor.calib.dig_P7) << 4);

	return (p) / 256;
}

void foo()
{
	BME280_S32_t bme280_val_t;
	BME280_S32_t comp_t;

	// Access the sensor context somehow?
	bme280_val_t = sensor.adc_T;

	// Adding the measurement as ADC value?
	// Calculating the compensation after taking the median?
	// No no no, median does not work for the adc as they does not
	// represent the real value of measured t, u & p.
	// Ooor it wouldnt matter... Only that matters is the
	// values read, median is just the middle of sorted list.

	// BME280_compensate_T_int32(BME280_S32_t adc_T, )
	// comp_t = BME280_compensate_T_int32(bme280.adc_T, &bme280.temp);
	add_measurement(bme280_val_t);
}
