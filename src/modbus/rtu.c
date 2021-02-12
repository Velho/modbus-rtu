/*
 * modbus-com.c
 *
 *  Created on: Feb 8, 2021
 *      Author: Velho
 */

#include "modbus/rtu.h"
#include "stm32l1xx.h"

static modbus_rtu_com_t modbus_com = { 0 };

modbus_rtu_com_t *modbus_init() {
	// Initialize rtu context.
}

void modbus_write(modbus_rtu_com_t *com, uint8_t* nData, size_t szData) {

}

uint8_t *modbus_read(modbus_rtu_com_t *com) {


	return NULL;
}
