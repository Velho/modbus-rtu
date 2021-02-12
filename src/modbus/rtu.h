/*
 * modbus-com.h
 *
 *  Created on: Feb 8, 2021
 *      Author: Velho
 */

#ifndef RTU_H_
#define RTU_H_

#include <stddef.h>
#include <stdint.h>

#define RTU_SADDR 0B
#define RTU_PACKET_COUNT 8


typedef struct modbus_packet {
	uint8_t rx_buffer[80];
} modbus_packet_t;

typedef struct modbus_rtu_com {
	int temp;
//	uint8_t rx_buffer[256]; // Size ref std.

	modbus_packet_t packets[RTU_PACKET_COUNT];
} modbus_rtu_com_t;

// Initializes the Modbus interface.
modbus_rtu_com_t *modbus_init();

// Writes the requested data to the channel.
void modbus_write(modbus_rtu_com_t *com, uint8_t* nData, size_t szData);
// Reads the data on the channel.
uint8_t *modbus_read(modbus_rtu_com_t *com);

modbus_packet_t *modbus_get_current_packet(modbus_rtu_com_t *com);

#endif /* RTU_H_ */
