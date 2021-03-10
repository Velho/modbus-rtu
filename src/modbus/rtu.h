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

#define RTU_SADDR				0x0B
#define RTU_PACKET_COUNT		8
#define RTU_APDU_MAX_BUF_LEN 	24

typedef struct modbus_frame {
    uint8_t saddr;
    uint8_t func;
    uint16_t address;
    uint16_t quantity;
    uint16_t crc;
} modbus_frame_t;

//
// 1. Moving the received bytes from interrupt to the modbus state struct.
// 2. Modbus frame handling can happen in the main loop
// 2.1. If dirty is set, handle the packet.
// 2.2. Construct the frame, check crc and address.
// 2.3. Mark the frame dirty from false to true and increase the packet count.
// 

/**
 * Manages the current packet.
 */
typedef struct modbus_packet {
    uint16_t size; /**< Size of the buffer. */
    // Work with this? Buffer to smol as sizeof frame != sizeof buffer;
    uint8_t buffer[RTU_APDU_MAX_BUF_LEN]; /**< Buffer used to store the raw data from serial com. */

    // 
	// 
    // 
    // Have we handled the last received packet.
    // Should this be handled under the packet rather than
    // here in the com management state struct?
    //
    int dirty; /**< Boolean used to mark if the packet needs processing. */
    modbus_frame_t rframe; /**< */
} modbus_packet_t;

/**
 * Modbus RTU communication structure. Manages the main state of the communication.
 */
typedef struct modbus_rtu_com {
    int current; 			/**< Current index of the packet. TODO : Remove if packets are not stored in array. */
    modbus_packet_t packet; /**< Latest packet assigned after constructing it. */
} modbus_rtu_com_t;

// Initializes the Modbus interface.
modbus_rtu_com_t *modbus_init();

// Writes the requested data to the channel.
void modbus_write(modbus_rtu_com_t *com, uint8_t* nData, size_t szData);

/**
 * Appends the data passed to the modbus packet buffer.
 * 
 * \param uint8_t Data read from the channel.
 * \return Pointer to the newly allocated data under the buffer.
 */
uint8_t *modbus_read(uint8_t); // (modbus_rtu_com_t *com) param to the main state?

modbus_packet_t *modbus_get_current_packet(modbus_rtu_com_t *com);

#endif /* RTU_H_ */
