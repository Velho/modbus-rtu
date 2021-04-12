/*
 * modbus-com.c
 *
 *  Created on: Feb 8, 2021
 *      Author: Velho
 */

#include "modbus/rtu.h"
#include "stm32l1xx.h"

#include "crc.h"

#define MODBUS_MAX_APDU_LEN	8

static modbus_rtu_com_t modbus_com = { 0 }; // Allocate the modbus com struct.

/**
 * Calculates the CRC for the given buffer.
 * \param uint8_t * Pointer to beginning of the buffer.
 * \param uint16_t Size of the buffer.
 * \param modbus_frame_t * Pointer to the compare frame.
 * \return int boolean condition if the CRC(modbus) is matching.
 */
static int modbus_crc_verify(uint8_t *buffer, uint16_t size, modbus_frame_t *frame) {
    return CRC16(buffer, size) == frame->crc;
}

/**
 * Constructs the frame from the given buffer.
 * Does the marshaling of frame fields. Due to the endianness
 * just casting the buffer, does not work.
 *
 * \param uint8_t * Pointer to the buffer containing the modbus packet.
 * \param uint16_t Length of the data contained in the buffer.
 * \return int boolean condition if the frame is accepted.
 */
static int modbus_receive_frame(uint8_t *buffer, uint16_t len) {
    modbus_frame_t frame;

    frame.saddr = buffer[0];
    frame.func = buffer[1];
    frame.address = (buffer[2] << 8) | buffer[3];
    frame.quantity = (buffer[4] << 8) | buffer[5];
    // Or start with the CRC, after which assign the other values.
    frame.crc = (buffer[6] << 8) | buffer[7];

    // TODO : Should we set some error flag or fail condition?
    // Frame is verified earlier.
    // Verify the frame address.
    // if (frame.saddr != RTU_SADDR) {
    //     // Mark as finished.
    //     return 0;
    // }

    // Verify the CRC with Slave address.
    if (!modbus_crc_verify(buffer, len - 2, &frame)) {
        return 0;
    }


    // Clear the dirty flag?
    modbus_com.packet.dirty = 0;
    modbus_com.packet.rframe = frame;

    return 1;
}

static void modbus_reset()
{
    modbus_com.packet.size = 0;
    modbus_com.packet.dirty = 0;
}

/**
 * 
 */
static uint8_t modbus_append_data(uint8_t data) {
    modbus_com.packet.buffer[modbus_com.packet.size] = data;
    modbus_com.packet.size++;

    uint8_t current = modbus_com.packet.size % MODBUS_MAX_APDU_LEN;
    if (current >= MODBUS_MAX_APDU_LEN) {
        // Packet not handled.
        modbus_com.packet.dirty = 1;
        return 0;
    }

    // Verify the address. 
    if (current == 1) {
        if (data != RTU_SADDR) {
            // Set error flag and reset the handler.
            modbus_com.err = 1;
            modbus_com.timeout = 1;
            modbus_reset();
            return 1;
        }
    }

    return 0;
}

modbus_rtu_com_t *modbus_init() {
    // Initialize rtu context.

    return &modbus_com;
}

void modbus_write(modbus_rtu_com_t *com, uint8_t* nData, size_t szData) {

}

uint8_t *modbus_read(uint8_t data) {
    // Return the packet from modbus_com struct.

    return modbus_append_data(data);
}
