/*
 * crc.h
 *
 *  Created on: Feb 7, 2021
 *      Author: Velho
 */

#ifndef CRC_H_
#define CRC_H_


/**
 * \brief Calculates the CRC for the given array of data.
 * \param char * Pointer to the data.
 * \param short int Length of the given data.
 * \return uint16_t error detecting code.
 */
unsigned short int CRC16(char *nData, unsigned short int wLength);

#endif /* CRC_H_ */
