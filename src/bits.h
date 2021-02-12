/*
 * bits.h
 *
 *  Created on: Feb 11, 2021
 *      Author: Velho
 */

#ifndef BITS_H_
#define BITS_H_


#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))

#define lowByte(w) ((w) & 0xff)
#define highByte(w) ((w) >> 8)


#endif /* BITS_H_ */
