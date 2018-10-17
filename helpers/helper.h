/*
 * helper.h
 *
 *  Created on: 8 Feb 2018
 *      Author: worluk
 */

#ifndef HELPER_H_
#define HELPER_H_

#define byte uint8_t
#define word uint16_t

#define delayMicroseconds(x) __delay_cycles(4L*(uint32_t)x) //with 8MHz
//void delay(uint32_t s);
#define delay(x) delayMicroseconds(1000L*(uint32_t)x)
#define bitRead(val, bit) (val >> (bit))
#define bitSet(val, bit) (val |= (1 << (bit)))


uint16_t freeMemory(void);
uint32_t byteTimeCvt(uint8_t tTime);
uint8_t  int2ByteTimeCvt(uint16_t tTime);
uint32_t intTimeCvt(uint16_t iTime);

#endif /* HELPER_H_ */
