/*
 * timeHelper.cpp
 *
 *  Created on: 8 Feb 2018
 *      Author: worluk
 */

#include "timeHelper.h"


uint32_t byteTimeCvt(uint8_t tTime) {
    const uint16_t c[8] = {1,10,50,100,600,3000,6000,36000};
    return (uint32_t)(tTime & 0x1f)*c[tTime >> 5]*100;
}
uint32_t intTimeCvt(uint16_t iTime) {
    if (iTime == 0) return 0;

    uint8_t tByte;
    if ((iTime & 0x1F) != 0) {
        tByte = 2;
        for (uint8_t i = 1; i < (iTime & 0x1F); i++) tByte *= 2;
    } else tByte = 1;

    return (uint32_t)tByte*(iTime>>5)*100;
}
