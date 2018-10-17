/*
 * regdev.h
 *
 *  Created on: 8 Feb 2018
 *      Author: worluk
 */

#ifndef REGDEV_H_
#define REGDEV_H_

typedef struct s_regDev_ {
    uint8_t                     :7;
    uint8_t intKeyVisib         :1; // reg:0x02, sReg:2.7
    uint8_t                     :6;
    uint8_t ledMode             :2; // reg:0x05, sReg:5.6
    uint8_t pairCentral[3];         // reg:0x0A, sReg:10
    uint8_t lowBatLimitBA;          // reg:0x12, sReg:18
} s_regDev;




#endif /* REGDEV_H_ */
