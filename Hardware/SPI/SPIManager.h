/*
 * SPIManager.h
 *
 *  Created on: Sep 20, 2018
 *      Author: a0406859
 */

#ifndef SYSTEM_SPI_SPIMANAGER_H_
#define SYSTEM_SPI_SPIMANAGER_H_

#include "System/GPIOManager/GPIO.h"
#include "SPI.h"

class SPIManager
{
public:
    SPIManager();
    virtual ~SPIManager();
    static SPI* getSPI(PinDef, PinDef, PinDef);
};

#endif /* SYSTEM_SPI_SPIMANAGER_H_ */
