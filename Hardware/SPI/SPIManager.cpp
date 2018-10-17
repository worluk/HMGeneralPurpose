/*
 * SPIManager.cpp
 *
 *  Created on: Sep 20, 2018
 *      Author: a0406859
 */

#include <Hardware/SPI/SPIManager.h>


SPI* SPIManager::getSPI(PinDef miso, PinDef mosi, PinDef sclk){
    return new SPI(miso, mosi, sclk);
}
