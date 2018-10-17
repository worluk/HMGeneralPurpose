/*
 * SPI.h
 *
 *  Created on: Sep 20, 2018
 *      Author: a0406859
 */

#ifndef SYSTEM_SPI_SPI_H_
#define SYSTEM_SPI_SPI_H_

#include "System/GPIOManager/GPIO.h"

class SPI
{

private:
    PinDef sclk;
    PinDef miso;
    PinDef mosi;

    GPIO* cs;

public:
    SPI(PinDef p_miso, PinDef p_mosi, PinDef p_sclk);
    virtual ~SPI(){}

    uint8_t send(uint8_t);

    void addCS(decltype(cs) p_cs){cs = p_cs;}
    void deselect(){if(cs){cs->setOutputState(GPIOOutputState::high);}}
    void select()  {if(cs){cs->setOutputState(GPIOOutputState::low) ;}}

    uint8_t getMISOState();
};

#endif /* SYSTEM_SPI_SPI_H_ */
