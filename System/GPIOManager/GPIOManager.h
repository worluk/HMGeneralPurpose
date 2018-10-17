/*
 * GPIOManager.h
 *
 *  Created on: Sep 6, 2018
 *      Author: a0406859
 */

#ifndef SYSTEM_GPIOMANAGER_GPIOMANAGER_H_
#define SYSTEM_GPIOMANAGER_GPIOMANAGER_H_

#include "GPIO.h"
#include <map>

using pinMap    = std::map<uint16_t, GPIO*>;
using portMap   = std::map<uint16_t,  pinMap>;
//std::map<uint16_t,  td::map<uint16_t, GPIO*> >

class GPIOManager
{
public:
    GPIOManager();
    virtual ~GPIOManager();

    bool requestInterrupt(GPIO* gpio);
    void toggleInterruptEdge(PinDef);

    std::map<uint16_t,  std::map<uint16_t, GPIO*> > ios;

};

extern GPIOManager gpiomanager;

#endif /* SYSTEM_GPIOMANAGER_GPIOMANAGER_H_ */
