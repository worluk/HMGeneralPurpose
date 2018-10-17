/*
 * HomeMatic.h
 *
 *  Created on: Oct 2, 2018
 *      Author: a0406859
 */

#ifndef HOMEMATIC_HOMEMATIC_H_
#define HOMEMATIC_HOMEMATIC_H_

#include "System/GPIOManager/GPIO.h"
#include <CC1101/CC1101.h>
#include <System/SystemTimer/SystemTimer.h>

class HomeMatic
{

private:
    CC1101 cc;
    Timer* sensorTimer = NULL;


public:
    HomeMatic();
    void init();
    virtual ~HomeMatic();

    void gd0fn();
    void gd2fn();
};

#endif /* HOMEMATIC_HOMEMATIC_H_ */
