/*
 * CC1101.cpp
 *
 *  Created on: Sep 21, 2018
 *      Author: a0406859
 */

#include <CC1101/CC1101.h>
#include "../helpers/helper.h"

CC1101::CC1101(){}
CC1101::~CC1101(){}

void CC1101::init(){

    CC::init();

    powerOnSequence();

    reset();

    //now we are in IDLE state

    manualCalibration();

    //wait for SM to be in idle state
    while (isIdle()) {                  // waits until module gets ready
        delayMicroseconds(1);
    }

    setPowerMode(maxPower);

    flushRXBuffer();

    resetRTC();
}

