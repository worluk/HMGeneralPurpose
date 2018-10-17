/*
 * led.cpp
 *
 *  Created on: 7 Feb 2018
 *      Author: worluk
 */

#include "led.h"
#include "../../helpers/helper.h"
#include "driverlib.h"



//- -----------------------------------------------------------------------------------------------------------------------
//- status led functions --------------------------------------------------------------------------------------------------
//- -----------------------------------------------------------------------------------------------------------------------

LED::LED(uint16_t pPort, uint16_t pPin) {

    PinDef pin;
    pin.pin = pPin;
    pin.port = pPort;
    io = new GPIO(this, pin, GPIOType::output);
    off();
}

void LED::shortBlink() {
    on();
    delay(150);
    off();
    delay(150);
}

void LED::shortBlink3() {
    shortBlink();
    shortBlink();
    shortBlink();
}

void LED::on() {
    io->setOutputState(GPIOOutputState::high);
}
void LED::off() {
    io->setOutputState(GPIOOutputState::low);                                              // switch led off
}
void LED::toggle() {
    io->toggleOutputState();

}

//void LD::poll() {}
  /* // if ((nTime == 0) || (nTime > millis())) return;                             // nothing to do or wrong time to do

    if (mode == 0) {                                                            // led off
        off();
        nTime = 0;

    } else if (mode == 1) {                                                     // led on
        on();
        nTime = 0;

    } else if (mode == 2) {                                                     // blink slow
        toggle();
    //    nTime = millis() + slowRate;

    } else if (mode == 3) {                                                     // blink fast
        toggle();
   //     nTime = millis() + fastRate;

    } else if (mode == 4) {                                                     // blink short one
        if (!bCnt++) {
            on();
    //        nTime = millis() + fastRate;
        } else set(0);

    } else if (mode == 5) {                                                     // blink short three
        if (bCnt++ >= 5) set(0);
        toggle();
  //      nTime = millis() + fastRate;

    } else if (mode == 6) {                                                     // heartbeat
        if (bCnt > 3) bCnt = 0;
        toggle();
  ///      nTime = millis() + heartBeat[bCnt++];
    }*/

