/*
 * led.h
 *
 *  Created on: 7 Feb 2018
 *      Author: worluk
 */

#ifndef LED_H_
#define LED_H_

#include <stdint.h>
#include "System/GPIOManager/GPIOManager.h"

class LED{

private:
    GPIO* io                        = NULL;
    PinDef pin;
public:
    LED(uint16_t pPort, uint16_t pPin);
    void shortBlink(void);
    void shortBlink3(void);
    void on();
    void off();
    void toggle(void);

};



#endif /* LED_H_ */
