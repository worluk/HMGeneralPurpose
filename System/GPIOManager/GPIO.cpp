/*
 * GPIO.cpp
 *
 *  Created on: Sep 6, 2018
 *      Author: a0406859
 */

#include <System/GPIOManager/GPIO.h>
#include "GPIOManager.h"
#include "driverlib.h"

GPIO::GPIO(void* owner, PinDef pPin, GPIOType p_type, std::function<void(InterruptEdge)> callback ) : pin(pPin), type(p_type), isr(callback)
{
   gpiomanager.requestInterrupt(this); //

    //configure
   if(type == GPIOType::isr)
       configureISRType();
   else if(type == GPIOType::output)
       configureOutputType();

}

GPIO::~GPIO(){
}

void GPIO::setOutputState(GPIOOutputState state){
    if(state == high)
        GPIO_setOutputHighOnPin(pin.port, pin.pin);
    else
        GPIO_setOutputLowOnPin(pin.port, pin.pin);
}

void GPIO::toggleOutputState(){
    GPIO_toggleOutputOnPin(pin.port, pin.pin);
}

void GPIO::configureOutputType(){
    GPIO_setAsOutputPin(pin.port, pin.pin);
}



void GPIO::configureISRType(){

    GPIO_setAsInputPin(pin.port, pin.pin);
    GPIO_selectInterruptEdge(pin.port, pin.pin, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(pin.port, pin.pin);
    GPIO_enableInterrupt(pin.port, pin.pin);

}


