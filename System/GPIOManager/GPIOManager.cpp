/*
 * GPIOManager.cpp
 *
 *  Created on: Sep 6, 2018
 *      Author: a0406859
 */

#include <System/GPIOManager/GPIOManager.h>
#include "driverlib.h"


GPIOManager gpiomanager;

GPIOManager::GPIOManager(){
}

GPIOManager::~GPIOManager(){
}

bool GPIOManager::requestInterrupt(GPIO* gpio){

    PinDef pin = gpio->getPin();
    ios[pin.port][pin.pin] = gpio;

    return true;
}

void GPIOManager::toggleInterruptEdge(PinDef pin){

    if(pin.edge == InterruptEdge::falling) {
        pin.edge = InterruptEdge::rising;
        GPIO_selectInterruptEdge(pin.port, pin.pin, GPIO_LOW_TO_HIGH_TRANSITION);
    }
    else {
        pin.edge = InterruptEdge::falling;
        GPIO_selectInterruptEdge(pin.port, pin.pin, GPIO_HIGH_TO_LOW_TRANSITION);
    }
}

#pragma vector=PORT1_VECTOR
__interrupt void GPIOPort1Vector(void){
    uint8_t i = P1IFG & P1IE;
    GPIO* gpio = gpiomanager.ios[GPIO_PORT_P1][i];
    if(gpio){
        PinDef pin = gpio->getPin();
        InterruptCallback isrCallback = gpio->getISR();
        if(isrCallback)
            isrCallback((InterruptEdge)(GPIO_getInputPinValue(pin.port, pin.pin)));  //the input value when the ISR is triggered matches the direction of the edge
        if(pin.edge == InterruptEdge::both)
            gpiomanager.toggleInterruptEdge(pin);                                    //if both edges are to be detected, we need to toggle the edge
    }
    GPIO_clearInterrupt(GPIO_PORT_P1, i);
}



#pragma vector=PORT2_VECTOR
__interrupt void GPIOPort2Vector(void){
    uint8_t i = P2IFG & P2IE;
    GPIO* gpio = gpiomanager.ios[GPIO_PORT_P2][i];
    if(gpio){
        PinDef pin = gpio->getPin();
        InterruptCallback isrCallback = gpio->getISR();
        if(isrCallback)
            isrCallback((InterruptEdge)(GPIO_getInputPinValue(pin.port, pin.pin)));  //the input value when the ISR is triggered matches the direction of the edge
        if(pin.edge == InterruptEdge::both)
            gpiomanager.toggleInterruptEdge(pin);                                    //if both edges are to be detected, we need to toggle the edge
    }
    GPIO_clearInterrupt(GPIO_PORT_P2, i);

}

#pragma vector=PORT3_VECTOR
__interrupt void GPIOPort3Vector(void){
    uint8_t i = P3IFG & P3IE;
    GPIO* gpio = gpiomanager.ios[GPIO_PORT_P3][i];
    if(gpio){
        PinDef pin = gpio->getPin();
        InterruptCallback isrCallback = gpio->getISR();
        if(isrCallback)
            isrCallback((InterruptEdge)(GPIO_getInputPinValue(pin.port, pin.pin)));  //the input value when the ISR is triggered matches the direction of the edge
        if(pin.edge == InterruptEdge::both)
            gpiomanager.toggleInterruptEdge(pin);                                    //if both edges are to be detected, we need to toggle the edge
    }
    GPIO_clearInterrupt(GPIO_PORT_P3, i);

}

#pragma vector=PORT4_VECTOR
__interrupt void GPIOPort4Vector(void){
    uint8_t i = P4IFG & P4IE;
    GPIO* gpio = gpiomanager.ios[GPIO_PORT_P4][i];
    if(gpio){
        PinDef pin = gpio->getPin();
        InterruptCallback isrCallback = gpio->getISR();
        if(isrCallback)
            isrCallback((InterruptEdge)(GPIO_getInputPinValue(pin.port, pin.pin)));  //the input value when the ISR is triggered matches the direction of the edge
        if(pin.edge == InterruptEdge::both)
            gpiomanager.toggleInterruptEdge(pin);                                    //if both edges are to be detected, we need to toggle the edge
    }
    GPIO_clearInterrupt(GPIO_PORT_P4, i);

}
