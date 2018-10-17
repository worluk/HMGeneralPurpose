/*
 * ButtonKey.cpp
 *
 *  Created on: 7 Feb 2018
 *      Author: worluk
 */

#include <functional>

#include "Hardware/Button/Button.h"
#include "driverlib.h"
#include "gpio.h"


Button::~Button(){

    delete buttonDown;
    delete buttonUp;
    delete longPressTimeout;
    delete shortPressTimeout;
    delete ret;

    delete idleState;
    delete downState;
    delete waitForAction;
    delete secondDownState;
    delete timeoutState;
    delete longWaitForAction;

    if(io) delete io;
}

//allow 3 actions: short button press, long button press, double short press
Button::Button(uint16_t p_port, uint8_t p_pin){

    pin.port = p_port;
    pin.pin = p_pin;
    pin.edge = InterruptEdge::both;

    buttonDown        = new Signal();
    buttonUp          = new Signal();
    longPressTimeout  = new Signal();
    shortPressTimeout = new Signal();
    ret               = new Signal();

    idleState         = new State();
    downState         = new State();
    waitForAction     = new State();
    secondDownState   = new State();
    timeoutState      = new State();
    longWaitForAction = new State();

    idleState       ->addTransition(buttonDown,         downState);
    downState       ->addTransition(buttonUp,           idleState);// waitForAction);
    downState       ->addTransition(longPressTimeout,   idleState);
    waitForAction   ->addTransition(buttonDown,         secondDownState);
    waitForAction   ->addTransition(shortPressTimeout,  idleState);
    secondDownState ->addTransition(buttonUp,           idleState);
    secondDownState ->addTransition(longPressTimeout,   timeoutState);
    timeoutState    ->addTransition(ret,                idleState);

}

void Button::init(){

    InterruptCallback eventFn = std::bind(&Button::event, this, std::placeholders::_1);
    io = new GPIO(this, pin, GPIOType::isr, eventFn);

    idleState->addOnEnter(downState,        std::bind(&Button::shortPress,  this) );
    //idleState->addOnEnter(downState,        std::bind(&Button::longPress,   this) );
    idleState->addOnEnter(secondDownState,  std::bind(&Button::doublePress, this) );
    //idleState->addOnEnter(timeoutState); //TODO solve trigger in trigger function -> add queue

    sm.start(idleState);
}

void Button::event(InterruptEdge edge){

    if(edge == InterruptEdge::rising)
        sm.trigger(buttonUp);
    else
        sm.trigger(buttonDown);

}
