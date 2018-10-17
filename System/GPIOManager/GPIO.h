/*
 * GPIO.h
 *
 *  Created on: Sep 6, 2018
 *      Author: a0406859
 */

#ifndef SYSTEM_GPIOMANAGER_GPIO_H_
#define SYSTEM_GPIOMANAGER_GPIO_H_

#include <functional>

enum InterruptEdge{
    falling = 0,
    rising = 1,
    both
};

using InterruptCallback = std::function<void(InterruptEdge)>;

enum GPIOType{
    isr = 0,
    input,
    output
};

enum GPIOOutputState{
    low = 0,
    high
};

struct PinDef{
    uint16_t port;
    uint16_t pin;
    InterruptEdge edge = InterruptEdge::falling;
};

class GPIO
{
private:

    void configureISRType();
    void configureOutputType();

    GPIOType type;
    PinDef pin;
    void* owner;
    InterruptCallback isr;
    InterruptEdge edge;

public:
    GPIO(void* owner, PinDef pPin, GPIOType type, decltype(isr) callback = NULL);
    virtual ~GPIO();
    void setISREdge(InterruptEdge);
    auto    getISREdge()    -> decltype(edge)   {return edge;}
    auto    getPin()        -> decltype(pin)    {return pin;}
    auto    getISR()        -> decltype(isr)    {return isr;}

    void    setOutputState(GPIOOutputState);
    void    toggleOutputState();




};

#endif /* SYSTEM_GPIOMANAGER_GPIO_H_ */
