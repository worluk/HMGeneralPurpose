#ifndef BUTTONKEY_H_
#define BUTTONKEY_H_

#include <stdint.h>
#include <functional>

#include "System/StateMachine/StateMachine.h"
#include "System/GPIOManager/GPIOManager.h"

using ButtonCallback = std::function<void(void)>;

class Button {

private:
    GPIO* io                        = NULL;
    PinDef pin;

    StateMachine sm;

    Signal* buttonDown              = NULL;
    Signal* buttonUp                = NULL;
    Signal* longPressTimeout        = NULL;
    Signal* shortPressTimeout       = NULL;
    Signal* ret                     = NULL;

    State* idleState                = NULL;
    State* downState                = NULL;
    State* waitForAction            = NULL;
    State* secondDownState          = NULL;
    State* timeoutState             = NULL;
    State* longWaitForAction        = NULL;

    ButtonCallback shortPressFn     = NULL;
    ButtonCallback doublePressFn    = NULL;
    ButtonCallback longPressFn      = NULL;

public:
    Button(uint16_t port, uint8_t pin);
    ~Button();

    void init();

    void event(InterruptEdge);

    void shortPress()   {if(shortPressFn)   shortPressFn(); }
    void doublePress()  {if(doublePressFn)  doublePressFn();}
    void longPress()    {if(longPressFn)    longPressFn();  }

    void assignShortPressFn (decltype(shortPressFn) fn){shortPressFn = fn;  }
    void assignDoublePressFn(decltype(shortPressFn) fn){doublePressFn = fn; }
    void assignLongPressFn  (decltype(shortPressFn) fn){longPressFn = fn;   }

};


#endif /* BUTTONKEY_H_ */
