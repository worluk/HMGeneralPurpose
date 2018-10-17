/*
 * StateMachine.h
 *
 *  Created on: Sep 5, 2018
 *      Author: a0406859
 */

#ifndef HELPERS_STATEMACHINE_STATEMACHINE_H_
#define HELPERS_STATEMACHINE_STATEMACHINE_H_

#include <vector>

#include "State.h"
#include "Signal.h"

class StateMachine
{

private:
    State* state = NULL;

public:
    StateMachine(){}
    virtual ~StateMachine(){}

    void start(decltype(state) s)           { state = s; }

    auto currentState() -> decltype(state)  { return state; }
    bool trigger(Signal* s);


};

#endif /* HELPERS_STATEMACHINE_STATEMACHINE_H_ */
