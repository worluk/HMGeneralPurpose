/*
 * State.h
 *
 *  Created on: Sep 5, 2018
 *      Author: a0406859
 */

#ifndef HELPERS_STATEMACHINE_STATE_H_
#define HELPERS_STATEMACHINE_STATE_H_

#include <string>
#include <map>
#include <vector>
#include <functional>

#include "Signal.h"

using stateCallback = std::function<void(void)>;

class State
{
private:
    std::map    <Signal* , State*>          transitions;
    std::map    <State*  , stateCallback>   stateEnterCallbacks;
    std::vector <stateCallback>             enterCallbacks;

public:
    State(){}
    virtual ~State(){}

    void    addTransition   (Signal* signal, State* nextState);
    State*  trigger         (State* prevState, Signal* s);

    void addOnEnter(stateCallback f)            {enterCallbacks.push_back(f);}
    void addOnEnter(State* s, stateCallback f)  {stateEnterCallbacks[s] = f;}


};

#endif /* HELPERS_STATEMACHINE_STATE_H_ */
