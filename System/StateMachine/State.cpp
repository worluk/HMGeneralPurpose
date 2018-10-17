/*
 * State.cpp
 *
 *  Created on: Sep 5, 2018
 *      Author: a0406859
 */

#include "State.h"

void State::addTransition(Signal* signal, State* nextState){

    transitions[signal] = nextState;
}

State* State::trigger(State* prevState, Signal* s){

    std::map<Signal*, State*>::iterator it = transitions.find(s);

    if(it != transitions.end() ){

        State*  nextState = it->second;

        for(auto f : nextState->enterCallbacks) f();

        auto f = nextState->stateEnterCallbacks[prevState];
        if(f) f();

        return nextState;

    }
    return NULL;
}
