/*
 * StateMachine.cpp
 *
 *  Created on: Sep 5, 2018
 *      Author: a0406859
 */

#include "StateMachine.h"

bool StateMachine::trigger(Signal* s){

    if(!state) return false;
    auto n= state->trigger(state, s);

    if(n){
        state = n;
        return true;
    }
    return false;
}
