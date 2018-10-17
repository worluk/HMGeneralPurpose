/*
 * Timer.cpp
 *
 *  Created on: Sep 24, 2018
 *      Author: a0406859
 */


#include "Timer.h"
#include "SystemTimer.h"



Timer::Timer(decltype(interval) p_interval, TimerFunction fn, bool cyclic) : nextTrigger(p_interval), interval(p_interval), callback(fn), bCyclic(cyclic){

}

Timer::~Timer(){
   // systemTimer.remove(this);
}

void Timer::trigger(){

    callback();
}
