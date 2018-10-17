/*
 * Timer.h
 *
 *  Created on: Sep 24, 2018
 *      Author: a0406859
 */

#ifndef SYSTEM_SYSTEMTIMER_TIMER_H_
#define SYSTEM_SYSTEMTIMER_TIMER_H_

#include <functional>

using TimerFunction = std::function<void(void)>;

class Timer{

private:
    bool bRunning       = true;

    bool bCyclic        = true;
    TimerFunction callback;

public:

    uint32_t nextTrigger;
    uint32_t interval   = 0;

    Timer(decltype(interval), TimerFunction, bool cyclic = true);
    ~Timer();
    void trigger();

    auto getInterval() -> decltype(interval){return interval;}
    auto isCyclic() -> decltype(bCyclic){return bCyclic;}

    bool isRunning(){return bRunning;}
    bool start(){bRunning = true; return true;}
    bool stop(){bRunning = false; return true;}



} ;



#endif /* SYSTEM_SYSTEMTIMER_TIMER_H_ */
