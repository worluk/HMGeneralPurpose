/*
 * SystemTimer.h
 *
 *  Created on: Sep 6, 2018
 *      Author: a0406859
 */

#ifndef SYSTEM_SYSTEMTIMER_SYSTEMTIMER_H_
#define SYSTEM_SYSTEMTIMER_SYSTEMTIMER_H_

#include <map>
#include <vector>
#include <queue>
#include <functional>
#include "Timer.h"

struct compareTimer : public std::binary_function<Timer*, Timer*, bool>{
    bool operator()(const Timer* lhs, const Timer* rhs) const    {
        return lhs->nextTrigger < rhs->nextTrigger;
    }
};

class SystemTimer
{

private:
    uint32_t counter = 0;

    std::priority_queue<Timer*, std::vector<Timer*>, compareTimer> timer;

public:
    SystemTimer();
    bool addTimer(Timer* t);
    void remove(Timer* t);
    auto getCounter() -> decltype(counter){return counter;}
    bool init();

    void trigger();




};

extern SystemTimer systemTimer;

#endif /* SYSTEM_SYSTEMTIMER_SYSTEMTIMER_H_ */
