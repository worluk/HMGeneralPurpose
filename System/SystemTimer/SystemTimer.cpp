/*
 * SystemTimer.cpp
 *
 *  Created on: Sep 6, 2018
 *      Author: a0406859
 */

#include <System/SystemTimer/SystemTimer.h>

#include "driverlib.h"

extern SystemTimer systemTimer;
#define COMPARE_VALUE 8000 //8MHZ --> ISR every 1ms

bool orderTimer(Timer* t1, Timer* t2){
    return t1->nextTrigger < t2->nextTrigger;
}

SystemTimer::SystemTimer(){
    Timer_A_initContinuousModeParam initContParam = {0};
    initContParam.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    initContParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    initContParam.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    initContParam.timerClear = TIMER_A_DO_CLEAR;
    initContParam.startTimer = false;
    Timer_A_initContinuousMode(TIMER_A1_BASE, &initContParam);

    //Initiaze compare mode
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0  );

    Timer_A_initCompareModeParam initCompParam = {0};
    initCompParam.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_0;
    initCompParam.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    initCompParam.compareOutputMode = TIMER_A_OUTPUTMODE_OUTBITVALUE;
    initCompParam.compareValue = COMPARE_VALUE;
    Timer_A_initCompareMode(TIMER_A1_BASE, &initCompParam);

}

bool SystemTimer::init(){
    Timer_A_startCounter( TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);
    return true;

}



bool SystemTimer::addTimer(Timer* t){
    timer.push(t);

    return true;
}

void SystemTimer::remove(Timer* t){
    //TODO
}

/**
 * The class basedISR
 */
void SystemTimer::trigger(){

    counter++;

    auto top = timer.top();
    while(top->nextTrigger <= counter){

        top->trigger(); //queue in idle loop?
        if(top->isCyclic()){
            top->nextTrigger += top->interval;
            timer.push(top);
        }
        timer.pop();
        top = timer.top();
    }

}

//******************************************************************************
//
//This is the TIMER1_A3 interrupt vector service routine.
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(TIMER1_A0_VECTOR)))
#endif
void TIMER1_A0_ISR (void)
{
    uint16_t compVal = Timer_A_getCaptureCompareCount(TIMER_A1_BASE,
            TIMER_A_CAPTURECOMPARE_REGISTER_0)
            + COMPARE_VALUE;

    systemTimer.trigger();
    //Add Offset to CCR0
    Timer_A_setCompareValue(TIMER_A1_BASE,
        TIMER_A_CAPTURECOMPARE_REGISTER_0,
        compVal
        );
}
