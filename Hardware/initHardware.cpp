/*
 * initHardware.cpp
 *
 *  Created on: 7 Feb 2018
 *      Author: worluk
 */

#include "initHardware.h"
#include "driverlib.h"
#include <string.h>

bool initHardware(){

    //Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    // Configure Pins for LFXIN
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_PJ, GPIO_PIN4 + GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);

    //Set external frequency for XT1
    CS_setExternalClockSource(32768,0);
    //Set DCO frequency to max DCO setting
    CS_setDCOFreq(CS_DCORSEL_1,CS_DCOFSEL_3);
    //Select XT1 as the clock source for ACLK with no frequency divider
    //CS_initClockSignal(CS_ACLK,CS_LFXTCLK_SELECT,CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_ACLK,CS_LFXTCLK_SELECT,CS_CLOCK_DIVIDER_1); //8MHz
    CS_initClockSignal(CS_SMCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_MCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_1); //8MHz
    //Start XT1 with no time out
    CS_turnOnLFXT(CS_LFXT_DRIVE_0);
    CS_turnOnSMCLK();

    /*
    * Disable the GPIO power-on default high-impedance mode to activate
    * previously configured port settings
    */
    PMM_unlockLPM5();

    //Wait for slave to initialize
    __delay_cycles(100);

    return true;
}



