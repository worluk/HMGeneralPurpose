/*
 * Serial.cpp
 *
 *  Created on: Sep 11, 2018
 *      Author: a0406859
 */

#include <Hardware/Serial/Serial.h>
#include "driverlib.h"
#include <string>

const std::string Serial::endl = "\n";

Serial::Serial()
{
    GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1, GPIO_SECONDARY_MODULE_FUNCTION );

    // Configure UART
    EUSCI_A_UART_initParam param = {0};
    param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar = 8;
    param.firstModReg = 10;
    param.secondModReg = 247;
    param.parity = EUSCI_A_UART_NO_PARITY;
    param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode = EUSCI_A_UART_MODE;
    param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

 /*   EUSCI_A_UART_initParam param = {0};
        param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
        param.clockPrescalar = 4;
        param.firstModReg = 5;
        param.secondModReg = 85;
        param.parity = EUSCI_A_UART_NO_PARITY;
        param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
        param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
        param.uartMode = EUSCI_A_UART_MODE;
        param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;
    */
    if (STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param)) {
      return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable USCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);                     // Enable interrupt

}

void Serial::putc(char c){
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, c);
}

Serial& Serial::operator<<(char c){
    putc(c);
    return *this;
}

Serial& Serial::operator<<(std::string s){
    for(char& c : s) {
        putc(c);
    }
    return *this;
}

Serial& Serial::operator<<(unsigned long i){

    (*this) << std::to_string(i);
    return *this;
}

Serial::~Serial(){}

