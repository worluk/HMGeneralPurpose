/*
 * SPI.cpp
 *
 *  Created on: Sep 20, 2018
 *      Author: a0406859
 */

#include <Hardware/SPI/SPI.h>
#include <driverlib.h>

SPI::SPI(PinDef p_miso, PinDef p_mosi, PinDef p_sclk) : miso(p_miso), mosi(p_mosi), sclk(p_sclk){

    // Configure SPI pins
    GPIO_setAsPeripheralModuleFunctionInputPin(sclk.port, sclk.pin,  GPIO_SECONDARY_MODULE_FUNCTION ); //SCLK
    GPIO_setAsPeripheralModuleFunctionInputPin(miso.port, miso.pin,  GPIO_SECONDARY_MODULE_FUNCTION );
    GPIO_setAsPeripheralModuleFunctionInputPin(mosi.port, mosi.pin,  GPIO_SECONDARY_MODULE_FUNCTION );

    //Initialize Master
    EUSCI_B_SPI_initMasterParam param = {0};
    param.selectClockSource = EUSCI_B_SPI_CLOCKSOURCE_SMCLK;
    param.clockSourceFrequency = CS_getSMCLK();
    param.desiredSpiClock = 2000000;                                                 // SPI speed = CLK/4 ??
    param.msbFirst = EUSCI_B_SPI_MSB_FIRST;
    param.clockPhase = EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
    param.clockPolarity = EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;
    param.spiMode = EUSCI_B_SPI_3PIN;
    EUSCI_B_SPI_initMaster(EUSCI_B0_BASE, &param);

    //Enable SPI module
    EUSCI_B_SPI_enable(EUSCI_B0_BASE);

    EUSCI_B_SPI_clearInterrupt(EUSCI_B0_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT);

    // Enable USCI_B0 RX interrupt
    EUSCI_B_SPI_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT);

}

uint8_t SPI::send(uint8_t c){

     while (EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT))
     EUSCI_B_SPI_clearInterrupt(EUSCI_B0_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT);
     EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, c);
     return EUSCI_B_SPI_receiveData(EUSCI_B0_BASE);
}

uint8_t SPI::getMISOState(){
    return GPIO_getInputPinValue(mosi.port, mosi.pin);
}

#pragma vector=USCI_B0_VECTOR
__interrupt void SPIISR(void){

    if(EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT)){
        EUSCI_B_SPI_clearInterrupt(EUSCI_B0_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT);

    }
    else if(EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT)){
        EUSCI_B_SPI_clearInterrupt(EUSCI_B0_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT);

    }
}
