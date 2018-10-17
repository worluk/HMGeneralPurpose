/*
 * CC.h
 *
 *  Created on: 7 Feb 2018
 *      Author: worluk
 */

#ifndef CC_H_
#define CC_H_

#include <stdint.h>
#include "CCregs.h"
#include "CCStates.h"
#include "Hardware/SPI/SPI.h"


using CC1101PinConfig = struct s_CC1101PinConfig{
    PinDef mosi;
    PinDef miso;
    PinDef cs;
    PinDef sclk;

    PinDef gd0;
    PinDef gd2;
};

enum CCPowerMode{
    lowPower    = 0x03,
    normalPower = 0x50,
    maxPower    = 0xC0

};

using TRX868 = struct s_trx868 {                                                           // TRX868 communication variables
    uint8_t rfState;                                                        // RF state
    uint8_t crc_ok;                                                         // CRC OK for received message
    uint8_t rssi;                                                           // signal strength
    uint8_t lqi;                                                            // link quality
};

using gdFunction = std::function<void(void)>;

class CC {

private:
    GPIO* gd0 = NULL;
    GPIO* gd2 = NULL;
    GPIO* cs  = NULL;

    SPI*  spi = NULL;

    TRX868 trx868;

    gdFunction gd0fn;
    gdFunction gd2fn;

public:
    CC();
    virtual void init();

    void    powerDown(void);                                                   // put CC1101 into power-down state

    bool    sendData(uint8_t *buf, uint8_t burst);                             // send data packet via RF
    uint8_t recvData(uint8_t *buf);                                            // read data packet from RX FIFO
    uint8_t monitorStatus();
    uint8_t detectBurst();

    void setGD0FN(decltype(gd0fn) fn){ gd0fn = fn;}
    void setGD2FN(decltype(gd2fn) fn){ gd2fn = fn;}

    void gd0ISR(){ if(gd0fn) gd0fn();}
    void gd2ISR(){ if(gd2fn) gd2fn();}

    auto getTRX() -> decltype(trx868) {return trx868;}

    bool setPowerMode(CCPowerMode);

    bool reset();
    bool isIdle();

protected:

    bool powerOnSequence();
    bool setDefaultValues();
    bool manualCalibration();
    bool flushRXBuffer();
    bool resetRTC();



private:


    // TRX868 communication functions
    uint8_t readReg   (uint8_t regAddr,  uint8_t  regType             );           // read CC1101 register via SPI

    void    cmdStrobe (uint8_t cmd                                    );           // send command strobe to the CC1101 IC via SPI
    void    readBurst (uint8_t* buf,     uint8_t  regAddr, uint8_t len);           // read burst data from CC1101 via SPI
    void    writeBurst(uint8_t  regAddr, uint8_t* buf,     uint8_t len);           // write multiple registers into the CC1101 IC via SPI
    void    writeReg  (uint8_t  regAddr, uint8_t  val                 );           // write single register into the CC1101 IC via SPI
};





#endif /* CC_H_ */
