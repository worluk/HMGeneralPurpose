/*
 * CC.cpp
 *
 *  Created on: 7 Feb 2018
 *      Author: worluk
 */

#include <CC1101/CC.h>
#include "driverlib.h"
#include "../helpers/helper.h"
#include "Hardware/SPI/SPIManager.h"


//- ----------------------------------------------------------------------------------------------------------------------
//- CC1101 communication functions ----------------------------------------------------------------------------------------
//- -----------------------------------------------------------------------------------------------------------------------
    const uint8_t initVal[] = {                                 // define init settings for TRX868
        0x00, 0x2E,         // IOCFG2: tristate                                 // non inverted GDO2, high impedance tri state
        0x01, 0x2E,         // IOCFG1: tristate                                 // low output drive strength, non inverted GD=1, high impedance tri state
        0x02, 0x06,         // IOCFG0: packet CRC ok                            // disable temperature sensor, non inverted GDO0, asserts when a sync word has been sent/received, and de-asserts at the end of the packet. in RX, the pin will also de-assert when a package is discarded due to address or maximum length filtering
        0x03, 0x0D,         // FIFOTHR: TX:9 / RX:56                            // 0 ADC retention, 0 close in RX, TX FIFO = 9 / RX FIFO = 56 byte
        0x04, 0xE9,         // SYNC1                                            // Sync word
        0x05, 0xCA,         // SYNC0
        0x06, 0x3D,         // PKTLEN(x): 61                                    // packet length 61
        0x07, 0x0C,         // PKTCTRL1:                                        // PQT = 0, CRC auto flush = 1, append status = 1, no address check
        0x0B, 0x06,         // FSCTRL1:                                         // frequency synthesizer control
        0x0D, 0x21,         // FREQ2
        0x0E, 0x65,         // FREQ1
        0x0F, 0x6A,         // FREQ0
        0x10, 0xC8,         // MDMCFG4
        0x11, 0x93,         // MDMCFG3
        0x12, 0x03,         // MDMCFG2
        0x15, 0x34,         // DEVIATN
        0x16, 0x01,         // MCSM2
        0x17, 0x30,         // MCSM1: always go into IDLE
        0x18, 0x18,         // MCSM0
        0x19, 0x16,         // FOCCFG
        0x1B, 0x43,         // AGCTRL2
        //0x1E, 0x28,       // ..WOREVT1: tEVENT0 = 50 ms, RX timeout = 390 us
        //0x1F, 0xA0,       // ..WOREVT0:
        //0x20, 0xFB,       // ..WORCTRL: EVENT1 = 3, WOR_RES = 0
        0x21, 0x56,         // FREND1
        0x25, 0x00,
        0x26, 0x11,         // FSCAL0
        0x2D, 0x35,         // TEST1
        0x3E, 0xC3,         // ?
    };

CC::CC(){



}

void CC::init(){
    //define SPI pins
    PinDef miso, mosi, sclk;

    miso.port = GPIO_PORT_P1;
    miso.pin  = GPIO_PIN7;

    mosi.port = GPIO_PORT_P1;
    mosi.pin  = GPIO_PIN6;

    sclk.port = GPIO_PORT_P2;
    sclk.pin  = GPIO_PIN2;

    spi = SPIManager::getSPI(miso, mosi, sclk);

    //define sync IOs
    PinDef gd0pin, gd2pin, cspin;

    gd0pin.port     = GPIO_PORT_P1;
    gd0pin.pin      = GPIO_PIN4;

    gd2pin.port     = GPIO_PORT_P1;
    gd2pin.pin      = GPIO_PIN5;

    cspin.port      = GPIO_PORT_P1;
    cspin.pin       = GPIO_PIN3;

    gd0 = new GPIO(this, gd0pin, GPIOType::isr, std::bind(&CC::gd0fn, this));
    gd2 = new GPIO(this, gd2pin, GPIOType::isr, std::bind(&CC::gd2fn, this));
    cs  = new GPIO(this, cspin,  GPIOType::output);

    spi->addCS(cs);
}


bool CC::flushRXBuffer(){
    cmdStrobe(CC1101_SRX);                                                      // flush the RX buffer
    return true;
}

bool CC::resetRTC(){
    cmdStrobe(CC1101_SWORRST);                                                  // reset real time clock
    return true;
}

bool CC::setPowerMode(CCPowerMode mode){
    writeReg(CC1101_PATABLE, mode);                                      // configure PATABLE
    return true;
}

bool CC::isIdle(){
    return readReg(CC1101_MARCSTAT, CC1101_STATUS) == CCState::idle;
}

bool CC::manualCalibration(){
    cmdStrobe(CC1101_SCAL);                                                     // calibrate frequency synthesizer and turn it off
    return true;
}

bool CC::setDefaultValues(){

    for (uint8_t i=0; i<sizeof(initVal); i += 2) {                              // write init value to TRX868
        writeReg(initVal[i], initVal[i + 1]);
    }
    return true;
}

bool CC::reset(){

    //issue SRES command and wait for MISO to go low again
    cmdStrobe(CC1101_SRES);
    while(spi->getMISOState() == 1);
    //TODO add timeout

    return true;

}

bool CC::powerOnSequence(){

    spi->deselect();
    delayMicroseconds(5);

    //Strobe CS low/high
    spi->select();
    delayMicroseconds(10);
    spi->deselect();

    //hold high for at least 40�s
    delayMicroseconds(41);

    //pull CS low and wait for MISO to go low -> CHIP_RDYn
    spi->select();
    while(spi->getMISOState() == 1);
    //TODO add timeout

    return true;
}

bool CC::sendData(uint8_t *buf, uint8_t burst) {                                // send data packet via RF

    // Going from RX to TX does not work if there was a reception less than 0.5
    // sec ago. Due to CCA? Using IDLE helps to shorten this period(?)
    //ccStrobe(CC1100_SIDLE);
    //uint8_t cnt = 0xff;
    //while(cnt-- && (ccStrobe( CC1100_STX ) & 0x70) != 2)
    //my_delay_us(10);
    cmdStrobe(CC1101_SIDLE);                                                    // go to idle mode
    cmdStrobe(CC1101_SFRX );                                                    // flush RX buffer
    cmdStrobe(CC1101_SFTX );                                                    // flush TX buffer


    if (burst) {                                                                // BURST-bit set?
        cmdStrobe(CC1101_STX  );                                                // send a burst
        delay(360);                                                             // according to ELV, devices get activated every 300ms, so send burst for 360ms
        //Serial << "send burst\n";
    } else {
        delay(1);                                                               // wait a short time to set TX mode
    }

    writeBurst(CC1101_TXFIFO, buf, buf[0]+1);                                   // write in TX FIFO

    cmdStrobe(CC1101_SFRX);                                                     // flush the RX buffer
    cmdStrobe(CC1101_STX);                                                      // send a burst

    for(uint8_t i=0; i< 200;++i) {                                              // after sending out all bytes the chip should go automatically in RX mode
        if( readReg(CC1101_MARCSTAT  , CC1101_STATUS) == CCState::rx)
            break;                                                              //now in RX mode, good
        if( readReg(CC1101_MARCSTAT  , CC1101_STATUS) != CCState::tx) {
            break;                                                              //neither in RX nor TX, probably some error
        }
        delayMicroseconds(10);
    }

    return true;
}

uint8_t CC::recvData(uint8_t *buf) {                                         // read data packet from RX FIFO

    auto rxBytes = readReg(CC1101_RXBYTES, CC1101_STATUS);                   // how many bytes are in the buffer

    if (rxBytes & 0x7F && !(rxBytes & 0x80)) {                                  // any byte waiting to be read and no overflow?
        buf[0] = readReg(CC1101_RXFIFO, CC1101_CONFIG);                         // read data length

        if (buf[0] > CC1101_DATA_LEN)                                           // if packet is too long
            buf[0] = 0;                                                         // discard packet
        else {
            readBurst(&buf[1], CC1101_RXFIFO, buf[0]);                          // read data packet
            readReg(CC1101_RXFIFO, CC1101_CONFIG);                              // read RSSI

            uint8_t val = readReg(CC1101_RXFIFO, CC1101_CONFIG);                // read LQI and CRC_OK
            trx868.lqi = val & 0x7F;
            trx868.crc_ok = bitRead(val, 7);

        }
    } else buf[0] = 0;                                                          // nothing to do, or overflow

    cmdStrobe(CC1101_SFRX);                                                     // flush Rx FIFO
    cmdStrobe(CC1101_SIDLE);                                                    // enter IDLE state
    cmdStrobe(CC1101_SRX);                                                      // back to RX state
    cmdStrobe(CC1101_SWORRST);                                                  // reset real time clock

    return buf[0];                                                              // return the data buffer
}

uint8_t CC::detectBurst(void) {                                                 // wake up CC1101 from power down state
    // 10 7/10 5 in front of the received string; 33 after received string
    // 10 - 00001010 - sync word found
    // 7  - 00000111 - GDO0 = 1, GDO2 = 1
    // 5  - 00000101 - GDO0 = 1, GDO2 = 1
    // 33 - 00100001 - GDO0 = 1, preamble quality reached
    // 96 - 01100000 - burst sent
    // 48 - 00110000 - in receive mode
    //
    // Status byte table:
    //  0 current GDO0 value
    //  1 reserved
    //  2 GDO2
    //  3 sync word found
    //  4 channel is clear
    //  5 preamble quality reached
    //  6 carrier sense
    //  7 CRC ok
    //
    // possible solution for finding a burst is to check for bit 6, carrier sense

    // set RXTX module in receive mode
    spi->select();                                                            // select CC1101
    while (!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT));    // wait until we can send/recv
    spi->deselect();                                                          // deselect CC1101
    cmdStrobe(CC1101_SRX);                                                      // set RX mode again
    delay(3);                                                                   // wait a short time to set RX mode

    return bitRead(monitorStatus(),6);                                    // return the detected signal
}

void CC::powerDown() {                                                   // put CC1101 into power-down state
    cmdStrobe(CC1101_SIDLE);                                                    // coming from RX state, we need to enter the IDLE state first
    cmdStrobe(CC1101_SFRX);
    cmdStrobe(CC1101_SPWD);                                                     // enter power down state
}

uint8_t CC::monitorStatus() {
    return readReg(CC1101_PKTSTATUS, CC1101_STATUS);
}


/**
 * Private communication functions below
 */



void CC::cmdStrobe(uint8_t cmd) {                                               // send command strobe to the CC1101 IC via SPI
    spi->select();                                                            // select CC1101
    spi->send(cmd);                                                              // send strobe command
    spi->deselect();                                                         // deselect CC1101
}

void CC::writeBurst(uint8_t regAddr, uint8_t *buf, uint8_t len) {               // write multiple registers into the CC1101 IC via SPI
    spi->select();                                                            // select CC1101
    spi->send(regAddr | WRITE_BURST);                                             // send register address
    for(uint8_t i=0 ; i<len ; i++) spi->send(buf[i]);                             // send value
    spi->deselect();                                                          // deselect CC1101
}

void CC::readBurst(uint8_t *buf, uint8_t regAddr, uint8_t len) {                // read burst data from CC1101 via SPI
    spi->select();                                                            // select CC1101
    spi->send(regAddr | READ_BURST);                                              // send register address
    for(uint8_t i=0 ; i<len ; i++) buf[i] = spi->send(0x00);                      // read result byte by byte
    spi->deselect();                                                          // deselect CC1101
}

uint8_t CC::readReg(uint8_t regAddr, uint8_t regType) {                         // read CC1101 register via SPI
    spi->select();                                                            // select CC1101
    while(spi->getMISOState() == 1);
    volatile uint8_t val = spi->send(regAddr | regType);                                                 // send register address

    val = spi->send(0x00);
    spi->deselect();                                                          // deselect CC1101
    return val;
}

void CC::writeReg(uint8_t regAddr, uint8_t val) {                               // write single register into the CC1101 IC via SPI
    spi->select();                                                            // select CC1101
    spi->send(regAddr);                                                           // send register address
    spi->send(val);                                                               // send value
    spi->deselect();                                                          // deselect CC1101
}

/**
 * End Private communication functions
 */
