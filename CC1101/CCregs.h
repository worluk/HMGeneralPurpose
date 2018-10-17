/*
 * CC1101_registers.h
 *
 *  Created on: 8 Feb 2018
 *      Author: worluk
 */

#ifndef CC1101_REGISTERS_H_
#define CC1101_REGISTERS_H_

#define CC1101_DATA_LEN          60

// some register definitions for TRX868 communication
#define READ_SINGLE              0x80
#define READ_BURST               0xC0
#define WRITE_BURST              0x40                                       // type of transfers

#define CC1101_CONFIG            0x80                                       // type of register
#define CC1101_STATUS            0xC0

#define CC1101_PATABLE           0x3E                                       // PATABLE address
#define CC1101_TXFIFO            0x3F                                       // TX FIFO address
#define CC1101_RXFIFO            0x3F                                       // RX FIFO address

#define CC1101_SRES              0x30                                       // reset CC1101 chip
#define CC1101_SFSTXON           0x31                                       // enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). if in RX (with CCA): Go to a wait state where only the synthesizer is running (for quick RX / TX turnaround).
#define CC1101_SXOFF             0x32                                       // turn off crystal oscillator
#define CC1101_SCAL              0x33                                       // calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode without setting manual calibration mode (MCSM0.FS_AUTOCAL=0)
#define CC1101_SRX               0x34                                       // enable RX. perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1
#define CC1101_STX               0x35                                       // in IDLE state: enable TX. perform calibration first if MCSM0.FS_AUTOCAL=1. if in RX state and CCA is enabled: only go to TX if channel is clear
#define CC1101_SIDLE             0x36                                       // exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable
#define CC1101_SWOR              0x38                                       // start automatic RX polling sequence (Wake-on-Radio) as described in Section 19.5 if WORCTRL.RC_PD=0
#define CC1101_SPWD              0x39                                       // enter power down mode when CSn goes high
#define CC1101_SFRX              0x3A                                       // flush the RX FIFO buffer. only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC1101_SFTX              0x3B                                       // flush the TX FIFO buffer. only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC1101_SWORRST           0x3C                                       // reset real time clock to Event1 value
#define CC1101_SNOP              0x3D                                       // no operation. may be used to get access to the chip status byte

#define CC1101_PARTNUM           0x30                                       // status register, chip ID
#define CC1101_VERSION           0x31                                       // chip ID
#define CC1101_FREQEST           0x32                                       // frequency offset estimate from demodulator
#define CC1101_LQI               0x33                                       // demodulator estimate for Link Quality
#define CC1101_RSSI              0x34                                       // received signal strength indication
#define CC1101_MARCSTAT        0x35                                       // main radio control state machine state
#define CC1101_WORTIME1          0x36                                       // high byte of WOR Time
#define CC1101_WORTIME0          0x37                                       // low byte of WOR Time
#define CC1101_PKTSTATUS         0x38                                       // current GDOx status and packet status
#define CC1101_VCO_VC_DAC        0x39                                       // current setting from PLL calibration module
#define CC1101_TXBYTES           0x3A                                       // underflow and number of bytes
#define CC1101_RXBYTES           0x3B                                       // overflow and number of bytes
#define CC1101_RCCTRL1_STATUS    0x3C                                       // last RC oscillator calibration result
#define CC1101_RCCTRL0_STATUS    0x3D                                       // last RC oscillator calibration result




#define PA_LowPower              0x03                                       // PATABLE values
#define PA_Normal                0x50                                       // PATABLE values
#define PA_MaxPower              0xC0




#endif /* CC1101_REGISTERS_H_ */
