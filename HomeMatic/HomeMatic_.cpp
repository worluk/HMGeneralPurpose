/*
 * HomeMatic.cpp
 *
 *  Created on: 7 Feb 2018
 *      Author: worluk
 */

#include <HomeMatic/HomeMatic_.h>
#include "../helpers/helper.h"
#include <string.h>
#include "driverlib.h"
#include "Register.h"

const uint8_t HM::broadCast[] = {0,0,0,0};

#pragma PERSISTENT
uint32_t peerdb[maxChannel][maxPeer] = { {0x013BD621, 0x0129D621, 0x03578520 },
                                         {0x013BD621, 0x0129D621, 0x03578520 },
                                         {0x01563412, 0x01578520, 0x02578520 } };

#pragma PERSISTENT
uint8_t bFirstStart = 1;

#pragma PERSISTENT
s_regs regs = { .ch_0 = {
                         .intKeyVisib       = 0,
                         .pairCentral       = {0x1A, 0xB1, 0x50}
                        },
                .ch_1 = {
                         .list1.dblPress    = 2,
                         .list1.sign        = 0,
                         .list1.longPress   = 4,
                         .peer = {
                                 {
                                 .peerNeedsBurst    = 1,
                                 .expectAES         = 0
                                 },
                                 {
                                 .peerNeedsBurst    = 1,
                                 .expectAES         = 0
                                 },
                                 {
                                 .peerNeedsBurst    = 0,
                                 .expectAES         = 0
                                 }

                         }
                },
                .ch_2.peer = {
                                 {
                                 .peerNeedsBurst    = 1,
                                 .expectAES         = 0
                                 },
                                 {
                                 .peerNeedsBurst    = 1,
                                 .expectAES         = 0
                                 },
                                 {
                                 .peerNeedsBurst    = 0,
                                 .expectAES         = 0
                                 }
                },
                .ch_3.peer = {
                                 {
                                     .shActionType  = 1,
                                     .lgActionType  = 1,
                                     .shSwJtOff     = 3,
                                     .lgSwJtOff     = 3,
                                     .shSwJtOn      = 6,
                                     .lgSwJtOn      = 6,
                                 },{
                                     .shActionType  = 0,
                                     .lgActionType  = 1,
                                     .shSwJtOff     = 3,
                                     .lgSwJtOff     = 3,
                                     .shSwJtOn      = 3,
                                     .lgSwJtOn      = 3,
                                 },{
                                    .shActionType  = 0,
                                    .lgActionType  = 1,
                                    .shSwJtOff     = 6,
                                    .lgSwJtOff     = 6,
                                    .shSwJtOn      = 6,
                                    .lgSwJtOn      = 6,
                                }


                            }
           };




HM::HM(s_jumptable *jtPtr, uint16_t **mcPtr, LED* g_ld) {
    jTblPtr = jtPtr;                                                            // jump table for call back functions
    mcConfPtr = *mcPtr;                                                // store pointer to main channel structure
    ld = g_ld;
}

void HM::init() {


    // starts also the send/receive class for the rf module
  //  GPIO_setAsInputPin(GDO0_PORT, GDO0_PIN); //GDO0
 //   GPIO_selectInterruptEdge(GDO0_PORT, GDO0_PIN, GPIO_HIGH_TO_LOW_TRANSITION);
    ld->shortBlink3();

    cc.init();                                                                  // init the TRX module
    initRegisters();                                                            // init the storage management module
    setPowerMode(0);                                                            // set default power mode of HM device
    delay(100);                                                                 // otherwise we get a problem with serial console
  //  enableIRQ_GDO0();                                                            // attach callback function for GDO0 (INT0)

}

void HM::poll() {                                                               // task scheduler
    if (recv.data[0] > 0) recv_poll();                                          // trace the received string and decide what to do further
    if (send.counter > 0) send_poll();                                          // something to send in the buffer?
    if (conf.act > 0) send_conf_poll();                                         // some config to be send out
    if (pevt.act > 0) send_peer_poll();                                         // send peer events
    power_poll();
  //  ld->poll();
}

void HM::send_out() {
    if (bitRead(send.data[2],5)) send.retries = maxRetries;                     // check for ACK request and set max retries counter
    else send.retries = 1;                                                      // otherwise send only one time

    send.burst = bitRead(send.data[2],4);                                       // burst necessary?

    if (memcmp(&send.data[7], HMID, 3) == 0) {                                  // if the message is addressed to us,
        memcpy(recv.data,send.data,send.data[0]+1);                             // then copy in receive buffer. could be the case while sending from serial console
        }
        send.counter = 1;

        /*
        send.counter = 0;                                                       // no need to fire
    } else {                                                                    // it's not for us, so encode and put in send queue

        send.counter = 1;                                                       // and fire
    }*/
}
void HM::reset(void) {
    bFirstStart = 1;              // clear magic byte in eeprom and step in initRegisters
    initRegisters();                                                            // reload the registers
 //   ld->stop();                                                                  // stop blinking
    ld->shortBlink3();                                                           // blink three times short
}
void HM::setConfigEvent(void) {
    s_jumptable x;
    for (s_jumptable* p = jTblPtr; ; ++p) {                                     // find the call back function
        x.code = p->code;                                        // get back variables, because they are in program memory
        x.spec = p->spec;
        x.fun =  (void (*)(uint8_t, uint8_t*, uint8_t))p->fun;

        if ((x.code == 0xFF) && (x.spec == 0xFF)) {
            x.fun(0,(uint8_t*) &broadCast,0);
            break;                                                              // and jump into
        }
    }
}

void HM::setPowerMode(uint8_t mode) {
    // there are 3 power modes for the TRX868 module
    // TX mode will switched on while something is in the send queue
    // 0 - RX mode enabled by default, take approx 17ma
    // 1 - RX is in burst mode, RX will be switched on every 250ms to check if there is a carrier signal
    //     if yes - RX will stay enabled until timeout is reached, prolongation of timeout via receive function seems not necessary
    //              to be able to receive an ACK, RX mode should be switched on by send function
    //     if no  - RX will go in idle mode and wait for the next carrier sense check
    // 2 - RX is off by default, TX mode is enabled while sending something
    //     configuration mode is required in this setup to be able to receive at least pairing and config request strings
    //     should be realized by a 30 sec timeout function for RX mode
    // as output we need a status indication if TRX868 module is in receive, send or idle mode
    // idle mode is then the indication for power down mode of AVR

//  switch (mode) {
//      case 1:                                                                 // no power savings, RX is in receiving mode
//          powr.mode = 1;                                                      // set power mode
//          set_sleep_mode(SLEEP_MODE_IDLE);                                    // normal power saving
//          break;
//
//      case 2:                                                                 // some power savings, RX is in burst mode
//          powr.mode = 2;                                                      // set power mode
//          powr.parTO = 15000;                                                 // pairing timeout
//          powr.minTO = 2000;                                                  // stay awake for 2 seconds after sending
//          powr.nxtTO = millis() + 250;                                        // check in 250ms for a burst signal
//
//          MCUSR &= ~(1<<WDRF);                                                // clear the reset flag
//          WDTCSR |= (1<<WDCE) | (1<<WDE);                                     // set control register to change enabled and enable the watch dog
//          WDTCSR = 1<<WDP2;                                                   // 250 ms
//          powr.wdTme = 256;                                                   // store the watch dog time for adding in the poll function
//          set_sleep_mode(SLEEP_MODE_PWR_DOWN);                                // max power saving
//          break;
//
//      case 3:                                                                 // most power savings, RX is off beside a special function where RX stay in receive for 30 sec
//          MCUSR &= ~(1<<WDRF);                                                // clear the reset flag
//          WDTCSR |= (1<<WDCE) | (1<<WDE);                                     // set control register to change enabled and enable the watch dog
//          //WDTCSR = 1<<WDP2;                                                 // 250 ms
//          //WDTCSR = 1<<WDP1 | 1<<WDP2;                                       // 1000 ms
//          //WDTCSR = 1<<WDP0 | 1<<WDP1 | 1<<WDP2;                             // 2000 ms
//          WDTCSR = 1<<WDP0 | 1<<WDP3;                                         // 8000 ms
//          powr.wdTme = 8190;                                                  // store the watch dog time for adding in the poll function
//
//      case 4:                                                                 // most power savings, RX is off beside a special function where RX stay in receive for 30 sec
//          powr.mode = mode;                                                   // set power mode
//          powr.parTO = 15000;                                                 // pairing timeout
//          powr.minTO = 1000;                                                  // stay awake for 1 seconds after sending
//          powr.nxtTO = millis() + 4000;                                       // stay 4 seconds awake to finish boot time
//
//          set_sleep_mode(SLEEP_MODE_PWR_DOWN);                                // max power saving
//
//          ld.set(2);                                                          // blink the led to show it is awake
//          break;
//      default:                                                                // no power saving, same as case 0, if user had chosen a wrong power saving mode
//          powr.mode = 0;                                                      // set power mode
//  }
//  powr.state = 1;                                                             // after init of the TRX module it is in RX mode
//  //Serial << "pwr.mode:" << powr.mode << '\n';
}

void HM::stayAwake(uint32_t xMillis) {
//  if (powr.state == 0) cc.detectBurst();                                      // if TRX is in sleep, switch it on
//  powr.state = 1;                                                             // remember TRX state
//  powr.nxtTO = millis() + xMillis;                                            // stay awake for some time by setting next check time

}

// external functions for pairing and communicating with the module
void HM::startPairing(void) {                                                   // send a pairing request to master
    //                               01 02    03                            04 05 06 07
    // 1A 00 A2 00 3F A6 5C 00 00 00 10 80 02 50 53 30 30 30 30 30 30 30 31 9F 04 01 01
    if (powr.mode > 1) stayAwake(powr.parTO);                                   // stay awake for the next 30 seconds
    memcpy(send_payLoad, devParam, 17);                                     // copy details out of register.h
    send_prep(send.mCnt++,0xA2,0x00,regDev.pairCentral,send_payLoad,17);
}

void HM::sendInfoActuatorStatus(uint8_t cnl, uint8_t status, uint8_t flag) {
    if (memcmp(regDev.pairCentral,broadCast,3) == 0) return;                    // not paired, nothing to send

//  "10;p01=06"   => { txt => "INFO_ACTUATOR_STATUS", params => {
//      CHANNEL => "2,2",
//      STATUS  => '4,2',
//      UNKNOWN => "6,2",
//      RSSI    => '08,02,$val=(-1)*(hex($val))' } },
    send_payLoad[0] = 0x06;                                                     // INFO_ACTUATOR_STATUS
    send_payLoad[1] = cnl;                                                      // channel
    send_payLoad[2] = status;                                                   // status
    send_payLoad[3] = flag;                                                     // unknown
    send_payLoad[4] = cc.getTRX().rssi;                                           // RSSI

    // if it is an answer to a CONFIG_STATUS_REQUEST we have to use the same message id as the request
    uint8_t tCnt;
    if ((recv.data[3] == 0x01) && (recv.data[11] == 0x0E)) tCnt = recv_rCnt;
    else tCnt = send.mCnt++;
    send_prep(tCnt,0xA4,0x10,regDev.pairCentral,send_payLoad,5);                // prepare the message
}

void HM::sendACKStatus(uint8_t cnl, uint8_t status, uint8_t douolo) {
    //if (memcmp(regDev.pairCentral,broadCast,3) == 0) return;                  // not paired, nothing to send

//  "02;p01=01"   => { txt => "ACK_STATUS",  params => {
//      CHANNEL        => "02,2",
//      STATUS         => "04,2",
//      DOWN           => '06,02,$val=(hex($val)&0x20)?1:0',
//      UP             => '06,02,$val=(hex($val)&0x10)?1:0',
//      LOWBAT         => '06,02,$val=(hex($val)&0x80)?1:0',
//      RSSI           => '08,02,$val=(-1)*(hex($val))', }},
    send_payLoad[0] = 0x01;                                                     // ACK Status
    send_payLoad[1] = cnl;                                                      // channel
    send_payLoad[2] = status;                                                   // status
    send_payLoad[3] = douolo;                                                   // down, up, low battery
    send_payLoad[4] = cc.getTRX().rssi;                                           // RSSI

    // l> 0E EA 80 02 1F B7 4A 63 19 63 01 01 C8 00 4B
    //send_prep(recv_rCnt,0x80,0x02,regDev.pairCentral,send_payLoad,5); // prepare the message
    send_prep(recv_rCnt,0x80,0x02,recv_reID,send_payLoad,5);                    // prepare the message
}

void HM::sendSensorData(uint32_t energyCounter, uint32_t power, uint16_t current, uint16_t voltage, uint8_t frequency) {
    if (memcmp(regDev.pairCentral,broadCast,3) == 0) return;                    // not paired, nothing to send

        // energy counter 3 Bytes
        // power: 3 Bytes
        // current: 2 Bytes
        // voltage: 2 Bytes
        // frequency: 1 Byte

    send_payLoad[0] = energyCounter >> 16;
    send_payLoad[1] = energyCounter >> 8;
    send_payLoad[2] = energyCounter;
    send_payLoad[3] = power >> 16;
    send_payLoad[4] = power >> 8;
    send_payLoad[5] = power;
        send_payLoad[6] = current >> 8;
        send_payLoad[7] = current;
        send_payLoad[8] = voltage >> 8;
        send_payLoad[9] = voltage;
        send_payLoad[10] = frequency;

    send_prep(send.mCnt++,0x80,0x5E,regDev.pairCentral,send_payLoad,11);                // prepare the message                                                          // short led blink
}

void HM::sendPeerREMOTE(uint8_t button, uint8_t longPress, uint8_t lowBat) {
    // no data needed, because it is a (40)REMOTE EVENT
    // "40"          => { txt => "REMOTE"      , params => {
    //     BUTTON   => '00,2,$val=(hex($val)&0x3F)',
    //     LONG     => '00,2,$val=(hex($val)&0x40)?1:0',
    //     LOWBAT   => '00,2,$val=(hex($val)&0x80)?1:0',
    //     COUNTER  => "02,2", } },
    if (button > maxChannel) return; // channel out of range, do nothing

    if (doesListExist(button,4) == 0) { // check if a list4 exist, otherwise leave
        //Serial << "sendPeerREMOTE failed\n";
        return;
    }

    // set variables in struct and make send_peer_poll active
    pevt.cnl = button;                                                          // peer database channel
    pevt.type = 0x40; // message type
    pevt.mFlg = (uint8_t)((longPress == 1)?0x80:0xA0); // no ACK needed while long key press is send
    pevt.data[0] = button | ((longPress)?1:0) << 6 | lowBat << 7; // construct message
    pevt.data[1] = pevt.mCnt[pevt.cnl-1];
    pevt.len = 2; // 2 bytes payload

    pevt.act = 1; // active, 1 = yes, 0 = no
    ld->shortBlink(); // short led blink

        if (longPress != 1) {
                ++pevt.mCnt[pevt.cnl-1]; // increase event counter except for long press (until long press end)
        }
}

void HM::sendPeerRAW(uint8_t cnl, uint8_t type, uint8_t *data, uint8_t len) {
    // validate the input, and fill the respective variables in the struct
    // message handling is taken from send_peer_poll
    if (cnl > maxChannel) return;                                               // channel out of range, do nothing
    if (pevt.act) return;                                                       // already sending an event, leave
    if (doesListExist(cnl,4) == 0) {                                            // check if a list4 exist, otherwise leave
        //Serial << "sendPeerREMOTE failed\n";
        return;
    }

    // set variables in struct and make send_peer_poll active
    pevt.cnl = cnl;                                                             // peer database channel
    pevt.type = type;                                                           // message type
    pevt.mFlg = 0xA2;

    if (len > 0) {                                                              // copy data if there are some
        memcpy(pevt.data, data, len);                                           // data to send
        pevt.len = len;                                                         // len of data to send
    }
    pevt.act = 1;                                                               // active, 1 = yes, 0 = no
    ld->shortBlink();                                                            // short led blink
}

void HM::send_ACK(void) {
    uint8_t payLoad[] = {0x00};                                                 // ACK
    send_prep(recv_rCnt,0x80,0x02,recv_reID,payLoad,1);
}
void HM::send_NACK(void) {
    uint8_t payLoad[] = {0x80};                                                 // NACK
    send_prep(recv_rCnt,0x80,0x02,recv_reID,payLoad,1);
}

//- private: //------------------------------------------------------------------------------------------------------------
// hardware definition for interrupt handling
void HM::isrGDO0event(void) {
    //disableIRQ_GDO0();                                                           // disable interrupt otherwise we could get some new content while we copy the buffer

    if (hm->cc.recvData(hm->recv.data)) {                                      // is something in the receive string
    hm->hm_dec(hm->recv.data);                                                // decode the content
}
   // enableIRQ_GDO0();                                                            // enable the interrupt again
}

// some polling functions
void HM::recv_poll(void) {                                                      // handles the receive objects

    // do some checkups
    if (memcmp(&recv.data[7], HMID, 3) == 0) recv.forUs = 1;                    // for us
    else recv.forUs = 0;
    if (memcmp(&recv.data[7], broadCast, 3) == 0) recv.bCast = 1;                       // or a broadcast
    else recv.bCast = 0;                                                        // otherwise only a log message

    // is the message from a valid sender (pair or peer), if not then exit - takes ~2ms
    if ((isPairKnown(recv_reID) == 0) && (isPeerKnown(recv_reID) == 0)) {       // check against peers
        recv.data[0] = 0;                                                       // clear receive string
        return;
    }

    // check if it was a repeated message, delete while already received - takes ~2ms
    if (bitRead(recv.data[2],6)) {                                              // check repeated flag
        bitSet(recv.p_data[2],6);                                               // set repeated flag in prev received string
        uint16_t ret = memcmp(recv.p_data,recv.data,recv.data[0]+1);            // compare with already received string

        if (ret == 0) {                                                         // already received
            recv.data[0] = 0;                                                   // therefore ignore
            return;                                                             // and skip
        }

    }
    memcpy(recv.p_data,recv.data,recv.data[0]+1);                               // save received string for next compare

    // decide where to jump in
    if((recv.forUs) && (recv_isMsg) && (recv_msgTp == 0x01)) {                  // message is a config message
        if      (recv_by11 == 0x01) recv_ConfigPeerAdd();                       // 01, 01
        else if (recv_by11 == 0x02) recv_ConfigPeerRemove();                    // 01, 02
        else if (recv_by11 == 0x03) recv_ConfigPeerListReq();                   // 01, 03
        else if (recv_by11 == 0x04) recv_ConfigParamReq();                      // 01, 04
        else if (recv_by11 == 0x05) recv_ConfigStart();                         // 01, 05
        else if (recv_by11 == 0x06) recv_ConfigEnd();                           // 01, 06
        else if (recv_by11 == 0x08) recv_ConfigWriteIndex();                    // 01, 08
        else if (recv_by11 == 0x09) recv_ConfigSerialReq();                     // 01, 09
        else if (recv_by11 == 0x0A) recv_Pair_Serial();                         // 01, 0A
        else if (recv_by11 == 0x0E) recv_ConfigStatusReq();                     // 01, 0E

    }

    // l> 0A 73 80 02 63 19 63 2F B7 4A 00
    if((recv.forUs) && (recv_isMsg) && (recv_msgTp == 0x02)) {                  // message seems to be an ACK
        send.counter = 0;
    }

    if((recv.forUs) && (recv_isMsg) && (recv_msgTp == 0x11)) {
        recv_PairEvent();
    }

        if((recv.forUs) && (recv.data[2] == 0x30) && (recv_msgTp == 0x11)) {
        recv_UpdateEvent();
    }


    if((recv.forUs) && (recv_isMsg) && (recv_msgTp >= 0x12)) {
        recv_PeerEvent();
    }

    //to do: if it is a broadcast message, do something with
    recv.data[0] = 0;                                                           // otherwise ignore
}
void HM::send_poll(void) {                                                      // handles the send queue
  /*  if((send.counter <= send.retries) && (send.timer <= millis())) {            // not all sends done and timing is OK

        // here we encode and send the string
        hm_enc(send.data);                                                      // encode the string
        disableIRQ_GDO0();                                                       // disable interrupt otherwise we could get some new content while we copy the buffer

        cc.sendData(send.data,send.burst);                                      // and send
        enableIRQ_GDO0();                                                        // enable the interrupt again
        hm_dec(send.data);                                                      // decode the string

        // setting some variables
        send.counter++;                                                         // increase send counter
        send.timer = millis() + timeOut;                                        // set the timer for next action
        powr.state = 1;                                                         // remember TRX module status, after sending it is always in RX mode
        if ((powr.mode > 0) && (powr.nxtTO < (millis() + powr.minTO))) stayAwake(powr.minTO); // stay awake for some time

    }*/

    if((send.counter > send.retries) && (send.counter < maxRetries)) {          // all send but don't wait for an ACK
        send.counter = 0; send.timer = 0;                                       // clear send flag
    }

   /* if((send.counter > send.retries) && (send.timer <= millis())) {             // max retries achieved, but seems to have no answer
        send.counter = 0; send.timer = 0;                                       // cleanup of send buffer
    }*/
}                                                                                                                           // ready, should work
void HM::send_conf_poll(void) {
    if (send.counter > 0) return;                                               // send queue is busy, let's wait
    uint8_t len;

    if (conf.type == 0x01) {
        // answer                            Cnl  Peer         Peer         Peer         Peer
        // l> 1A 05 A0 10 1E 7A AD 63 19 63  01   1F A6 5C 02  1F A6 5C 01  11 22 33 02  11 22 33 01
        //                                   Cnl  Termination
        // l> 0E 06 A0 10 1E 7A AD 63 19 63  01   00 00 00 00
        len = getPeerListForMsg(conf.channel, send_payLoad+1);                  // get peer list
        if (len == 0x00) {                                                      // check if all done
            memset(&conf, 0, sizeof(conf));                                     // clear the channel struct
            return;                                                             // exit
        } else if (len == 0xff) {                                               // failure, out of range
            memset(&conf, 0, sizeof(conf));                                     // clear the channel struct
            send_NACK();
        } else {                                                                // seems to be ok, answer
            send_payLoad[0] = 0x01;                                             // INFO_PEER_LIST
            send_prep(conf.mCnt++,0xA0,0x10,conf.reID,send_payLoad,len+1);      // prepare the message
            //send_prep(send.mCnt++,0xA0,0x10,conf.reID,send_payLoad,len+1);        // prepare the message
        }
    } else if (conf.type == 0x02) {
        // INFO_PARAM_RESPONSE_PAIRS message
        //                               RegL_01:  30:06 32:50 34:4B 35:50 56:00 57:24 58:01 59:01 00:00
        // l> 1A 04 A0 10  1E 7A AD  63 19 63  02  30 06 32 50 34 4B 35 50 56 00 57 24 58 01 59 01 (l:27)(131405)

        //Serial << "hab dich\n";
        len = getListForMsg2(conf.channel, conf.list, conf.peer, send_payLoad+1); // get the message
        if (len == 0) {                                                         // check if all done
            memset(&conf, 0, sizeof(conf));                                     // clear the channel struct
            return;                                                             // and exit
        } else if (len == 0xff) {                                               // failure, out of range
            memset(&conf, 0, sizeof(conf));                                     // clear the channel struct
            send_NACK();
        } else {                                                                // seems to be ok, answer
            send_payLoad[0] = 0x02;                                             // INFO_PARAM_RESPONSE_PAIRS
            send_prep(conf.mCnt++,0xA0,0x10,conf.reID,send_payLoad,len+1);      // prepare the message
            //send_prep(send.mCnt++,0xA0,0x10,conf.reID,send_payLoad,len+1);        // prepare the message
        }
    } else if (conf.type == 0x03) {
        // INFO_PARAM_RESPONSE_SEQ message
        //                               RegL_01:  30:06 32:50 34:4B 35:50 56:00 57:24 58:01 59:01 00:00
        // l> 1A 04 A0 10  1E 7A AD  63 19 63  02  30 06 32 50 34 4B 35 50 56 00 57 24 58 01 59 01 (l:27)(131405)

    }

}
void HM::send_peer_poll(void) {
    // go through the peer database and get the idx per slot, load the respective list4
    // if no peer exist in the respective channel then send to master
    // send out the message accordingly, loop until send_poll is clear and start the next peer.
    // if the message was a long key press, then prepare the struct for sending out a last message
    // with ACK requested

    if (send.counter > 0) return;                                               // something is in the send queue, lets wait for the next free slot

    // we are in a loop, therefore check if the request is completed and clear struct
    if (pevt.idx >= peermax[pevt.cnl-1]) {                                      // we are through the list of peers, clear variables
        // check if a message was send to at least on device, if not send to master
        if (pevt.sta == 0) send_prep(send.mCnt++,(bitRead(pevt.mFlg,5)?0xA2:0x82),pevt.type,regDev.pairCentral,pevt.data,pevt.len);

        pevt.idx = 0; pevt.sta = 0; pevt.act = 0;               // clear struct object, no need to jump in again
        return;

    }

    // prepare the next peer address
    uint8_t peerBuf[4];
    uint8_t ret = getPeerByIdx(pevt.cnl,pevt.idx,peerBuf);                      // get the respective peer from database
    if ((memcmp(peerBuf,broadCast,4) == 0) || (ret == 0)) {                     // if peer is empty, increase the idx and leave while we are in a loop
        pevt.idx++;
        return;
    }

    // get the respective list4
    s_slcVar sV;                                                                // some declarations
    uint8_t* regLstByte = 0;

    ret = getSliceDetail(pevt.cnl, 4, peerBuf, &sV);                            // get cnl list4
    if (ret) {
        // at the moment we are looking only for register address 1 in list 4 (peerNeedsBurst, expectAES), list 4 will not be available in user space
        uint8_t tLst[sV.phyLen];                                                // size a variable
        ret = getRegList(sV.slcPtr, sV.slcLen, tLst);                           // get the register in the variable

        uint8_t *x = (uint8_t*)memchr(tLst, 0x01, sV.phyLen);                                // search the character in the address string
        if (x) {                                                      // if we found the searched string
            uint8_t* dataPtr = (uint8_t*)(x-tLst);                      // calculate the respective address in list
            uint8_t* regPtr = (uint8_t*)&regs;
            uint8_t* phyPtr = (uint8_t*)sV.phyAddr;
            regLstByte = (uint8_t*)((uint32_t)dataPtr + (uint32_t)regPtr + (uint32_t)phyPtr);
         }
     }

    // in regLstByte there are two information. peer needs AES and burst needed
    // AES will be ignored at the moment, but burst needed will be translated into the message flag - bit 0 in regLstByte, translated to bit 4 = burst transmission in msgFlag
    uint8_t mFlg = pevt.mFlg;                                                   // copy the message flag
    mFlg |= bitRead(*regLstByte,0) << 4;                                         // read the need burst flag

    // prepare send string and increase timer
    send_prep(send.mCnt++,mFlg,pevt.type,peerBuf,pevt.data,pevt.len);           // prepare the message
    pevt.sta = 1;                                                               // indicates that we had found a peer and string was send
    pevt.idx++;                                                                 // increase idx for next try
}


void HM::power_poll(void) {
    // there are 3 power modes for the TRX868 module
    // TX mode will switched on while something is in the send queue
    // 1 - RX mode enabled by default, take approx 17ma
    // 2 - RX is in burst mode, RX will be switched on every 250ms to check if there is a carrier signal
    //     if yes - RX will stay enabled until timeout is reached, prolongation of timeout via receive function seems not necessary
    //              to be able to receive an ACK, RX mode should be switched on by send function
    //     if no  - RX will go in idle mode and wait for the next carrier sense check
    // 3 - RX is off by default, TX mode is enabled while sending something
    //     configuration mode is required in this setup to be able to receive at least pairing and config request strings
    //     should be realized by a 15 sec timeout function for RX mode
    //     system time in millis will be hold by a regular wakeup from the watchdog timer
    // 4 - Same as power mode 3 but without watchdog

    if (powr.mode == 0) return;                                                 // in mode 0 there is nothing to do
  //  if (powr.nxtTO > millis()) return;                                          // no need to do anything
    if (send.counter > 0) return;                                               // send queue not empty

    // power mode 2, module is in sleep and next check is reached
    if ((powr.mode == 2) && (powr.state == 0)) {
        if (cc.detectBurst()) {                                                 // check for a burst signal, if we have one, we should stay awake
 //           powr.nxtTO = millis() + powr.minTO;                                 // schedule next timeout with some delay
        } else {                                                                // no burst was detected, go to sleep in next cycle
 //           powr.nxtTO = millis();                                              // set timer accordingly
        }
        powr.state = 1;                                                         // set status to awake
        return;
    }

    // power mode 2, module is active and next check is reached
    if ((powr.mode == 2) && (powr.state == 1)) {
        cc.powerDown();                                                  // go to sleep
        powr.state = 0;
    //    powr.nxtTO = millis() + 250;                                            // schedule next check in 250 ms
    }

    //  power mode 3, check RX mode against timer. typically RX is off beside a special command to switch RX on for at least 30 seconds
    if ((powr.mode >= 3) && (powr.state == 1)) {
        cc.powerDown();                                                  // go to sleep
        powr.state = 0;
    }

    // sleep for mode 2, 3 and 4
    if ((powr.mode > 1) && (powr.state == 0)) {                                 // TRX module is off, so lets sleep for a while
//TODO low power
//      ld.stop();                                                              // stop blinking, because we are going to sleep
//      if ((powr.mode == 2) || (powr.mode == 3)) WDTCSR |= (1<<WDIE);          // enable watch dog if power mode 2 or 3

//      ADCSRA = 0;                                                             // disable ADC
//      uint8_t xPrr = PRR;                                                     // turn off various modules
//      PRR = 0xFF;

//      sleep_enable();                                                         // enable the sleep mode

////        MCUCR |= (1<<BODS) | (1<<BODSE);                                        // turn off brown-out enable in software
////        MCUCR &= ~(1<<BODSE);                                                   // must be done right before sleep
//      sleep_cpu();                                                            // goto sleep
//
//      /* wake up here */
//
//      sleep_disable();                                                        // disable sleep
//      if ((powr.mode == 2) || (powr.mode == 3)) WDTCSR &= ~(1<<WDIE);                                                 // disable watch dog
//      PRR = xPrr;                                                             // restore modules
//
//      if (wd_flag == 1) {                                                     // add the watchdog time to millis()
//          wd_flag = 0;                                                        // to detect the next watch dog timeout
//          timer0_millis += powr.wdTme;                                        // add watchdog time to millis() function
//      } else {
//          stayAwake(powr.minTO);                                              // stay awake for some time, if the wakeup where not raised from watchdog
//      }
//      ld.set(2);                                                              // blink the led to show it is awake
    }
}

// receive message handling
void HM::recv_ConfigPeerAdd(void) {
    // description --------------------------------------------------------
    //                                  Cnl      PeerID    PeerCnl_A  PeerCnl_B
    // l> 10 55 A0 01 63 19 63 1E 7A AD 03   01  1F A6 5C  06         05

    // do something with the information ----------------------------------
    addPeerFromMsg(recv_payLoad[0], recv_payLoad+2);

    // send appropriate answer ---------------------------------------------
    // l> 0A 55 80 02 1E 7A AD 63 19 63 00
    if (recv_ackRq) send_ACK();                                                     // send ACK if requested

}
void HM::recv_ConfigPeerRemove(void) {
    // description --------------------------------------------------------
    //                                  Cnl      PeerID    PeerCnl_A  PeerCnl_B
    // l> 10 55 A0 01 63 19 63 1E 7A AD 03   02  1F A6 5C  06         05

    // do something with the information ----------------------------------
    removePeerFromMsg(recv_payLoad[0], recv_payLoad+2);

    // send appropriate answer ---------------------------------------------
    // l> 0A 55 80 02 1E 7A AD 63 19 63 00
    if (recv_ackRq) send_ACK();
    //if ((recv_ackRq) && (ret == 1)) send_ACK();                                   // send ACK if requested
    //else if (recv_ackRq) send_NACK();
}

void HM::recv_ConfigPeerListReq(void) {
    // description --------------------------------------------------------
    //                                  Cnl
    // l> 0B 05 A0 01 63 19 63 1E 7A AD 01  03

    // do something with the information ----------------------------------
    conf.mCnt = recv_rCnt;
    conf.channel = recv_payLoad[0];
    memcpy(conf.reID, recv_reID, 3);
    conf.type = 0x01;

    // send appropriate answer ---------------------------------------------
    // answer will be generated in config_poll function
    conf.act = 1;
}

void HM::recv_ConfigParamReq(void) {
    // description --------------------------------------------------------
    //                                  Cnl    PeerID    PeerCnl  ParmLst
    // l> 10 04 A0 01 63 19 63 1E 7A AD 01  04 00 00 00  00       01
    // do something with the information ----------------------------------
    conf.mCnt = recv_rCnt;
    conf.channel = recv_payLoad[0];
    conf.list = recv_payLoad[6];
    memcpy(conf.peer, &recv_payLoad[2], 4);
    memcpy(conf.reID, recv_reID, 3);

    conf.type = 0x02;

    // send appropriate answer ---------------------------------------------
    // answer will be generated in config_poll function
    conf.act = 1;
}
void HM::recv_ConfigStart(void) {
    // description --------------------------------------------------------
    //                                  Cnl    PeerID    PeerCnl  ParmLst
    // l> 10 01 A0 01 63 19 63 1E 7A AD 00  05 00 00 00  00       00

    // do something with the information ----------------------------------
    // todo: check against known master id, if master id is empty, set from everyone is allowed
    conf.channel = recv_payLoad[0];                                             // set parameter
    memcpy(conf.peer,&recv_payLoad[2],4);
    conf.list = recv_payLoad[6];
    conf.wrEn = 1;                                                              // and enable write to config

    // send appropriate answer ---------------------------------------------
    if (recv_ackRq) send_ACK();                                                 // send ACK if requested
}
void HM::recv_ConfigEnd(void) {
    // description --------------------------------------------------------
    //                                  Cnl
    // l> 0B 01 A0 01 63 19 63 1E 7A AD 00  06

    // do something with the information ----------------------------------
    conf.wrEn = 0;                                                              // disable write to config
    getMainChConfig();                                                          // probably something changed, reload config
    setConfigEvent();                                                           // raise a config had changed event in user space

    // send appropriate answer ---------------------------------------------
    if (recv_ackRq) send_ACK();                                                 // send ACK if requested
}
void HM::recv_ConfigWriteIndex(void) {
    // description --------------------------------------------------------
    //                                  Cnl    Data
    // l> 13 02 A0 01 63 19 63 1E 7A AD 00  08 02 01 0A 63 0B 19 0C 63

    // do something with the information ----------------------------------
    if ((!conf.wrEn) || (!(conf.channel == recv_payLoad[0]))) {                 // but only if we are in config mode
        return;
    }
    uint8_t payLen = recv_len - 11;                                             // calculate len of payload and provide the data
    uint8_t ret = setListFromMsg(conf.channel, conf.list, conf.peer, &recv_payLoad[2], payLen);
    //Serial << "we: " << conf.wrEn << ", cnl: " << conf.channel << ", lst: " << conf.list << ", peer: " << pHex(conf.peer,4) << '\n';
    //Serial << "pl: " << pHex(&recv_payLoad[2],payLen) << ", ret: " << ret << '\n';

    // send appropriate answer ---------------------------------------------
    if ((recv_ackRq) && (ret == 1))send_ACK();                                  // send ACK if requested
    else if (recv_ackRq) send_NACK();                                           // send NACK while something went wrong
}

void HM::recv_ConfigSerialReq(void) {
    // description --------------------------------------------------------
    // l> 0B 48 A0 01 63 19 63 1E 7A AD 00 09

    // do something with the information ----------------------------------
    // nothing to do, we have only to answer

    // send appropriate answer ---------------------------------------------
    //                                     SerNr
    // l> 14 48 80 10 1E 7A AD 63 19 63 00 4A 45 51 30 37 33 31 39 30 35
    send_payLoad[0] = 0x00;                                                     // INFO_SERIAL
    memcpy(&send_payLoad[1], &devParam[3], 11);                             // copy details out of register.h
    send_prep(recv_rCnt,0x80,0x10,recv_reID,send_payLoad,11);                   // prepare the message
    //send_prep(send.mCnt++,0x80,0x10,recv_reID,send_payLoad,11);               // prepare the message
}

void HM::recv_Pair_Serial(void) {
    // description --------------------------------------------------------
    //                                         Serial
    // l> 15 48 A0 01 63 19 63 1E 7A AD 00 0A  4A 45 51 30 37 33 31 39 30 35

    // do something with the information ----------------------------------
    // compare serial number with own serial number and send pairing string back
    if (memcmp(&recv_payLoad[2],&devParam[3],10) != 0) return;

    // send appropriate answer ---------------------------------------------
    // l> 1A 01 A2 00 3F A6 5C 00 00 00 10 80 02 50 53 30 30 30 30 30 30 30 31 9F 04 01 01
    memcpy(send_payLoad, devParam, 17);                                     // copy details out of register.h
    send_prep(send.mCnt++,0xA2,0x00,recv_reID,send_payLoad,17);
}

void HM::recv_ConfigStatusReq(void) {
    // description --------------------------------------------------------
    //                                   Cnl
    // l> 0B 30 A0 01 63 19 63 2F B7 4A  01  0E

    // do something with the information ----------------------------------
    uint8_t ret = recv_Jump(0);

    // send appropriate answer ---------------------------------------------
    // answer will be send from client function; if client function was not found in jump table, we send here an empty status
    if (!ret) sendInfoActuatorStatus(recv_payLoad[0], 0xff, 0);
}

/**
 * We will reboot the controller and start the update via the bootloader
 */
void HM::recv_UpdateEvent(void) {
    WDT_A_start(WDT_A_BASE);
    while(1) {}
}
void HM::recv_PeerEvent(void) {
    // description --------------------------------------------------------
    //                 peer                cnl  payload
    // -> 0B 56 A4 40  AA BB CC  3F A6 5C  03   CA

    // do something with the information ----------------------------------
    uint8_t peer[4];                                                            // bring it in a search able format
    memcpy(peer,recv_reID,4);
    peer[3] = recv_payLoad[0] & 0xF;                                            // only the low byte is the channel indicator

    uint8_t cnl = getCnlByPeer(peer);                                           // check on peerdb
    if (!cnl) return;                                                           // if peer was not found, the function returns a 0 and we can leave

    getList3ByPeer(cnl, peer);                                                  // load list3
    recv_Jump(cnl);                                                             // jump in user function, we do not need to check, because answer is an ACK

    // send appropriate answer ---------------------------------------------
    // answer should be initiated by client function in user area
    // if (recv_ackRq) send_ACK();                                              // send ACK if requested
}

void HM::recv_PairEvent(void) {
    // description --------------------------------------------------------
    //                 peer                    cnl  payload
    // -> 0E E6 A0 11  63 19 63  2F B7 4A  02  01   C8 00 00
    // answer is an enhanced ACK:
    // <- 0E E7 80 02 1F B7 4A 63 19 63 01 01 C8 00 54

    // do something with the information ----------------------------------
    recv_Jump(0);                                                   // jump in user function, if no answer from user function, we send a blank status answer


    // send appropriate answer ---------------------------------------------
    // answer should be initiated by client function in user area
}

uint8_t HM::recv_Jump(uint8_t tCnl) {
    s_jumptable x;

    for (s_jumptable* p = jTblPtr; ; ++p) {                                     // find the call back function
        // check message type, should be in the list
        // if message type 01, check against byte11, return byte10 as channel and 12 to x as payload
        // l> 0B E2 A0 01 63 19 63 2F B7 4A 01 0E
        //
        // if message type 11, check against byte10, return byte11 as channel and 12 to x as payload
        // -> 0E E3 A0 11 63 19 63 2F B7 4A 02 01 C8 00 00
        //
        // if message type >12, check against nothing, return byte10 as channel and 11 to x as payload
        // -> 0B 4E A4 40 22 66 08 2F B7 4A 01 68
        //
        //Serial << "rm:" << recv_msgTp << ", by10:" << recv_by10 << ", ps:" << p->spec << '\n';

        x.code = p->code;                                       // get back variables, because they are in program memory
        x.spec = p->spec;
        x.fun =  (void (*)(uint8_t, uint8_t*, uint8_t))p->fun;

        if ((recv_msgTp == 0x01) && (recv_msgTp == x.code)) {
            x.fun(recv_payLoad[0], recv_payLoad+1, recv_len - 10);              // and jump into
            return 1;
        } else if ((recv_msgTp == 0x11) && (recv_msgTp == x.code) && (recv_by10 == x.spec)) {
            x.fun(recv_payLoad[1], recv_payLoad+2, recv_len - 11);              // and jump into
            return 1;
        } else if ((recv_msgTp >= 0x12) && (recv_msgTp == x.code)) {
            x.fun(tCnl, recv_payLoad, recv_len - 9);                            // and jump into
            return 1;
        }
        if (x.code == 0) break;                                                 // break if on end of list
    }
    return 0;
}

// internal send functions
void    HM::send_prep(uint8_t msgCnt, uint8_t comBits, uint8_t msgType, uint8_t *targetID, uint8_t *payLoad, uint8_t payLen) {
    send.data[0]  = 9 + payLen;                                                 // message length
    send.data[1]  = msgCnt;                                                     // message counter

    // set the message flags
    //    #RPTEN    0x80: set in every message. Meaning?
    //    #RPTED    0x40: repeated (repeater operation)
    //    #BIDI     0x20: response is expected                      - should be in comBits
    //    #Burst    0x10: set if burst is required by device        - will be set in peer send string if necessary
    //    #Bit3     0x08:
    //    #CFG      0x04: Device in Config mode                     - peer seems to be always in config mode, message to master only if an write mode enable string was received
    //    #WAKEMEUP 0x02: awake - hurry up to send messages         - only for a master, peer don't need
    //    #WAKEUP   0x01: send initially to keep the device awake   - should be only necessary while receiving

    send.data[2]  = comBits;                                                    // take the communication bits
    send.data[3]  = msgType;                                                    // message type
    memcpy(&send.data[4], HMID, 3);                                             // source id
    memcpy(&send.data[7], targetID, 3);                                         // target id

    if (payLoad != send_payLoad)
        memcpy(send_payLoad, payLoad, payLen);                                  // payload

    send_out();
}

// some internal helpers
void HM::hm_enc(uint8_t *buffer) {

    buffer[1] = (~buffer[1]) ^ 0x89;
    uint8_t buf2 = buffer[2];
    uint8_t prev = buffer[1];

    uint8_t i;
    for (i=2; i<buffer[0]; i++) {
        prev = (prev + 0xdc) ^ buffer[i];
        buffer[i] = prev;
    }

    buffer[i] ^= buf2;
}
void HM::hm_dec(uint8_t *buffer) {

    uint8_t prev = buffer[1];
    buffer[1] = (~buffer[1]) ^ 0x89;

    uint8_t i, t;
    for (i=2; i<buffer[0]; i++) {
        t = buffer[i];
        buffer[i] = (prev + 0xdc) ^ buffer[i];
        prev = t;
    }

    buffer[i] ^= buffer[2];
}
void HM::exMsg(uint8_t *buf) {

}
// - Storage Management ---------------------------------------------------------------------------------------------------
void    HM::initRegisters() {

    if (bFirstStart == 1)
        bFirstStart = 0;

    getMainChConfig();                                                         // fill the structure of main channel configs
};

// slice table functions
uint8_t HM::getSliceDetail(uint8_t cnl, uint8_t lst, uint8_t *peer, s_slcVar *sV) {
    uint8_t peerIdx, cType, cnt;                                                // size the variables
    uint16_t listSliceIdx = 0;
    uint32_t peerL =  *(long*)&peer[0];                                         // convert peer from byte array to long

    // request must be valid and peer must exist
    if (cnl >= devDef.nbrChannels) return 0;                                    // channel out of range, end
    if ((peerL == 0) && (lst >= 3)) return 0;                                   // empty peer is not valid for List3 or List4
    if ((cnl == 0) && (lst !=0)) return 0;                                      // channel 0 has only a list0

    if (lst < 3) peerIdx = 0;                                                   // List0 & List1 needs no index
    else peerIdx = getIdxByPeer(cnl,peer);                                      // get the peer index of respective peer
    //Serial << "pI:" << peerIdx << ", cnl:" << cnl << '\n';
    if (peerIdx == 0xff) return 0;                                              // peer failure

    cType = devDef.chDefType[cnl].type;                                         // is the request for the device or for a channel
    for (cnt = 0; cnt < listTypeDef[cType].nbrLists; cnt++) {                   // find respective list number
        if (listTypeDef[cType].type[cnt].ListNo == lst) break;
        listSliceIdx += listTypeDef[cType].type[cnt].nbrOfSlice;                // remember the appropriate listSliceIndex
    }

    if (listTypeDef[cType].nbrLists <= cnt) return 0;                           // list not found
    if (listTypeDef[cType].type[cnt].nbrPeers < peerIdx) return 0;              // peer out of range

    // find slice and read variables
    sV->slcPtr = devDef.chDefType[cnl].sliceIdx + listSliceIdx +
    peerIdx*listTypeDef[cType].type[cnt].nbrOfSlice;                            // calculate slice idx incl. peer idx

    sV->phyAddr = sliceStr[sV->slcPtr].phyAddr;                                 // get physical address

    sV->slcLen = listTypeDef[cType].type[cnt].nbrOfSlice;                       // define end slice
    sV->phyLen = 0;                                                             // len to 0 because we want to add current slices
    for (uint8_t i = 0; i < sV->slcLen; i++) {                                  // step through slices
        sV->phyLen += sliceStr[sV->slcPtr+i].nbrBytes;                          // add amount of bytes
    }

    return 1;
}
uint8_t HM::doesListExist(uint8_t cnl, uint8_t lst) {
    // check if a list exist
    uint8_t cType, cnt;                                                         // size variables
    cType = devDef.chDefType[cnl].type;                                         // get channel device type by channel
    for (cnt = 0; cnt < listTypeDef[cType].nbrLists; cnt++) {                   // find respective list number
        if (listTypeDef[cType].type[cnt].ListNo == lst) break;                  // if we have found list4 then break
    }
    //Serial << "cnl: " << cnl << ", cType: " << cType << ", lst: " << lst << ", cnt: " << cnt << '\n';
    if (listTypeDef[cType].nbrLists <= cnt) return 0;                           // list not found
    else return 1;                                                              // list found
}

uint8_t HM::getRegList(uint8_t slcPtr, uint8_t slcLen, uint8_t *buf) {

    slcLen += slcPtr;                                                           // calculate slice len

    uint8_t msgLen = 0;                                                         // counter for bytes written in message
    for (uint8_t i = slcPtr; i < slcLen; i++) {                                 // count through slice table
        uint16_t addr = sliceStr[i].regAddr;                                    // position the pointer on the slice string

        for (uint8_t j = 0; j < sliceStr[i].nbrBytes; j++) {                    // count through slice
            *(buf++) = addr++;                                                  // write to buffer
            msgLen++;                                                           // increase message counter
        }
    }

    return msgLen;                                                              // return the message len
}
void HM::getMainChConfig(void) {
    uint8_t peer[] = {0xff,0xff,0xff,0x00};                                // some declarations
    s_slcVar sV;

    // get cnl0 list0 for internal and external use
    getSliceDetail(0, 0, (uint8_t*) &broadCast, &sV);

    memcpy((uint16_t*)(sV.phyAddr+(uint16_t*)&regs), (const void*)&regDev, sV.phyLen);

    // step through the channels and lists and load registers
    uint8_t cnt = 0;
    uint16_t *mc = (uint16_t*)mcConfPtr;
    for (uint8_t i = 0; i <= maxChannel; i++) {                                 // count through the channel
        for (uint8_t j = 0; j <= 7; j++) {                                      // count through the lists, 7 should be the max
            if (!getSliceDetail(i, j, &peer[0], &sV)) continue;                 // get the slice details, if empty then next
            if (j == 3) l3Ptr[i] = mc[cnt];                                     // remember list3 pointer

            memcpy((uint16_t*)(sV.phyAddr+(uint16_t*)&regs), (const void*)mc[cnt], sV.phyLen);
            cnt++;                                                              // increase pointer to ptr list
        }
    }
}
void HM::getList3ByPeer(uint8_t cnl, uint8_t *peer) {
    s_slcVar sV;                                                                // some declarations

    uint8_t ret = getSliceDetail(cnl, 3, peer, &sV);                            // get cnl list3 for external use
    if (ret)
        memcpy((uint16_t*)(sV.phyAddr+(uint16_t*)&regs), (const void*)l3Ptr[cnl], sV.phyLen);

}

// message generation for TRX868 module
uint8_t HM::getListForMsg2(uint8_t cnl, uint8_t lst, uint8_t *peer, uint8_t *buf) {
    // return 0     = no data, junk out of range
    //        3-254 = chars returned
    //        0xff  = input params unknown

    const uint8_t bytesPerJunk = 8;                                                     // how many bytes should one junk deliver

    static uint8_t tcnl, tlst, msgPtr = 0;                                      // size variables
    static uint32_t tpeerL, peerL;
    static s_slcVar sV;

    // check if we are complete, check mark is the physical len of the data string
    if (msgPtr > sV.phyLen) return msgPtr = 0;                                      // we are completely through, stop action

    // check to load the storage details only one time
    peerL = *(long*)&peer[0];                                                   // convert byte array to long
    if ((tcnl != cnl) || (tlst != lst) || (tpeerL != peerL)) msgPtr = 0;        // request had changed, start from beginning
    tcnl = cnl; tlst = lst; tpeerL = peerL;                                     // store for next check

    if (msgPtr == 0) {                                                          // only the first time we have to get the details
        if (!getSliceDetail(cnl, lst, peer, &sV)) return 0xff;
        sV.slcLen += sV.slcPtr;                                                 // calculate slice len
    }

    // go through the slices and take only the bytes needed
    uint8_t msgCnt = 0, bInMsg = 0;                                             // counter the bytes

    for (uint8_t i = sV.slcPtr; i < sV.slcLen; i++) {                           // count through slice table
        if (bInMsg >= bytesPerJunk) break;                                      // step out if we have enough bytes
        uint8_t addr = sliceStr[i].regAddr;                                     // remember the address byte from slice table
        uint16_t* dataPtr = sliceStr[i].phyAddr + (uint16_t*)&regs;           // remember the physical address

        for (uint8_t j = 0; j < sliceStr[i].nbrBytes; j++) {                    // count through slice
            msgCnt++;                                                           // increase message counter
            if (msgCnt > msgPtr) {                                              // write only to buffer if we need that bytes
                bInMsg++;                                                       // increase the byte counter
                *(buf++) = addr;                                                // add the address byte
                *(buf++) = *dataPtr;                                            // add the data byte from eeprom
            }
            if (bInMsg >= bytesPerJunk) break;                                  // step out if we have enough bytes
            addr++;                                                             // increase addr counter accordingly
            dataPtr++;                                                          // increase the pointer to the eeprom location
        }
    }
    msgPtr = msgCnt;                                                            // set the msgPtr for next try

    // add the 00 00 as termination
    if ((msgPtr == sV.phyLen) && (bInMsg < bytesPerJunk)) {                     // if we have delivered all bytes and there is some space in the junk
        *(buf++) = 0;                                                           // add a 0 on end for termination
        *(buf++) = 0;                                                           // add a 0 on end for termination
        msgPtr++;                                                               // increase msgPtr to step out next try
        bInMsg++;                                                               // increase the byte counter to deliver the right amount of bytes
    }
    bInMsg = bInMsg << 1;                                                       // *2 because we add addr and data byte

    return bInMsg;                                                              // return accordingly
}
uint8_t HM::getListForMsg3(uint8_t cnl, uint8_t lst, uint8_t *peer, uint8_t *buf) {
    return 0;
}
uint8_t HM::setListFromMsg(uint8_t cnl, uint8_t lst, uint8_t *peer, const uint8_t *buf, uint8_t len) {
    uint8_t aLen; // size variables

    s_slcVar sV;

    // get the slice details and the address list
    if (!getSliceDetail(cnl, lst, peer, &sV)) return 0;
    uint8_t addrStr[sV.phyLen];                                                 // size the buffer for the address string

    aLen = getRegList(sV.slcPtr, sV.slcLen, addrStr);                           // get the address string
    if (aLen == 0) return 0;                                                    // if string is empty we can leave

    // step through the buffer bytes and search for the right address
    uint8_t* dataPtr;
    for (uint8_t i = 0; i < len; i+=2) {                                        // count through input string
        uint8_t *x = (uint8_t *)memchr(addrStr, buf[i], sV.phyLen);                           // search the character in the address string
        if (x)
            continue;                                                   // if we got no result try next

        dataPtr = (uint8_t*)(x-addrStr);                                // calculate the respective address in list

        *(uint8_t*)((uint32_t)dataPtr+(uint32_t)&regs+(uint32_t)sV.phyAddr) = buf[i+1];
    }


    // reread of channel config will be done when we got the message to end setup
    //if ((lst <= 1) || (lst >= 5)) getMainChConfig();                          // reread main channel config
    return 1;
}
uint8_t HM::getPeerListForMsg(uint8_t cnl, uint8_t *buf) {
    const uint8_t bytesPerJunk = 16;                                                        // how many bytes should one junk deliver
    if (cnl > maxChannel) return 0xff;                                          // if channel out of range, return

    uint8_t *t, cnt=0;                                                          // size variables
    static uint8_t tcnl, tPtr;
    cnl--;                                                                      // adjust channel while peer database starts with 0

    if (tPtr > peermax[cnl]) return tPtr = 0;                                   // tPtr has exceed limit, assumption we are completely through, stop action
    if (cnl != tcnl) tPtr = 0;                                                  // channel has changed, assume we got a new request
    tcnl = cnl;                                                                 // remember channel for next try

    while (tPtr < peermax[cnl]) {                                               // step through the peers of specific channel
        if (peerdb[cnl][tPtr]) {                                                // only step in while some content available
            t = (uint8_t*)&peerdb[cnl][tPtr++];                                 // pointer to the content in database
            memcpy(buf+cnt,t,4);                                                // copy to buffer
            cnt+=4;                                                             // increase buffer for next try
            if (cnt >= bytesPerJunk) return cnt;                                // take care of max payload
        } else tPtr++;                                                          // if nothing in, increase the pointer only
    }
    memset(buf+cnt,0,4);                                                        // add termination bytes
    tPtr++;
    return cnt+=4;                                                              // all done, waiting for next request
}

// peerdb handling; add, remove and search functions
uint8_t HM::addPeerFromMsg(uint8_t cnl, uint8_t *peer) {
    if (cnl > maxChannel) return 0xff;                                          // if channel out of range, return

    uint8_t ret, tPeer[4];                                                      // size variables

    // copy first peer to database, if everything is ok we got a 1 back
    memcpy(tPeer,peer,4);                                                       // copy the peer in a variable
    ret = addPeerToDB(cnl,tPeer);

    // check if we have to add another peer, if not return the status of adding the first peer
    if (peer[4] == 0 || peer[3] == peer[4]) {
            if (ret) {
                loadDefaultRegset(cnl, peer, false, 0);
                getMainChConfig();
            }
            return ret;
        }

        if (ret) {
            loadDefaultRegset(cnl, peer, true, 0);
        }

    // if we are here we have to copy a second peer to database, if everything is ok we got a 1 back
    tPeer[3] = peer[4];                                                         // copy the second channel to the peer
    ret = addPeerToDB(cnl,tPeer);

        if (ret) {
            loadDefaultRegset(cnl, peer, true, 1);
        }

        getMainChConfig();


    return ret;                                                                 // return the status of the second add
}
uint8_t HM::removePeerFromMsg(uint8_t cnl, uint8_t *peer) {
    if (cnl > maxChannel) return 0xff;                                          // if channel out of range, return

    uint8_t idx1, idx2, tPeer[4];                                               // size variables

    // remove both peers from the database; we will not check if both are existing
    // if we found a valid index, we will delete the peer by writing the broad cast variable in the slot
    memcpy(tPeer,peer,4);                                                       // copy the peer in a variable
    idx1 = getIdxByPeer(cnl, tPeer);                                            // get the idx of the first peer
    if (idx1 != 0xff)
        memcpy(&peerdb[cnl-1][idx1], &broadCast, 4);

    tPeer[3] = peer[4];                                                         // change the peer channel
    idx2 = getIdxByPeer(cnl, tPeer);                                            // get the idx of the second peer
    if (idx2 != 0xff)
        memcpy(&peerdb[cnl-1][idx2], &broadCast, 4);

    return 1;                                                                   // every thing went ok,
}
uint8_t HM::getCnlByPeer(uint8_t *peer) {
    if (memcmp(peer,broadCast,4) == 0) return 0;                                // return failure while peer is empty

    for (uint8_t i=0; i < maxChannel; i++) {                                    // step through the channels
        for (uint8_t j=0; j < peermax[i]; j++) {                                // step through the peers of channel i
            if (memcmp((uint8_t*)&peerdb[i][j],peer,4) == 0) {
                //Serial << "x i:" << i << ", j:" << j << '\n';
                return i+1;     // return 1 if we found something
            }
        }
    }
    return 0;                                                                   // found nothing, return 0
}
uint8_t HM::getIdxByPeer(uint8_t cnl, uint8_t *peer) {
    uint8_t tPeer[3] = {0xff,0xff,0xff};
    if (cnl > maxChannel) return 0xff;                                          // check against max channels

    cnl--;                                                                      // adjust channel due to database start with 0
    for (uint8_t j=0; j < peermax[cnl]; j++) {                                  // step through the peers of channel i
        if (memcmp((uint8_t*)&peerdb[cnl][j],peer,4) == 0) return j;            // return idx if we found something
    }

    if (memcmp(peer,tPeer,3) == 0) {                                            // dummy peer, FFFFFF01 means index 1
        if (peer[3] >= peermax[cnl]) return 0xff;                               // if index out of range, return failure
        else return peer[3];                                                    // otherwise give index
    }

    return 0xff;                                                                // nothing found, return failure
}
uint8_t HM::getPeerByIdx(uint8_t cnl, uint8_t idx, uint8_t *peer) {
    if (cnl > maxChannel) return 0;                                             // check against max channels
    if (idx >= peermax[cnl-1]) return 0;                                        // check against max peer slots

    cnl--;                                                                      // adjust channel due to database start with 0
    memcpy(peer,(uint8_t*)&peerdb[cnl][idx],4);                                 // copy peer from peerdb
    return 1;                                                                   // everything should be fine, return 1
}
uint8_t HM::getFreePeerSlot(uint8_t cnl) {
    cnl--;                                                                      // adjust channel due to database start with 0
    for (uint8_t j=0; j < peermax[cnl]; j++) {                                  // step through the peers of channel
        if (memcmp((uint8_t*)&peerdb[cnl][j],broadCast,4) == 0) return j;       // return idx if we found a free slot
    }
    return 0xff;                                                                // otherwise return failure
}
uint8_t HM::countFreePeerSlot(uint8_t cnl) {
    uint8_t counter = 0;                                                        // size counter variable and set to 0
    cnl--;                                                                      // adjust channel due to database start with 0
    for (uint8_t j=0; j < peermax[cnl]; j++) {                                  // step through the peers of channel
        if (memcmp((uint8_t*)&peerdb[cnl][j],broadCast,4) == 0) counter++;      // increase counter if we found a free slot
    }
    return counter;                                                             // otherwise return failure
}

uint8_t HM::loadDefaultRegset(uint8_t cnl, uint8_t *peer, bool dual, uint8_t idx) {
        uint8_t type = devDef.chDefType[cnl].type;  // get channel type
        if (default_regChans_dev[type].regChan_len == 0) return 0;  // No default. Nothing to do

        // get the slice details and the address list
    s_slcVar sV;
    if (!getSliceDetail(cnl, default_regChans_dev[type].lst, peer, &sV)) return 0;

      //  Serial << "loadDefaultRegset: cnl=" << cnl << " dual: " << dual << " regChan_len: " << default_regChans_dev[type].regChan_len << " lst: " << default_regChans_dev[type].lst << " idx: " << idx << '\n';
        const uint8_t* regset;
        if (dual) {
          if (idx == 0) {
            regset = default_regChans_dev[type].default_regChan_dual_1;
          } else {
            regset = default_regChans_dev[type].default_regChan_dual_2;
          }
        } else {
          regset = default_regChans_dev[type].default_regChan_single;
        }


        memcpy((uint16_t*)(sV.phyAddr+(uint16_t*)&regs), (void*)regset, sV.phyLen);

        return 1;
}

uint8_t HM::addPeerToDB(uint8_t cnl, uint8_t *peer) {
    // check if peer is already known
    uint8_t tCnl = getIdxByPeer(cnl,peer);
    if (tCnl < 0xFF)
        return 0;                                                               // peer already exist

    // check if we have a free slot and add to eeprom
    uint8_t idx = getFreePeerSlot(cnl);                                         // find a free slot

    if (idx == 0xff)
        return 0;                                                               // no free slot

    memcpy(&peerdb[cnl-1][idx], &peer, 4);                                                  // write peer

    return 1;                                                                   // every thing went ok,
}

// to check incoming messages if sender is known
uint8_t HM::isPeerKnown(uint8_t *peer) {
    for (uint8_t i=0; i < maxChannel; i++) {                                    // step through the channels
        for (uint8_t j=0; j < peermax[i]; j++) {                                // step through the peers of channel i
            if (memcmp((uint8_t*)&peerdb[i][j],peer,3) == 0) return 1;          // return 1 if we found something
        }
    }
    return 0;                                                                   // found nothing, return 0
}
uint8_t HM::isPairKnown(uint8_t *pair) {
    if (memcmp(regDev.pairCentral, broadCast, 3) == 0) return 1;                // return 1 while not paired

    if (memcmp(regDev.pairCentral, pair, 3) == 0) return 1;                     // check against regDev
    else return 0;                                                              // found nothing, return 0
}


/*#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void){

    if(GPIO_getInterruptStatus(GDO0_PORT, GDO0_PIN)){
        HM::isrGDO0event();
        GPIO_clearInterrupt(GDO0_PORT, GDO0_PIN);

    }
}
*/

