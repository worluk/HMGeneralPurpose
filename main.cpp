/*
 * main.cpp
 *
 *  Created on: 7 Feb 2018
 *      Author: worluk
 */

#include <Hardware/Button/Button.h>
#include <HomeMatic/HomeMatic_.h>
#include "Hardware/initHardware.h"
#include <msp430.h>

#include "HomeMatic/Register.h"

#include <string.h>
#include <intrinsics.h>

#include "driverlib.h"
#include "Hardware/LED/led.h"


#include "helpers/helper.h"

#include <functional>

#include "Hardware/Serial/Serial.h"
#include <System/SystemTimer/SystemTimer.h>


HM* hm;
//- homematic communication -----------------------------------------------------------------------------------------------


//- current sensor
unsigned long lastCurrentInfoSentTime = 0;
unsigned long lastCurrentSenseTime = 0;
unsigned long currentImpulsStart = 0;
unsigned long lastSensorImpulsLength = 0;
unsigned long lastCurrentSenseImpulsLength = 0;
bool lastCurrentSense = false;
bool lastCurrentPin = false;
//const uint8_t pinCurrent = 31;
//const uint8_t pinRelay = 12;
const unsigned long minImpulsLength = 5000;


//- key handler functions -------------------------------------------------------------------------------------------------
void buttonState(uint8_t idx, uint8_t state) {
    // possible events of this function:
    //   0 - short key press
    //   1 - double short key press
    //   2 - long key press
    //   3 - repeated long key press
    //   4 - end of long key press
    //   5 - double long key press
    //   6 - time out for a double long

    // channel device
    if (idx == 0) {
                if (state == 0) hm->ld->shortBlink();
        if (state == 6) hm->startPairing();                                          // long key press, start pairing
        if (state == 5) hm->reset();                                             // double long key press, reset the device
    }

    // channel 1 - 2
    if ((idx >= 1) && (idx <= 2)) {
        if ((state == 0) || (state == 1)) hm->sendPeerREMOTE(idx,0,0);           // short key or double short key press detected
        if ((state == 2) || (state == 3)) hm->sendPeerREMOTE(idx,1,0);           // long or repeated long key press detected
        if (state == 4) hm->sendPeerREMOTE(idx,2,0);                             // end of long or repeated long key press detected
    }
}

//- relay handler functions -----------------------------------------------------------------------------------------------
/*void relayState(uint8_t cnl, uint8_t curStat, uint8_t nxtStat) {

        if (cnl == 3) { // cnl 3 => switch, cnl 4 => wechselschalter
          if (curStat == 3)
          {
            hm->ld.set(1);
          } else {
            hm->ld.set(0);
          }
        }
}*/

/*void setInternalRelay(uint8_t cnl, uint8_t tValue) {
    if(tValue == 0)
        GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5 );
    else
        GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5 );
}*/

/*void setVirtualRelay(uint8_t cnl, uint8_t tValue) {
   if (! isInitialized) return;
  if (rl[0].getCurStat() == 3) {
    rl[0].setNxtStat(6);
  } else {
    rl[0].setNxtStat(3);
  }
}*/


//- HM functions ----------------------------------------------------------------------------------------------------------
void HM_Status_Request(uint8_t cnl, uint8_t *data, uint8_t len) {
    // message from master to client while requesting the channel specific status
    // client has to send an INFO_ACTUATOR_MESSAGE with the current status of the requested channel
    // there is no payload; data[0] could be ignored

    //if (cnl == 3) rl[0].sendStatus();                                         // send the current status
}
void HM_Set_Cmd(uint8_t cnl, uint8_t *data, uint8_t len) {
    // message from master to client for setting a defined value to client channel
    // client has to send an ACK with the current status; payload is typical 3 bytes
    // data[0] = status message; data[1] = down,up,low battery; data[2] = rssi (signal quality)

//    if (cnl == 3) rl[0].trigger11(data[0], data+1, (len>4)?data+3:NULL);
 //   if (cnl == 4) rl[1].trigger11(data[0], data+1, (len>4)?data+3:NULL);
}
void HM_Reset_Cmd(uint8_t cnl, uint8_t *data, uint8_t len) {

    hm->send_ACK();                                                              // send an ACK
    if (cnl == 0) hm->reset();                                                   // do a reset only if channel is 0
}
void HM_Switch_Event(uint8_t cnl, uint8_t *data, uint8_t len) {
    // sample needed!
    // ACK is requested but will send automatically

}

void HM_Remote_Event(uint8_t cnl, uint8_t *data, uint8_t len) {
    // message from a remote to the client device; this event pop's up if the remote is peered
    // cnl = indicates client device channel
    // data[0] the remote channel, but also the information for long key press - ((data[0] & 0x40)>>6) extracts the long key press
    // data[1] = typically the key counter of the remote

   // if (cnl == 3) rl[0].trigger40(((data[0] & 0x40)>>6),data[1],(void*)&regMC.ch3.l3);
   // if (cnl == 4) rl[1].trigger40(((data[0] & 0x40)>>6),data[1],(void*)&regMC.ch4.l3);
}
void HM_Sensor_Event(uint8_t cnl, uint8_t *data, uint8_t len) {
    // sample needed!
    // ACK is requested but will send automatically

}
void HM_Config_Changed(uint8_t cnl, uint8_t *data, uint8_t len) {

}

/*static const s_jumptable jumptable[] = {                                               // jump table for HM communication
    { 0x01, 0x0E, HM_Status_Request },
    { 0x11, 0x02, HM_Set_Cmd },
    { 0x11, 0x04, HM_Reset_Cmd },
    { 0x3E, 0x00, HM_Switch_Event  },
    { 0x40, 0x00, HM_Remote_Event  },
    { 0xFF, 0xFF, HM_Config_Changed },
    { 0x0 }
};*/

LED*     led1;

Serial debug; //initialize the debug outbound pipe
SystemTimer systemTimer;

//
void testme(){
   // std::string s = std::to_string(systemTimer.getCounter());
    debug <<   " Timer called" << Serial::endl;
    led1->toggle();
}

int main(){

    initHardware();

    Button  button1 (GPIO_PORT_P1,  GPIO_PIN1);         //Set up button on port and pin
    led1   = new LED (GPIO_PORT_P4,  GPIO_PIN6);         //Set up LED on port and pin

//    ButtonCallback shortPressFn = std::bind(&LED::shortBlink3, led1);

  //  button1.assignShortPressFn( shortPressFn );         //if a short press is detected, call shortBlink3 on led1
 //   button1.init();                                     //initialize button, since now it is completely set up

   // CC1101 cc;
  //  cc.init();

     debug << Serial::endl << Serial::endl;
     debug << "Starting up HomeMatic general purpose node" << Serial::endl;
     debug << "---------------------------------------------" << Serial::endl;
     debug << Serial::endl;

     __bis_SR_register(GIE);                            // enable interrupts

     systemTimer.init();
     Timer t(1000, std::bind(testme));
     systemTimer.addTimer(&t);



     while(1);                                       // the rest is interrupt driven, nothing to do but wait.

}


