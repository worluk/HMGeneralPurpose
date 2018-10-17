/*
 * HomeMatic.cpp
 *
 *  Created on: Oct 2, 2018
 *      Author: a0406859
 */

#include <HomeMatic/HomeMatic.h>
#include "Hardware/Serial/Serial.h"

extern Serial debug;

HomeMatic::HomeMatic(){



}

void HomeMatic::init(){
    cc.setGD0FN(std::bind(&HomeMatic::gd0fn, this));
    cc.setGD2FN(std::bind(&HomeMatic::gd2fn, this));

    //sensorTimer = new Timer(100, std::bind(&HomeMatic::sendSensorData, this));

    cc.init();
}

HomeMatic::~HomeMatic(){

}


void HomeMatic::gd0fn(){
    debug << "gd0 function triggered" << Serial::endl;
}

void HomeMatic::gd2fn(){

}
