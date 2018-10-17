/*
 * CCStates.h
 *
 *  Created on: Sep 21, 2018
 *      Author: a0406859
 */

#ifndef CC1101_CCSTATES_H_
#define CC1101_CCSTATES_H_



enum CCState{
    sleep           = 0,
    idle            = 1,
    xoff            = 2,
    vcoon_mc        = 3,
    regon_mc        = 4,
    mancal          = 5,
    vcoon           = 6,
    regon           = 7,
    startcal        = 8,
    bwboost         = 9,
    fs_lock         = 10,
    ifadon          = 11,
    endcal          = 12,
    rx              = 13,
    rx_end          = 14,
    rx_rst          = 15,
    txrx_switch     = 16,
    rxinfo_oflow    = 17,
    fstxon          = 18,
    tx              = 19,
    tx_end          = 20,
    rxtx_switch     = 21,
    txfifo_uflow    = 22

};


#endif /* CC1101_CCSTATES_H_ */
