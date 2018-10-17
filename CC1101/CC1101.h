/*
 * CC1101.h
 *
 *  Created on: Sep 21, 2018
 *      Author: a0406859
 */

#ifndef CC1101_CC1101_H_
#define CC1101_CC1101_H_

#include "CC.h"

class CC1101 : public CC
{
public:
    CC1101();
    virtual void    init(void);

    virtual ~CC1101();
};

#endif /* CC1101_CC1101_H_ */
