/*
 * Serial.h
 *
 *  Created on: Sep 11, 2018
 *      Author: a0406859
 */

#ifndef SYSTEM_SERIAL_SERIAL_H_
#define SYSTEM_SERIAL_SERIAL_H_

#include <string>
#include <stdint.h>
#include <ostream>


class Serial
{
private:
    uint8_t recv(){return 0;}

public:
    Serial();
    virtual ~Serial();

    Serial& operator<<(char);
    Serial& operator<<(std::string);
    Serial& operator<<(uint32_t);
    void putc(char);
    static const std::string endl ;

};

#endif /* SYSTEM_SERIAL_SERIAL_H_ */
