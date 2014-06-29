//
//  RXPacket.h
//  
//
//  Created by Rahul Salvi on 6/28/14.
//
//

#ifndef ____RXPacket__
#define ____RXPacket__

#include <iostream>
#include "Arduino.h"

enum RXIndex {
    VELX = 0,
    VELY = 1,
    VELZ = 2,
    ROTX = 3,
    ROTY = 4,
    ROTZ = 5,
    POSZ = 6,
    TORPEDOCTL = 8,
    SERVOCTL = 9,
    SPARE = 15,
    CHECKSUM = 17
};

class RXPacket {
public:
    RXPacket();
private:
    int8_t data[18];
    bool readPacket();
    bool isChecksumValid();
};

#endif /* defined(____RXPacket__) */
