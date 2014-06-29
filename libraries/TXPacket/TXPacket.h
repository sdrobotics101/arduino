//
//  TXPacket.h
//  
//
//  Created by Rahul Salvi on 6/28/14.
//
//

#ifndef ____TXPacket__
#define ____TXPacket__

enum TXIndex {
    ACCX = 0,
    ACCY = 1,
    ACCZ = 2,
    MAGX = 3,
    MAGY = 4,
    MAGZ = 5,
    PRESSURE = 6,
    SPARE = 7,
    CHECKSUM = 8
};

class TXPacket {
    public:
        
};


#include <iostream>
#include "Arduino.h"

#endif /* defined(____TXPacket__) */
