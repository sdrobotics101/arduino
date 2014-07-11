//
//  TXPacket.h
//  
//
//  Created by Rahul Salvi on 6/28/14.
//
//

#ifndef ____TXPacket__
#define ____TXPacket__

#include "Arduino.h"
#include <math.h>


enum TXIndex {
    ACCX = 2,
    ACCY = 3,
    ACCZ = 4,
    MAGX = 5,
    MAGY = 6,
    MAGZ = 7,
    PRESSURE = 8,
    TXSPARE = 9,
};

class TXPacket {
    friend class PacketController;
private:
    TXPacket();
    void begin();
    
    void sendPacket(USARTClass serialPort);
    int16_t computeChecksum();
    
    uint8_t _data[12];
};


#endif /* defined(____TXPacket__) */
