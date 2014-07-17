//
//  RXPacket.h
//  
//
//  Created by Rahul Salvi on 6/28/14.
//
//

#ifndef ____RXPacket__
#define ____RXPacket__

#include "Arduino.h"


enum PacketStatus {
    VALID_PACKET = 0,
    INVALID_PACKET = 1,
    NO_HEADER_FOUND = 2,
    NOT_ENOUGH_DATA = 3
};

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
    RXSPARE = 15,
    CHECKSUM = 16
};

class RXPacket {
    friend class PacketController;
private:
    RXPacket();
    void begin();
    
    PacketStatus readPacket(USARTClass serialPort);
    uint16_t computeChecksum();
    
    uint8_t _data[18];
};

#endif /* defined(____RXPacket__) */
