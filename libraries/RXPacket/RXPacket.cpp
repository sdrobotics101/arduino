//
//  RXPacket.cpp
//  
//
//  Created by Rahul Salvi on 6/28/14.
//
//

#include "RXPacket.h"


RXPacket::RXPacket() {
    for (int i = 0; i < 18; i++) {
        _data[i] = 0;
    }
}

PacketStatus RXPacket::readPacket() {
    Serial1.readBytes(_data, 18);
    if (isChecksumValid()) {
        return VALID_PACKET;
    } else {
        return INVALID_PACKET;
    }
}

bool RXPacket::isChecksumValid() {
    int16_t sum = 0xBDFA;
    for (int i = 0; i < 6; i++) {
        sum += _data[i];
    }
    sum += _data[POSZ] * 256;
    for (int i = 7; i < 16; i++) {
        sum += _data[i];
    }
    if (sum == ((_data[CHECKSUM] * 256) + _data[CHECKSUM+1])) {
        return true;
    } else {
        return false;
    }
}