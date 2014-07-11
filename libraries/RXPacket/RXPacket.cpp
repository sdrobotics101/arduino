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

void RXPacket::begin() {
    Serial.println("RXPacket Initialized");
}

PacketStatus RXPacket::readPacket(USARTClass serialPort) {
    serialPort.readBytes(_data, 18);
    if (isChecksumValid()) {
        return VALID_PACKET;
    } else {
        return INVALID_PACKET;
    }
}

bool RXPacket::isChecksumValid() {
    int32_t sum = 0xBDFA;
    for (int i = 0; i < 6; i++) {
        sum += (int8_t)_data[i];
    }
    sum += ((int8_t)_data[POSZ+1] * 256);
    sum += ((int8_t)_data[POSZ]);
    for (int i = 8; i < 16; i++) {
        sum += (int8_t)_data[i];
    }
    if ((int16_t)sum == (((int8_t)_data[CHECKSUM+1] * 256) + (int8_t)_data[CHECKSUM])) {
        return true;
    } else {
        return false;
    }
}