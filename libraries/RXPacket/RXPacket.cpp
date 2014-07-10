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
    int16_t sum = 0xBDFA;
    for (int i = 0; i < 6; i++) {
        sum += (int8_t)_data[i];
    }
    sum += ((int8_t)_data[POSZ] * 256);
    for (int i = 7; i < 16; i++) {
        sum += (int8_t)_data[i];
    }
    if (sum == (((int8_t)_data[CHECKSUM] * 256) + (int8_t)_data[CHECKSUM+1])) {
        return true;
    } else {
        return false;
    }
}