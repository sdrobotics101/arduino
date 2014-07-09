//
//  TXPacket.cpp
//  
//
//  Created by Rahul Salvi on 6/28/14.
//
//

#include "TXPacket.h"


TXPacket::TXPacket() {
    _data[0] = 0x1D;
    _data[1] = 0x3B;
    for (int i = 2; i < 12; i++) {
        _data[i] = 0;
    }
}

void TXPacket::begin() {
    Serial.println("TXPacket Initialized");
}

void TXPacket::sendPacket(USARTClass serialPort) {
    int16_t checksum = computeChecksum();
    int16_t checksum2 = checksum;
    _data[10] = (uint8_t)checksum >> 8;
    _data[11] = (uint8_t)((checksum2 << 8) >> 8);
    serialPort.write(_data, (size_t)12);
}

int16_t TXPacket::computeChecksum() {
    int16_t sum = 0x1D3B;
    for (int i = 2; i < 10; i++) {
        sum += (uint8_t)_data[i];
    }
    return sum;
}