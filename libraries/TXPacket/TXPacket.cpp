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
    _data[10] = (int8_t)(floor(checksum / 256));
    _data[11] = (int8_t)(checksum % 256);
    serialPort.write(_data, (size_t)12);
}

int16_t TXPacket::computeChecksum() {
    int16_t sum = 0x1D3B;
    for (int i = 2; i < 10; i++) {
        sum += (int8_t)_data[i];
    }
    return sum;
}