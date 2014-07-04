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

void TXPacket::sendPacket() {
    int16_t checksum = computeChecksum();
    int16_t checksum2 = checksum;
    _data[10] = (int8_t)checksum >> 8;
    _data[11] = (int8_t)((checksum2 << 8) >> 8);
    Serial1.write(_data, 12);
}

int16_t TXPacket::computeChecksum() {
    int16_t sum = 0x1D3B;
    for (int i = 2; i < 10; i++) {
        sum += _data[i];
    }
    return sum;
}