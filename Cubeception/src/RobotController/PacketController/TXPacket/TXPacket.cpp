//
//  TXPacket.cpp
//  
//
//  Created by Rahul Salvi on 6/28/14.
//
//

#include "TXPacket.h"


/**
 *  Constructor
 */
TXPacket::TXPacket() {
    _data[0] = 0x3B;
    _data[1] = 0x1D;
    for (int i = 2; i < 12; i++) {
        _data[i] = (int8_t)0;
    }
}

/**
 *  Initializes TXPacket
 */
void TXPacket::begin() {
    Serial.println("TXPacket Initialized");
}

/**
 *  Sends a packet to the serial port
 *
 *  @param serialPort Which serial port to send to
 */
void TXPacket::sendPacket(USARTClass serialPort) {
    uint16_t checksum = computeChecksum();
    _data[11] = (floor(checksum / 256));
    _data[10] = (checksum % 256);
    serialPort.write(_data, 12);
}

/**
 *  Computes the checksum
 *
 *  @return The computed checksum
 */
uint16_t TXPacket::computeChecksum() {
    uint16_t sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += (uint8_t)_data[i];
    }
    return sum;
}