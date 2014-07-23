//
//  RXPacket.cpp
//  
//
//  Created by Rahul Salvi on 6/28/14.
//
//

#include "RXPacket.h"

/**
 *  Default constructor
 */
RXPacket::RXPacket() {
    for (int i = 0; i < 18; i++) {
        _data[i] = 0;
    }
}

/**
 *  Initializes RXPacket
 */
void RXPacket::begin() {
    Serial.println("RXPacket Initialized");
}

/**
 *  Reads in a packet from the buffer
 *
 *  @param serialPort Which serial port to read from
 *
 *  @return PacketStatus indicating validity of packet
 */
PacketStatus RXPacket::readPacket(USARTClass serialPort) {
    serialPort.readBytes(_data, 18);
    uint16_t checksum = _data[CHECKSUM+1] << 8;
    checksum += _data[CHECKSUM];
    if (computeChecksum() == checksum) {
        return VALID_PACKET;
    } else {
        return INVALID_PACKET;
    }
}

/**
 *  Computes checksum
 *
 *  @return The computed checksum
 */
uint16_t RXPacket::computeChecksum() {
    uint16_t sum = 0xFA;
    sum += 0xBD;
    for (int i = 0; i < CHECKSUM; i++) {
        sum += (uint8_t)_data[i];
    }
    return sum;
}
