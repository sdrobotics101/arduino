//
//  PacketController.cpp
//  
//
//  Created by Rahul Salvi on 6/28/14.
//
//

#include "PacketController.h"

PacketController::PacketController() {
    Serial1.begin(115200);
}

PacketStatus PacketController::listen() {
    if (Serial1.available() > 19) {
        for (int i = 0; i < 20; i++) {
            if (Serial1.read() == 0xBD) {
                if (Serial1.read() == 0xFA) {
                    return _rxPacket.readPacket();
                }
            }
        }
        return NO_HEADER_FOUND;
    } else {
        return NOT_ENOUGH_DATA;
    }
}

void PacketController::send() {
    _txPacket.sendPacket();
}

void PacketController::set(TXIndex index, uint8_t value) {
    _txPacket._data[index] = value;
}

int8_t PacketController::get(RXIndex index) {
    return _rxPacket._data[index];
}

int16_t PacketController::getPosZ() {
    int16_t posZ = _rxPacket._data[POSZ];
    posZ << 8;
    posZ += _rxPacket._data[POSZ+1];
    return posZ;
}