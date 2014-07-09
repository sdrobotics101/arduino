//
//  PacketController.cpp
//  
//
//  Created by Rahul Salvi on 6/28/14.
//
//

#include "PacketController.h"

PacketController::PacketController(USARTClass rxSerialPort, USARTClass txSerialPort) : _rxSerialPort(rxSerialPort), _txSerialPort(txSerialPort) {}

void PacketController::begin(int baudRate) {
    Serial.begin(baudRate);
    _rxSerialPort.begin(baudRate);
    _txSerialPort.begin(baudRate);
    _rxPacket.begin();
    _txPacket.begin();
    Serial.println("PacketController Initialized");
}

PacketStatus PacketController::listen() {
    if (_rxSerialPort.available() > 19) {
        for (int i = 0; i < 20; i++) {
            if (_rxSerialPort.read() == 0xBD) {
                if (_rxSerialPort.read() == 0xFA) {
                    return _rxPacket.readPacket(_rxSerialPort);
                }
            }
        }
        return NO_HEADER_FOUND;
    } else {
        return NOT_ENOUGH_DATA;
    }
}

void PacketController::send() {
    _txPacket.sendPacket(_txSerialPort);
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