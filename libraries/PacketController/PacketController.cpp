//
//  PacketController.cpp
//  
//
//  Created by Rahul Salvi on 6/28/14.
//
//

#include "PacketController.h"

PacketController::PacketController(USARTClass &rxSerialPort, USARTClass &txSerialPort) : _rxSerialPort(rxSerialPort), _txSerialPort(txSerialPort) {}

void PacketController::begin(int baudRate) {
    
    Serial.begin(baudRate);
    
    if (Serial) {
        Serial.println("Serial Initialized");
    } else {
        Serial.println("Serial Failed to Initalize"); //How can this actually output?
    }
    
    _rxSerialPort.begin(baudRate);
    _txSerialPort.begin(baudRate);
    
    if (_rxSerialPort) {
        Serial.print("RX Serial Port Initialized: ");
        if (&_rxSerialPort == &Serial1) {
            Serial.println("Serial 1");
        } else if (&_rxSerialPort == &Serial2) {
            Serial.println("Serial 2");
        } else if (&_rxSerialPort == &Serial3) {
            Serial.println("Serial 3");
        } else {
            Serial.println("Unknown");
        }
    } else {
        Serial.println("RX Serial Port Failed to Initialize");
    }
    
    if (_txSerialPort) {
        Serial.print("TX Serial Port Initialized: ");
        if (&_txSerialPort == &Serial1) {
            Serial.println("Serial 1");
        } else if (&_txSerialPort == &Serial2) {
            Serial.println("Serial 2");
        } else if (&_txSerialPort == &Serial3) {
            Serial.println("Serial 3");
        } else {
            Serial.println("Unknown");
        }
    } else {
        Serial.println("RX Serial Port Failed to Initialize");
    }
    
    _rxPacket.begin();
    _txPacket.begin();
    
    Serial.println("PacketController Initialized");
}

PacketStatus PacketController::listen() {
    if (_rxSerialPort.available() > 19) {
        for (int i = 0; i < 20; i++) {
            if (_rxSerialPort.read() == 0xFA) {
                if (_rxSerialPort.peek() == 0xBD) {
                    _rxSerialPort.read();
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

void PacketController::set(TXIndex index, int8_t value) {
    _txPacket._data[index] = value;
}

int8_t PacketController::get(RXIndex index) {
    return _rxPacket._data[index];
}

int16_t PacketController::getPosZ() {
    int16_t posZ = _rxPacket._data[POSZ+1];
    posZ *= 256;
    posZ += _rxPacket._data[POSZ];
    return posZ;
}