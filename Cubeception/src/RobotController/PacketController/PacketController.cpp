//
//  PacketController.cpp
//  
//
//  Created by Rahul Salvi on 6/28/14.
//
//

#include "PacketController.h"

/**
 *  Constructor
 *
 *  @param rxSerialPort Which serial port to read from
 *  @param txSerialPort Which serial port to send to
 *
 *  @return Nothing
 */
PacketController::PacketController(USARTClass &rxSerialPort,
                                   USARTClass &txSerialPort) :
                                        _rxSerialPort(rxSerialPort),
                                        _txSerialPort(txSerialPort)
{}

/**
 *  Initializes PacketController
 *
 *  @param baudRate What baud rate to run at
 */
void PacketController::begin(int baudRate) {
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

/**
 *  Listens for a packet header
 *
 *  @return PacketStatus indicating validity of packet
 */
PacketStatus PacketController::listen() {
    if (_rxSerialPort.available() > 12) {
        for (int i = 0; i < 13; i++) {
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


void PacketController::setS8(TXIndexS8 index, int8_t value) {
    _txPacket._data[index] = value;
}

void PacketController::setU8(TXIndexU8 index, uint8_t value) {
	_txPacket._data[index] = value;
}

void PacketController::setS16(TXIndexS16 index, int16_t value) {
	_txPacket._data[index+1] = (floor(value / 256));
    _txPacket._data[index] = (value % 256);
}

void PacketController::setU16(TXIndexU16 index, uint16_t value) {
	_txPacket._data[index+1] = (floor(value / 256));
    _txPacket._data[index] = (value % 256);
}

int8_t PacketController::getS8(RXIndexS8 index) {
    return (int8_t)_rxPacket._data[index];
}

uint8_t PacketController::getU8(RXIndexU8 index) {
	return (uint8_t)_rxPacket._data[index];
}

int16_t PacketController::getS16(RXIndexS16 index) {
    int16_t val = _rxPacket._data[index+1];
    val *= 256;
    val += _rxPacket._data[index];
    return (int16_t)val;
}

uint16_t PacketController::getU16(RXIndexU16 index) {
	uint16_t val = _rxPacket._data[index+1];
    val *= 256;
    val += _rxPacket._data[index];
    return (uint16_t)val;
}