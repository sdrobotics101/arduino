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

/**
 *  Sends a packet to the serial port
 */
void PacketController::send() {
    _txPacket.sendPacket(_txSerialPort);
}

/**
 *  Sets a value in the packet to be sent
 *
 *  @param index What value to set
 *  @param value What to set value to
 */
void PacketController::set(TXIndex index, int8_t value) {
    _txPacket._data[index] = value;
}

/**
 *  Gets a single 8-bit value from the packet received
 *
 *  @param index Which value to get
 *
 *  @return Value of specified index
 */
int8_t PacketController::get8(RXIndex index) {
    return _rxPacket._data[index];
}

/**
 *  Gets a single 16-bit value from the packet received
 *
 *  @param index Which value to get
 *
 *  @return Value of specified index
 */
int16_t PacketController::get16(RXIndex index) {
    int16_t val = _rxPacket._data[index+1];
    val *= 256;
    val += _rxPacket._data[index];
    return val;
}