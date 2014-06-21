//
// RXPacket.cpp 
// Class library C++ code
// ----------------------------------
// Developed with embedXcode 
// http://embedXcode.weebly.com
//
// Project 		DueSerialIn
//
// Created by 	Rahul Salvi, 6/19/14 11:00 PM
// 				Rahul Salvi
//
// Copyright 	© Rahul Salvi, 2014
// License     <#license#>
//
// See 			RXPacket.h and ReadMe.txt for references
//


// Library header
#include "RXPacket.h"

// Code
RXPacket::RXPacket() {
    _header = 0;
    _velX = 0;
    _velY = 0;
    _velZ = 0;
    _rotX = 0;
    _rotY = 0;
    _rotZ = 0;
    _posZ = 0;
    _torpedoControl = 0;
    _servoControl[6] = 0;
    _spare = 0;
    _checksum = 0;
}

void RXPacket::setVelX(int8_t velX) {
    _velX = velX;
}

void RXPacket::setVelY(int8_t velY){
    _velY = velY;
}

void RXPacket::setVelZ(int8_t velZ) {
    _velZ = velZ;
}

void RXPacket::setRotX(int8_t rotX) {
    _rotX = rotX;
}

void RXPacket::setRotY(int8_t rotY) {
    _rotY = rotY;
}

void RXPacket::setRotZ(int8_t rotZ) {
    _rotZ = rotZ;
}

void RXPacket::setPosZ(uint16_t posZ) {
    _posZ = posZ;
}

void RXPacket::setTorpedoControl(int8_t torpedoControl) {
    _torpedoControl = torpedoControl;
}

void RXPacket::setServoControl(int8_t servoControl[6]) {
    for (int i = 0; i < 6; i++) {
        _servoControl[i] = servoControl[i];
    }
}

void RXPacket::setSpare(int8_t spare) {
    _spare = spare;
}

void RXPacket::setChecksum(int16_t checksum) {
    _checksum = checksum;
}

int8_t RXPacket::velX() {
    return _velX;
}

int8_t RXPacket::velY() {
    return _velY;
}
int8_t RXPacket::velZ() {
    return _velZ;
}

int8_t RXPacket::rotX() {
    return _rotX;
}

int8_t RXPacket::rotY() {
    return _rotY;
}

int8_t RXPacket::rotZ() {
    return _rotZ;
}

uint16_t RXPacket::posZ() {
    return _posZ;
}

int8_t RXPacket::torpedoControl() {
    return _torpedoControl;
}

int8_t* RXPacket::servoControl() {
    return _servoControl;
}

int8_t RXPacket::spare() {
    return _spare;
}

int16_t RXPacket::checksum() {
    return _checksum;
}

bool RXPacket::readPacket() {
    if (Serial1.available() > 19) {
        _header = (Serial1.read() << 8) + Serial1.read();
        if (_header = 0xBDFA) {
            setVelX(Serial1.read());
            setVelY(Serial1.read());
            setVelZ(Serial1.read());
            setRotX(Serial1.read());
            setRotY(Serial1.read());
            setRotZ(Serial1.read());
            setPosZ((Serial1.read() << 8) + Serial1.read());
            setTorpedoControl(Serial1.read());
            int8_t servoControl[6];
            for (int i = 0; i < 6; i++) {
                servoControl[i] = Serial1.read();
            }
            setServoControl(servoControl);
            setSpare(Serial1.read());
            setChecksum((Serial1.read() << 8) + Serial1.read());
            return isChecksumValid();
        }
    }
}

void RXPacket::sendPacket() {
    Serial1.write(0xBDFA);
    Serial1.write(_velX);
    Serial1.write(_velY);
    Serial1.write(_velZ);
    Serial1.write(_rotX);
    Serial1.write(_rotY);
    Serial1.write(_rotZ);
    Serial1.write(_posZ);
    Serial1.write(_torpedoControl);
    for (int i = 0; i < 6; i++) {
        Serial1.write(_servoControl[i]);
    }
    Serial1.write(_spare);
    Serial1.write(computeChecksum());
}

bool RXPacket::isChecksumValid() {
    int16_t checksum =
        velX() +
        velY() +
        velZ() +
        rotX() +
        rotY() +
        rotZ() +
        posZ() +
        torpedoControl() +
        spare() +
        0xBDFA;
    for (int i = 0; i < 6; i++) {
        checksum += _servoControl[i];
    }
    if (checksum == _checksum) {
        return true;
    } else {
        return false;
    }
}

int16_t RXPacket::computeChecksum() {
    int16_t checksum =
    velX() +
    velY() +
    velZ() +
    rotX() +
    rotY() +
    rotZ() +
    posZ() +
    torpedoControl() +
    spare() +
    0xBDFA;
    for (int i = 0; i < 6; i++) {
        checksum += _servoControl[i];
    }
    return checksum;
}
