//
// TXPacket.cpp 
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
// See 			TXPacket.h and ReadMe.txt for references
//


// Library header
#include "TXPacket.h"

// Code
TXPacket::TXPacket() {
    _header = 0x1D3B;
    _accX = 0;
    _accY = 0;
    _accZ = 0;
    _magX = 0;
    _magY = 0;
    _magZ = 0;
    _pressure = 0;
    _spare = 0;
    _checksum = 0;
}

void TXPacket::setAccX(int8_t accX) {
    _accX = accX;
}

void TXPacket::setAccY(int8_t accY) {
    _accY = accY;
}

void TXPacket::setAccZ(int8_t accZ) {
    _accZ = accZ;
}

void TXPacket::setMagX(int8_t magX) {
    _magX = magX;
}

void TXPacket::setMagY(int8_t magY) {
    _magY = magY;
}

void TXPacket::setMagZ(int8_t magZ) {
    _magZ = magZ;
}

void TXPacket::setPressure(int8_t pressure) {
    _pressure = pressure;
}

void TXPacket::setSpare(int8_t spare) {
    _spare = spare;
}

int8_t TXPacket::accX() {
    return _accX;
}

int8_t TXPacket::accY() {
    return _accY;
}

int8_t TXPacket::accZ() {
    return _accZ;
}

int8_t TXPacket::magX() {
    return _magX;
}

int8_t TXPacket::magY() {
    return _magY;
}

int8_t TXPacket::magZ() {
    return _magZ;
}

int8_t TXPacket::pressure() {
    return _pressure;
}

int8_t TXPacket::spare() {
    return _spare;
}

void TXPacket::sendPacket() {
    int16_t txPacket[6];
    txPacket[0] = _header;
    txPacket[1] = (int16_t)((_accX << 8) + _accY);
    txPacket[2] = (int16_t)((_accZ << 8) + _magX);
    txPacket[3] = (int16_t)((_magY << 8) + _magZ);
    txPacket[4] = (int16_t)((_pressure << 8) + _spare);
    txPacket[5] = computeChecksum();
    Serial1.write(txPacket, 12);
}

int16_t TXPacket::computeChecksum() {
    int16_t checksum;
    checksum =
        accX() +
        accY() +
        accZ() +
        magX() +
        magY() +
        magZ() +
        pressure() +
        spare() +
        0x1D3B;
    return checksum;
}













