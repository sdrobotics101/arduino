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
    
}

//0xBDFA
