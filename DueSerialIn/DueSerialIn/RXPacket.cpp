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

void RXPacket::begin() {
    _data = 0;
}

String RXPacket::WhoAmI() {
    return "RXPacket";
}

void RXPacket::set(uint8_t data) {
    _data = data;
}

uint8_t RXPacket::get() {
    return _data; 
}

