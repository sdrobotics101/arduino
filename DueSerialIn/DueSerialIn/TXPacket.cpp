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
    
}

void TXPacket::begin() {
    _data = 0;
}

String TXPacket::WhoAmI() {
    return "TXPacket";
}

void TXPacket::set(uint8_t data) {
    _data = data;
}

uint8_t TXPacket::get() {
    return _data; 
}

