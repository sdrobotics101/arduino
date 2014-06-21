//
// File			TXPacket.h
// Class library header
//
// Details		<#details#>
//
// Project	 	DueSerialIn
// Developed with [embedXcode](http://embedXcode.weebly.com)
//
// Author		Rahul Salvi
// 				Rahul Salvi
//
// Date			6/19/14 11:00 PM
// Version		<#version#>
//
// Copyright	© Rahul Salvi, 2014
// License	    <#license#>
//
// See			ReadMe.txt for references
//


// Core library - IDE-based
#if defined(WIRING) // Wiring specific
#include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#include "WProgram.h"
#elif defined(MPIDE) // chipKIT specific
#include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad MSP430 G2 and F5529, Stellaris and Tiva, Experimeter Board FR5739 specific
#include "Energia.h"
#elif defined(MICRODUINO) // Microduino specific
#include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
#include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#include "Arduino.h"
#else // error
#error Platform not defined
#endif // end IDE

#ifndef TXPacket_h

#define TXPacket_h


class TXPacket {
  
public:
    TXPacket();
    
    void setAccX(int8_t accX);
    void setAccY(int8_t accY);
    void setAccZ(int8_t accZ);
    void setMagX(int8_t magX);
    void setMagY(int8_t magY);
    void setMagZ(int8_t magZ);
    void setPressure(int8_t pressure);
    void setSpare(int8_t spare);
    
    int8_t accX();
    int8_t accY();
    int8_t accZ();
    int8_t magX();
    int8_t magY();
    int8_t magZ();
    int8_t pressure();
    int8_t spare();
    
    void sendPacket();
    bool readPacket();
  
private:
    uint16_t _header;
    int8_t _accX;
    int8_t _accY;
    int8_t _accZ;
    int8_t _magX;
    int8_t _magY;
    int8_t _magZ;
    int8_t _pressure;
    int8_t _spare;
    int16_t _checksum;
    
    int16_t computeChecksum();
    bool isChecksumValid();
    
};

#endif
