//
// File			RXPacket.h
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

#ifndef RXPacket_h

#define RXPacket_h


class RXPacket {
  
public:
    RXPacket();
    
    void setVelX(int8_t velX);
    void setVelY(int8_t velY);
    void setVelZ(int8_t velZ);
    void setRotX(int8_t rotX);
    void setRotY(int8_t rotY);
    void setRotZ(int8_t rotZ);
    void setPosZ(uint16_t posZ);
    void setTorpedoControl(int8_t torpedoControl);
    void setServoControl(int8_t servoControl[6]);
    void setSpare(int8_t spare);
    void setChecksum(int16_t checksum);
    
    int8_t velX();
    int8_t velY();
    int8_t velZ();
    int8_t rotX();
    int8_t rotY();
    int8_t rotZ();
    uint16_t posZ();
    int8_t torpedoControl();
    int8_t* servoControl();
    int8_t spare();
    int16_t checksum();
    
    bool readPacket();
    void sendPacket();
    
private:
    uint16_t _header;
    int8_t _velX;
    int8_t _velY;
    int8_t _velZ;
    int8_t _rotX;
    int8_t _rotY;
    int8_t _rotZ;
    uint16_t _posZ;
    int8_t _torpedoControl;
    int8_t _servoControl[6];
    int8_t _spare;
    int16_t _checksum;
    
    bool isChecksumValid();
    int16_t computeChecksum();
};

#endif
