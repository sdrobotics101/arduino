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
    void begin();
    String WhoAmI();
    void set(uint8_t data);
    uint8_t get();
  
private:
    
};

#endif
