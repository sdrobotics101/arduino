// 
// Development 
//
// Develop Libraries for Arduino
// Developed with [embedXcode](http://embedXcode.weebly.com)
// 
// Author	 	Rahul Salvi
// 				Rahul Salvi
//
// Date			6/26/14 6:45 PM
// Version		<#version#>
// 
// Copyright	© Rahul Salvi, 2014
// License		<#license#>
//
// See			ReadMe.txt for references
//

// Core library for code-sense
#if defined(WIRING) // Wiring specific
#include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#include "WProgram.h"   
#elif defined(MICRODUINO) // Microduino specific
#include "Arduino.h"
#elif defined(MPIDE) // chipKIT specific
#include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad MSP430, Stellaris and Tiva, Experimeter Board FR5739 specific
#include "Energia.h"
#elif defined(TEENSYDUINO) // Teensy specific
#include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#include "Arduino.h"
#else // error
#error Platform not defined
#endif

#include "RXPacket.h"
#include "TXPacket.h"
#include "PacketController.h"

PacketController controller;

void setup() {
    Serial.begin(115200);
}

void loop() {
    controller.set(ACCX, 0);
    controller.set(ACCY, 1);
    controller.set(ACCZ, 2);
    controller.set(MAGX, 3);
    controller.set(MAGY, 4);
    controller.set(MAGZ, 5);
    controller.set(PRESSURE, 6);
    controller.set(TXSPARE, 7);
    
    controller.send();
    
    if (Serial1.available() > 0) {
        Serial.write(Serial1.read());
    }
    
}









