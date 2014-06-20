// 
// DueSerialIn 
//
// Receive packets using Due
// Developed with [embedXcode](http://embedXcode.weebly.com)
// 
// Author	 	Rahul Salvi
// 				Rahul Salvi
//
// Date			6/19/14 10:54 PM
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

#include "TXPacket.h"
#include "RXPacket.h"

RXPacket rxPacket;
TXPacket txPacket;

void setup() {
    Serial1.begin(115200);
}

void loop() {
    if (rxPacket.readPacket()) {
        //run actuators with inputs
    } else {
        //run actuators to hold position
    }
    
    txPacket.setAccX(0);
    txPacket.setAccY(1);
    txPacket.setAccZ(2);
    txPacket.setMagX(3);
    txPacket.setMagY(4);
    txPacket.setMagZ(5);
    txPacket.setPressure(6);
    txPacket.setSpare(7);
    
    txPacket.sendPacket();
    
}
