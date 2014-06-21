// 
// DuePiSim 
//
// Simulate RPi Packets using Due
// Developed with [embedXcode](http://embedXcode.weebly.com)
// 
// Author	 	Rahul Salvi
// 				Rahul Salvi
//
// Date			6/21/14 3:50 PM
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

#define TESTMODE 1

RXPacket rxPacket;
TXPacket txPacket;

void setup() {
    Serial1.begin(115200);
    Serial.begin(115200);
}

void loop() {
#if defined(TESTMODE)
    if (rxPacket.readPacket()) {
        Serial.print(rxPacket.velX());
        Serial.print(rxPacket.velY());
        Serial.print(rxPacket.velZ());
        Serial.print(rxPacket.rotX());
        Serial.print(rxPacket.rotY());
        Serial.print(rxPacket.rotZ());
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
    
    delay(100);
#else
    rxPacket.setRotX(0);
    rxPacket.setRotY(1);
    rxPacket.setRotZ(2);
    rxPacket.setVelX(3);
    rxPacket.setVelY(4);
    rxPacket.setVelZ(5);
    
    rxPacket.sendPacket();
    
    if (txPacket.readPacket()) {
        Serial.print(txPacket.accX());
        Serial.print(txPacket.accY());
        Serial.print(txPacket.accZ());
    }
    
#endif
}
