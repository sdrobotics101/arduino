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
    
    txPacket.setAccX(7);
    txPacket.setAccY(6);
    txPacket.setAccZ(5);
    txPacket.setMagX(4);
    txPacket.setMagY(3);
    txPacket.setMagZ(2);
    txPacket.setPressure(1);
    txPacket.setSpare(0);
    txPacket.sendPacket();
    
    delay(100);
#else
    rxPacket.setRotX(5);
    rxPacket.setRotY(4);
    rxPacket.setRotZ(3);
    rxPacket.setVelX(2);
    rxPacket.setVelY(1);
    rxPacket.setVelZ(0);
    
    rxPacket.sendPacket();
    
    if (txPacket.readPacket()) {
        Serial.print(txPacket.accX());
        Serial.print(txPacket.accY());
        Serial.print(txPacket.accZ());
    }
    
#endif
}
