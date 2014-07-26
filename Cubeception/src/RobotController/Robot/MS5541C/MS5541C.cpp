//
//  MS5541C.cpp
//  
//
//  Created by Rahul Salvi on 7/24/14.
//
//

#include "MS5541C.h"

MS5541C::MS5541C() : _D1(0),
                     _D2(0),

                     _W1(0),
                     _W2(0),
                     _W3(0),
                     _W4(0),

                     _C1(0),
                     _C2(0),
                     _C3(0),
                     _C4(0),
                     _C5(0),
                     _C6(0) {}

void MS5541C::begin() {
    getW1();
    getW2();
    getW3();
    getW4();
    
    computeConstants();
    
    Serial.println("MS5541C Initialized");
    
    Serial.print("MS5541C W: ");
    Serial.print(_W1); Serial.print(" ");
    Serial.print(_W2); Serial.print(" ");
    Serial.print(_W3); Serial.print(" ");
    Serial.print(_W4); Serial.print(" ");
    Serial.println("");
    
    Serial.print("MS5541C C: ");
    Serial.print(_C1); Serial.print(" ");
    Serial.print(_C2); Serial.print(" ");
    Serial.print(_C3); Serial.print(" ");
    Serial.print(_C4); Serial.print(" ");
    Serial.print(_C5); Serial.print(" ");
    Serial.print(_C6); Serial.print(" ");
    Serial.println("");
    
}

void MS5541C::reset() {
    SPI.setDataMode(SPI_MODE0);
    SPI.transfer(0x15);
    SPI.transfer(0x55);
    SPI.transfer(0x40);
}

void MS5541C::queueD1() {
    reset();
    SPI.transfer(0x0F);
    SPI.transfer(0x40);
}

void MS5541C::queueD2() {
    reset();
    SPI.transfer(0x0F);
    SPI.transfer(0x20);
}

void MS5541C::readD1() {
    SPI.setDataMode(SPI_MODE1);
    _D1 = SPI.transfer(0x00);
    _D1 = _D1 << 8;
    
    _D1 = _D1 | SPI.transfer(0x00);
}

void MS5541C::readD2() {
    SPI.setDataMode(SPI_MODE1);
    _D2 = SPI.transfer(0x00);
    _D2 = _D2 << 8;
    
    _D2 = _D2 | SPI.transfer(0x00);
}

int16_t MS5541C::getPressure() {
    int16_t UT1 = (8 * _C5) + 10000;
    int16_t dT = _D2 - UT1;
    int16_t OFF = _C2 + (((_C4 - 250) * dT) / 4096) + 10000;
    int16_t SENS = (_C1 / 2) + (((_C3 + 200) * dT) / 8192) + 3000;
    return ((SENS * (_D1 - OFF)) / 4096) + 1000;
}

int16_t MS5541C::getTemperature() {
    int16_t UT1 = (8 * _C5) + 10000;
    int16_t dT = _D2 - UT1;
    return 200 + ((dT * (_C6 + 100)) / 2048);
}

void MS5541C::getW1() {
    reset();
    SPI.transfer(0x1D);
    SPI.transfer(0x50);
    SPI.setDataMode(SPI_MODE1);
    
    _W1 = SPI.transfer(0x00);
    _W1 = _W1 << 8;
    
    _W1 = _W1 | SPI.transfer(0x00);
    
}

void MS5541C::getW2() {
    reset();
    SPI.transfer(0x1D);
    SPI.transfer(0x60);
    SPI.setDataMode(SPI_MODE1);
    
    _W2 = SPI.transfer(0x00);
    _W2 = _W2 << 8;
    
    _W2 = _W2 | SPI.transfer(0x00);
}

void MS5541C::getW3() {
    reset();
    SPI.transfer(0x1D);
    SPI.transfer(0x90);
    SPI.setDataMode(SPI_MODE1);
    
    _W3 = SPI.transfer(0x00);
    _W3 = _W3 << 8;
    
    _W3 = _W3 | SPI.transfer(0x00);
}

void MS5541C::getW4() {
    reset();
    SPI.transfer(0x1D);
    SPI.transfer(0xA0);
    SPI.setDataMode(SPI_MODE1);
    
    _W4 = SPI.transfer(0x00);
    _W4 = _W4 << 8;
    
    _W4 = _W4 | SPI.transfer(0x00);
}

void MS5541C::computeConstants() {
    _C1 = _W1 >> 3 & 0x1FFF;
    _C2 = ((_W1 & 0x07) << 10) | ((_W2 >> 6) & 0x03FF);
    _C3 = (_W3 >> 6) & 0x03FF;
    _C4 = (_W4 >> 7) & 0x07FF;
    _C5 = ((_W2 & 0x003F) << 6) | (_W3 & 0x003F);
    _C6 = _W4 & 0x007F;
    reset();
}