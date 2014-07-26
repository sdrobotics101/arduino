//
//  MS5541C.cpp
//  
//
//  Created by Rahul Salvi on 7/24/14.
//
//

#include "MS5541C.h"

MS5541C::MS5541C() : _C1(0),
                     _C2(0),
                     _C3(0),
                     _C4(0),
                     _C5(0),
                     _C6(0)
{
    reset();
}

MS5541C::begin() {
    reset();
    getW1();
    getW2();
    getW3();
    getW4();
}

MS5541C::reset() {
    SPI.setDataMode(SPI_MODE0);
    SPI.transfer(0x15);
    SPI.transfer(0x55);
    SPI.transfer(0x40);
}

MS5541C::getPressure() {
    
}

MS5541C::getTemperature() {
    
}

MS5541C::getW1() {
    reset();
    SPI.transfer(0x1D);
    SPI.transfer(0x50);
    SPI.setDataMode(SPI_MODE1);
    
    _W1 = SPI.transfer(0x00);
    _W1 = _W1 << 8;
    
    _W1 = _W1 | SPI.transfer(0x00);
    
}

MS5541C::getW2() {
    reset();
    SPI.transfer(0x1D);
    SPI.transfer(0x60);
    SPI.setDataMode(SPI_MODE1);
    
    _W2 = SPI.transfer(0x00);
    _W2 = _W2 << 8;
    
    _W2 = _W2 | SPI.transfer(0x00);
}

MS5541C::getW3() {
    reset();
    SPI.transfer(0x1D);
    SPI.transfer(0x90);
    SPI.setDataMode(SPI_MODE1);
    
    _W3 = SPI.transfer(0x00);
    _W3 = _W3 << 8;
    
    _W3 = _W3 | SPI.transfer(0x00);
}

MS5541C::getW4() {
    reset();
    SPI.transfer(0x1D);
    SPI.transfer(0xA0);
    SPI.setDataMode(SPI_MODE1);
    
    _W4 = SPI.transfer(0x00);
    _W4 = _W4 << 8;
    
    _W4 = _W4 | SPI.transfer(0x00);
}

MS5541C::computeConstants() {
    _C1 = _W1 >> 3 & 0x1FFF;
    _C2 = ((_W1 & 0x07) << 10) | ((_W2 >> 6) & 0x03FF);
    _C3 = (_W3 >> 6) & 0x03FF;
    _C4 = (_W4 >> 7) & 0x07FF;
    _C5 = ((_W2 & 0x003F) << 6) | (_W3 & 0x003F);
    _C6 = _W4 & 0x007F;
    reset();
}