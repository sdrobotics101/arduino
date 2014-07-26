//
//  MS5541C.h
//  
//
//  Created by Rahul Salvi on 7/24/14.
//
//

#ifndef ____MS5541C__
#define ____MS5541C__

#include "Arduino.h"

class MS5541C {
public:
    MS5541C();
    void begin();
    void reset();
    uint16_t getPressure();
    int16_t getTemperature();
private:
    void getW1();
    void getW2();
    void getW3();
    void getW4();
    
    void computeConstants();
    
    int16_t _W1;
    int16_t _W2;
    int16_t _W3;
    int16_t _W4;
    
    int16_t _C1;
    int16_t _C2;
    int16_t _C3;
    int16_t _C4;
    int16_t _C5;
    int16_t _C6;
    
};

#endif /* defined(____MS5541C__) */