//
//  Robot.h
//  
//
//  Created by Rahul Salvi on 7/19/14.
//
//

#ifndef ____Robot__
#define ____Robot__

#include "Arduino.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Adafruit_PWMServoDriver.h"

class Robot {
public:
    Robot(uint8_t mpuAddr = 0x68, pwmU1Addr = 0x79, pwmU2Addr = 0x71);
    void begin();
    void setMotion(int8_t velX,
                   int8_t velY,
                   int8_t velZ,
                   int8_t rotX,
                   int8_t rotY,
                   int8_t rotZ,
                   int16_t posZ,
                   int8_t torpedoCtl,
                   int8_t[6] servoCtl);
    void stop();
private:
    MPU6050 _mpu9150;
    Adafruit_PWMServoDriver _pwmU1;
    Adafruit_PWMServoDriver _pwmU2;
};


#endif /* defined(____Robot__) */