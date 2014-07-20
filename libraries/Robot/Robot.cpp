//
//  Robot.cpp
//  
//
//  Created by Rahul Salvi on 7/19/14.
//
//

#include "Robot.h"

Robot::Robot(uint8_t mpuAddr, uint8_t pwmU1Addr, uint8_t pwmU2Addr) : _mpu9150(MPU6050(mpuAddr)), _pwmU1(Adafruit_PWMServoDriver(pwmU1Addr)), _pwmU2(Adafruit_PWMServoDriver(pwmU2Addr)) {}

void Robot::begin() {
    _mpu9150.initialize();
    Serial.println("MPU9150 Initialized");
    _pwmU1.begin();
    Serial.println("PWMU1 Initialized");
    _pwmU2.begin();
    Serial.println("PWMU2 Initialized");
}