//
//  Robot.cpp
//  
//
//  Created by Rahul Salvi on 7/19/14.
//
//

#include "Robot.h"

Robot::Robot(uint8_t mpuAddr,
             uint8_t pwmU1Addr,
             uint8_t pwmU2Addr):
                _mpu9150(MPU6050(mpuAddr)),
                _pwmU1(Adafruit_PWMServoDriver(pwmU1Addr)),
                _pwmU2(Adafruit_PWMServoDriver(pwmU2Addr)) {}

void Robot::begin() {
    _mpu9150.initialize();
    Serial.print("MPU9150 Initialized: ");
    Serial.println(_mpu9150.getAddress());
    
    _pwmU1.begin();
    Serial.print("PWMU1 Initialized: ");
    Serial.println(_pwmU1.getAddress());
    
    _pwmU2.begin();
    Serial.print("PWMU2 Initialized: ");
    Serial.println(_pwmU2.getAddress());
    
    Serial.println("Robot Initialized");
}

void Robot::setMotion(int8_t velX,
                      int8_t velY,
                      int8_t velZ,
                      int8_t rotX,
                      int8_t rotY,
                      int8_t rotZ,
                      int16_t posZ,
                      int8_t torpedoCtl,
                      int8_t* servoCtl) {
    
    
}

void Robot::stop() {
    for (int i = 0; i < 15; i++) {
        _pwmU1.setPWM(i, 0, 0);
    }
    for (int i = 8; i < 15; i++) {
        _pwmU2.setPWM(i, 0, 0);
    }
}