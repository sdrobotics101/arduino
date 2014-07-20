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
                      int8_t servoCtl[6])
{
    stabilize();
    //Write velX, velY, rotX, rotY
    //Other commands
    
}

void Robot::stop() {
    for (int i = 0; i < 15; i++) {
        _pwmU1.setPWM(i, 0, 0);
    }
    for (int i = 8; i < 15; i++) {
        _pwmU2.setPWM(i, 0, 0);
    }
}

double Robot::updateMPU9150() {
    _mpu9150.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
}

double Robot::getDispX(double dt) {
    double accAngle = atan2(accY, sqrt(pow(accX, 2) + pow(accZ, 2)));
    accAngle *= (180 / PI);
    
    double gyroAngle = stateGyroX + ((gyroX/131) * dt);
    
    return (0.98 * gyroAngle) + (0.02 * accAngle);
}

double Robot::getDispY(double dt) {
    double accAngle = atan2(accX, sqrt(pow(accY, 2) + pow(accZ, 2)));
    accAngle *= (180 / PI);
    
    double gyroAngle = stateGyroY + ((gyroY/131) * dt);
    
    return (0.98 * gyroAngle) + (0.02 * accAngle);
}

double Robot::getDispZ(double dt) {
    return stateGyroZ + (gyroZ/131) * dt);
}

void Robot::stabilize() {
    double dt = micros() - time;
    time = micros();
    
    updateMPU9150();
    
    double outputX = pidOutputX.compute(getDispX(dt));
    double outputY = pidOutputY.compute(getDispY(dt));
    
    double z1 = (-outputX) + (-outputY);
    double z2 = ( outputX) + (-outputY);
    double z3 = ( outputX) + ( outputY);
    double z4 = (-outputX) + ( outputY);
}

void Robot::setMotorU1(MotorU1 motor, uint16_t value) {
    _pwmU1.setPWM(motor, motor*256, ((motor*256) + value) % 4096);
}

void Robot::setMotorU2(MotorU2 motor, uint16_t value) {
    _pwmU2.setPWM(motor, motor*256, ((motor*256) + value) % 4096);
}

















