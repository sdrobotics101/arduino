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
             uint8_t pwmU2Addr,
             
             double pidOutputXKP,
             double pidOutputXKI,
             double pidOutputXKD,
             double pidOutputXKF,
             
             double pidOutputYKP,
             double pidOutputYKI,
             double pidOutputYKD,
             double pidOutputYKF,
             
             double pidDepthKP,
             double pidDepthKI,
             double pidDepthKD,
             double pidDepthKF,
             
             double combinerConstant) :

                _mpu9150(mpuAddr),
                _pwmU1(pwmU1Addr),
                _pwmU2(pwmU2Addr),

                _pidOutputX(pidOutputXKP,
                            pidOutputXKI,
                            pidOutputXKD,
                            pidOutputXKF),

                _pidOutputY(pidOutputYKP,
                            pidOutputYKI,
                            pidOutputYKD,
                            pidOutputYKF),

                _pidDepth(  pidDepthKP,
                            pidDepthKI,
                            pidDepthKD,
                            pidDepthKF),

                _combinerConstant(combinerConstant) {}

void Robot::begin() {
    _mpu9150.initialize();
    Serial.print("MPU9150 Initialized: ");
    Serial.println(_mpu9150.getAddress(), HEX);
    
    _pwmU1.begin();
    Serial.print("PWMU1 Initialized: ");
    Serial.println(_pwmU1.getAddress(), HEX);
    
    _pwmU2.begin();
    Serial.print("PWMU2 Initialized: ");
    Serial.println(_pwmU2.getAddress(), HEX);
    
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
    updateMPU9150();
    stabilize(posZ);
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
    _mpu9150.getMotion6(&_accX, &_accY, &_accZ, &_gyroX, &_gyroY, &_gyroZ);
}

double Robot::getDispX(double dt) {
    _accAngleX   = atan2(_accY, sqrt(pow(_accX, 2) + pow(_accZ, 2)));
    _gyroAngleX += (((_gyroX - _gyroOffsetX)/65.54) * dt) * (PI/180);
    
    return sin((DISP_XY_RATIO * _gyroAngleX) + ((1-DISP_XY_RATIO) * _accAngleY));
}

double Robot::getDispY(double dt) {
    _accAngleY   = atan2(_accX, sqrt(pow(_accY, 2) + pow(_accZ, 2)));
    _gyroAngleY += (((_gyroY - _gyroOffsetY)/65.54) * dt) * (PI/180);
    
    return sin((DISP_XY_RATIO * _gyroAngleY) + ((1-DISP_XY_RATIO) * _accAngleY));
}

double Robot::getDispZ(double dt) {
    _gyroAngleZ += (((_gyroZ - _gyroOffsetZ)/65.54) * dt) * (PI/180);
    
    return sin(_gyroAngleZ);
}

void Robot::stabilize(int16_t posZ) {
    double dt = (micros() - time)/1000000;
    time = micros();
    
    double outputX = _pidOutputX.compute(getDispX(dt));
    double outputY = _pidOutputY.compute(getDispY(dt));
    
    double z1 = (-outputX) + (-outputY);
    double z2 = ( outputX) + (-outputY);
    double z3 = ( outputX) + ( outputY);
    double z4 = (-outputX) + ( outputY);
    
    z1 = (_combinerConstant * z1) + ((1 - _combinerConstant) * posZ);
    z2 = (_combinerConstant * z2) + ((1 - _combinerConstant) * posZ);
    z3 = (_combinerConstant * z3) + ((1 - _combinerConstant) * posZ);
    z4 = (_combinerConstant * z4) + ((1 - _combinerConstant) * posZ);
    
    
    
    
}

void Robot::setMotorU1(MotorU1 motor, uint16_t value) {
    _pwmU1.setPWM(motor, motor*256, ((motor*256) + value) % 4096);
}

void Robot::setMotorU2(MotorU2 motor, uint16_t value) {
    _pwmU2.setPWM(motor, motor*256, ((motor*256) + value) % 4096);
}

















