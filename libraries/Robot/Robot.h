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

#include <math.h>

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Adafruit_PWMServoDriver.h"
#include "PID.h"

/**
 *  Motor mapping for U1 chip
 */
enum MotorU1 {
    MYF4 = 0,
    MYF3 = 1,
    MYF2 = 2,
    MYF1 = 3,
    
    MYR4 = 4,
    MYR3 = 5,
    MYR2 = 6,
    MYR1 = 7,
    
    MXF4 = 8,
    MXF3 = 9,
    MXF2 = 10,
    MXF1 = 11,
    
    MXR4 = 12,
    MXR3 = 13,
    MXR2 = 14,
    MXR1 = 15
};

/**
 *  Motor mapping for U2 chip
 */
enum MotorU2 {
    MZF4 = 8,
    MZF3 = 9,
    MZF2 = 10,
    MZF1 = 11,
    
    MZR4 = 12,
    MZR3 = 13,
    MZR2 = 14,
    MZR1 = 15
};

/**
 *  Reads sensor data, calculates motor outputs, sets actuators
 */
class Robot {
    friend class RobotController;
public:
    Robot(uint8_t mpuAddr,
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
          
          double dispXYRatio,
          double verticalCombinerRatio,
          
          double outputScaleZ,
          double outputOffsetZ);

    void begin();
    void setMotion(int8_t velX,
                   int8_t velY,
                   int8_t velZ,
                   int8_t rotX,
                   int8_t rotY,
                   int8_t rotZ,
                   int16_t posZ,
                   int8_t torpedoCtl,
                   int8_t servoCtl[6],
                   uint8_t mode);
    void continueMotion();
    void stop();
    void calibrate();
    
private:
    void updateDt();
    
    void updateMPU9150();
    double getDispX();
    double getDispY();
    double getDispZ();
    
    void stabilize();
    
    void setMotorU1(MotorU1 motor, int16_t value);
    void setMotorU2(MotorU2 motor, int16_t value);

    //Command values
    int8_t      _velX;
    int8_t      _velY;
    int8_t      _velZ;
    int8_t      _rotX;
    int8_t      _rotY;
    int8_t      _rotZ;
    int16_t     _posZ;
    int8_t      _torpedoCtl;
    int8_t      _servoCtl[6];
    uint8_t     _mode;
    
    // Constants : Based on calibration
    int16_t _accOffsetX;
    int16_t _accOffsetY;
    int16_t _accOffsetZ;
    int16_t _gyroOffsetX;
    int16_t _gyroOffsetY;
    int16_t _gyroOffsetZ;
    
    //Raw sensor data
    int16_t _accX , _accY , _accZ ;
    int16_t _gyroX, _gyroY, _gyroZ;

    //Processed data
    double _accAngleX , _accAngleY , _accAngleZ ;
    double _gyroAngleX, _gyroAngleY, _gyroAngleZ;
    double _combAngleX, _combAngleY, _combAngleZ;
    double _dispX     , _dispY     , _dispZ     ;
    double _filtX     , _filtY     , _filtZ     ;
    double _stabZ[4];
    double _combZ[4];

    //Motor commands
    int16_t _mxf[4];
    int16_t _mxr[4];
    int16_t _myf[4];
    int16_t _myr[4];
    int16_t _mzf[4];
    int16_t _mzr[4];
    
    //Timekeepers
    unsigned long _realtime;
    double        _dt;
    
    //Loop constants
    double _dispXYRatio;
    double  _verticalCombinerRatio;
    
    //Scale values
    double  _outputScaleZ;
    double  _outputOffsetZ;
    
    //Sensors and actuators
    MPU6050 _mpu9150;
    Adafruit_PWMServoDriver _pwmU1;
    Adafruit_PWMServoDriver _pwmU2;
    
    //PID controllers
    PID _pidOutputX;
    PID _pidOutputY;
    PID _pidDepth;
};


#endif /* defined(____Robot__) */
