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

class Robot {
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
          
          double combinerConstant);
    
    double getDispX(double dt);
    double getDispY(double dt);
    double getDispZ(double dt);
    
    void begin();
    void setMotion(int8_t velX,
                   int8_t velY,
                   int8_t velZ,
                   int8_t rotX,
                   int8_t rotY,
                   int8_t rotZ,
                   int16_t posZ,
                   int8_t torpedoCtl,
                   int8_t servoCtl[6]);
    void stop();
private:
    
    // Constants : These fine tune the control loop
    double DISP_XY_RATIO       = 0.8;
    double STABILIZER_KP       = 10.0/16.0;
    double STABILIZER_KI       =  5.0/16.0;
    double STABILIZER_KD       =  1.0/16.0;
    double DEPTH_KP            = 10.0/16.0;
    double DEPTH_KI            =  5.0/16.0;
    double DEPTH_KD            =  1.0/16.0;
    double VERT_COMBINE_RATIO  = 0.5;
    double MOTOR_Z_OFFSET      = 2048;
    double MOTOR_Z_SCALE       = 4096;
    
    // Constants : Based on calibration
    double _gyroOffsetX = 0;
    double _gyroOffsetY = 0;
    double _gyroOffsetZ = 0;
    
    unsigned long time = micros();
    
    int16_t _accX,  _accY,  _accZ;
    int16_t _gyroX, _gyroY, _gyroZ;
    
    double _accAngleX  = 0.0;
    double _accAngleY  = 0.0;
    double _accAngleZ  = 0.0;
    
    double _gyroAngleX = 0;
    double _gyroAngleY = 0;
    double _gyroAngleZ = 0;
    
    double _combinerConstant;
    
    double updateMPU9150();
    
    //double getDispX(double dt);
    //double getDispY(double dt);
    //double getDispZ(double dt);
    
    void stabilize(int16_t posZ);
    
    void setMotorU1(MotorU1 motor, uint16_t value);
    void setMotorU2(MotorU2 motor, uint16_t value);
    
    MPU6050 _mpu9150;
    Adafruit_PWMServoDriver _pwmU1;
    Adafruit_PWMServoDriver _pwmU2;
    
    PID _pidOutputX;
    PID _pidOutputY;
    PID _pidDepth;
};


#endif /* defined(____Robot__) */