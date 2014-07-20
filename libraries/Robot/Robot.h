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

#define SYSTEM_SAMPLE_PERIOD   ((double) 0.1)
#define DISP_XY_RATIO          ((double) 0.8)
#define DISP_XY_SCALE          ((double) 1/50.0)
#define STABILIZER_KP          ((double) 10.0/16.0)
#define STABILIZER_KI          ((double) 5.0/16.0)
#define STABILIZER_KD          ((double) 1.0/16.0)
#define DEPTH_KP               ((double) 10.0/16.0)
#define DEPTH_KI               ((double) 5.0/16.0)
#define DEPTH_KD               ((double) 1.0/16.0)
#define VERT_COMBINE_RATIO     ((double) 0.5)
#define MOTOR_Z_OFFSET         ((double) 2048)
#define MOTOR_Z_SCALE          ((double) 4096)

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
    Robot(uint8_t mpuAddr = 0x68,
          uint8_t pwmU1Addr = 0x79,
          uint8_t pwmU2Addr = 0x71);
    
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
    
    unsigned long time = micros();
    
    int16_t accX,  accY,  accZ;
    int16_t gyroX, gyroY, gyroZ;
    
    double stateGyroX = 0;
    double stateGyroY = 0;
    double stateGyroZ = 0;
    
    double updateMPU9150();
    
    double getDispX(double dt);
    double getDispY(double dt);
    double getDispZ(double dt);
    
    void stabilize();
    
    void setMotorU1(MotorU1 motor, int16_t value);
    void setMotorU2(MotorU2 motor, int16_t value);
    
    MPU6050 _mpu9150;
    Adafruit_PWMServoDriver _pwmU1;
    Adafruit_PWMServoDriver _pwmU2;
    
    PID pidOutputX;
    PID pidOutputY;
    PID pidDepth;
};


#endif /* defined(____Robot__) */