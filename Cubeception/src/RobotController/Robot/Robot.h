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
#include <Wire.h>
#include <SPI.h>

#include <math.h>

#include "MPU6050/MPU6050.h"
#include "PCA9685/Adafruit_PWMServoDriver.h"
#include "MS5541C/MS5541C.h"
#include "PID/PID.h"

/**
 *  Operating modes
 */
#define MODE_RESET                  0x8000
#define MODE_KILL                   0x4000
#define MODE_LINEAR_DISABLE         0x0080
#define MODE_ROTATION_DISABLE       0x0040
#define MODE_DEPTH_DISABLE          0x0020
#define MODE_STABILIZER_DISABLE     0x0010
#define MODE_CALIBRATION_ENABLE     0x0008
#define MODE_NORMAL_ENABLE          0x0004
#define MODE_LOG_LEVEL              0x0003
#define MODE_COEFF_PRESET			0x0F00
#define MODE_SKETCH					0x3000

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

enum Sketch {
	NORMAL 		= 0,
	MOTOR 		= 1,
	IMU 		= 2,
	PRESSURE 	= 3
};

struct CoeffSet {
	double outputXKP;
	double outputXKI;
	double outputXKD;
	double outputXKF;
	
	double outputYKP;
	double outputYKI;
	double outputYKD;
	double outputYKF;
	
	double depthKP;
	double depthKI;
	double depthKD;
	double depthKF;
	
	double angleKP;
	double angleKI;
	double angleKD;
	double angleKF;
	
	double dispXYRatio;
	double verticalCombinerRatio;
	double horizontalCombinerRatio;
	
	double outputScaleXY;
	double outputScaleZ;
	double outputOffsetZ;
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
          
          double pidAngleKP,
          double pidAngleKI,
          double pidAngleKD,
          double pidAngleKF,
          
          double dispXYRatio,
          double verticalCombinerRatio,
          double horizontalCombinerRatio,
          
          double outputScaleXY,
          double outputScaleZ,
          double outputOffsetZ,
		  
		  uint16_t mode);

    void begin();
    void setMotion(int8_t velX,
                   int8_t velY,
                   int8_t velZ,
                   int8_t rotZ,
                   uint8_t torpedoCtl,
                   uint8_t servoCtl,
                   uint8_t ledCtl,
                   uint16_t mode);
    void continueMotion();
    void stop();
    void calibrate();
    
    int16_t getMagX();
    int16_t getMagY();
    int16_t getMagZ();
    
    uint16_t getPosZ();
    uint16_t getBatV();
    
private:
    void setDispXYRatio(double dispXYRatio);
    void setVerticalCombinerRatio(double verticalCombinerRatio);
    void setHorizontalCombinerRatio(double horizontalCombinerRatio);
    
    void setOutputScaleXY(double outputScaleXY);
    void setOutputScaleZ(double outputScaleZ);
    void setOutputOffsetZ(double outputOffsetZ);
	
	void changeConstants();
	void setCoeffs(CoeffSet coeffs);
	void initializeCoeffSets();
    
    double getDispXYRatio();
    double getVerticalCombinerRatio();
    double getHorizontalCombinerRatio();
    
    double getOutputScaleXY();
    double getOutputScaleZ();
    double getOutputOffsetZ();
    
    void reset();
    void updateDt();
    
    void updateMPU9150();
    double getDispX();
    double getDispY();
    double getDispZ();
    
    void queueMS5541C();
    void readMS5541C();
    
    void stabilize();
    void move();
	
	void normalOperation();
	void motorTest();
	void imuTest();
	void pressureTest();
    
    void setMotorU1(MotorU1 motor, int16_t value);
    void setMotorU2(MotorU2 motor, int16_t value);

    //Command values
    int8_t      _velX;
    int8_t      _velY;
    int8_t      _velZ;
    int8_t      _rotZ;
    uint8_t     _torpedoCtl;
    uint8_t     _servoCtl;
    uint8_t     _ledCtl;
    uint16_t    _mode;
    
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
    int16_t _magX , _magY , _magZ ;
    int16_t _pressure;
	
	//Offset accelerations
	int16_t _scaledAccX;
	int16_t _scaledAccY;
	int16_t _scaledAccZ;

    //Processed data for stabilization and depth control
    double _accAngleX , _accAngleY;
    double _gyroAngleX, _gyroAngleY;
    double _combAngleX, _combAngleY;
    double _dispX     , _dispY     , _dispZ;
    double _filtX     , _filtY     , _filtZ;
    double _stabZ[4];
    double _depthZ;
    double _combZ[4];

    //Processed data for linear and rotation control
    double _gyroAngleZ;
    double _combAngleZ;
    double _rotR;
    double _filtR;
    double _scaledVelX, _scaledVelY;
    double _rotXF[4], _rotXR[4];
    double _rotYF[4], _rotYR[4];
    double _linXF[4], _linXR[4];
    double _linYF[4], _linYR[4];
    
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
    double _verticalCombinerRatio;
    double _horizontalCombinerRatio;
    
    //Scale values
    double _outputScaleXY;
    double _outputScaleZ;
    double _outputOffsetZ;
    
    //For pressure sensor
    bool _temp;
    double _queueTime;
    unsigned long _timeSinceQueuing;
	
	//Coefficient sets
	CoeffSet _coeffs[16];
    
    //Sensors and actuators
    MPU6050 _mpu9150;
    MS5541C _ms5541C;
    Adafruit_PWMServoDriver _pwmU1;
    Adafruit_PWMServoDriver _pwmU2;
    
    //PID controllers
    PID _pidOutputX;
    PID _pidOutputY;
    PID _pidDepth;
    PID _pidAngle;
};


#endif /* defined(____Robot__) */
