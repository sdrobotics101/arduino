//
//  Robot.cpp
//  
//
//  Created by Rahul Salvi on 7/19/14.
//
//

#include "Robot.h"

/**
 *  Constructor
 *
 *  @param mpuAddr                 I2C address of MPU9150
 *  @param pwmU1Addr               I2C address of PCA9685 on U1
 *  @param pwmU2Addr               I2C address of PCA9685 on U2
 *  @param pidOutputXKP            Kp for output X PID controller
 *  @param pidOutputXKI            Ki for output X PID controller
 *  @param pidOutputXKD            Kd for output X PID controller
 *  @param pidOutputXKF            Kf for output X PID controller
 *  @param pidOutputYKP            Kp for output Y PID controller
 *  @param pidOutputYKI            Ki for output Y PID controller
 *  @param pidOutputYKD            Kd for output Y PID controller
 *  @param pidOutputYKF            Kf for output Y PID controller
 *  @param pidDepthKP              Kp for depth PID controller
 *  @param pidDepthKI              Ki for depth PID controller
 *  @param pidDepthKD              Kd for depth PID controller
 *  @param pidDepthKF              Kf for depth PID controller
 *  @param pidAngleKP              Kp for angle Z PID controller
 *  @param pidAngleKI              Ki for angle Z PID controller
 *  @param pidAngleKD              Kd for angle Z PID controller
 *  @param pidAngleKF              Kf for angle Z PID controller
 *  @param dispXYRatio             Complementary filter ratio
 *  @param verticalCombinerRatio   Vertical combiner ratio
 *  @param horizontalCombinerRatio Horizontal combiner ratio
 *  @param outputScaleXY           Scale in XY direction
 *  @param outputScaleZ            Scale in Z direction
 *  @param outputOffsetZ           Offset in Z direction
 *
 *  @return Nothing
 */
Robot::Robot(uint8_t mpuAddr,
             uint8_t pwmU1Addr,
             uint8_t pwmU2Addr,
             
             double  pidOutputXKP,
             double  pidOutputXKI,
             double  pidOutputXKD,
             double  pidOutputXKF,
             
             double  pidOutputYKP,
             double  pidOutputYKI,
             double  pidOutputYKD,
             double  pidOutputYKF,
             
             double  pidDepthKP,
             double  pidDepthKI,
             double  pidDepthKD,
             double  pidDepthKF,
             
             double  pidAngleKP,
             double  pidAngleKI,
             double  pidAngleKD,
             double  pidAngleKF,
             
             double  dispXYRatio,
             double  verticalCombinerRatio,
             double  horizontalCombinerRatio,
             
             double  outputScaleXY,
             double  outputScaleZ,
             double  outputOffsetZ) :

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

                _pidAngle(  pidAngleKP,
                            pidAngleKI,
                            pidAngleKD,
                            pidAngleKF),

                _dispXYRatio(dispXYRatio),
                _verticalCombinerRatio(verticalCombinerRatio),
                _horizontalCombinerRatio(horizontalCombinerRatio),

                _outputScaleXY(outputScaleXY),
                _outputScaleZ (outputScaleZ ),
                _outputOffsetZ(outputOffsetZ)
{
    reset();
    initializeCoeffSets();
}

/**
 *  Initializes I2C bus, sensors, actuators
 */
void Robot::begin() {
    Wire.begin();
    Serial.println("Wire Initialized");
    
    SPI.begin();
    Serial.println("SPI Initialized");
    
    _ms5541C.begin();
    
    _mpu9150.initialize();
    Serial.print("MPU9150 Initialized: ");
    Serial.println(_mpu9150.getAddress(), HEX);
    
    _pwmU1.begin();
    Serial.print("PWMU1 Initialized: ");
    Serial.println(_pwmU1.getAddress(), HEX);
    
    _pwmU2.begin();
    Serial.print("PWMU2 Initialized: ");
    Serial.println(_pwmU2.getAddress(), HEX);
    
    analogReadResolution(12);
   
    Serial.println("Robot Initialized");
}

void Robot::setMotion(int8_t velX,
                      int8_t velY,
                      int8_t velZ,
                      int8_t rotZ,
                      uint8_t torpedoCtl,
                      uint8_t servoCtl,
                      uint8_t ledCtl,
                      uint16_t mode)
{
    _velX = velX;
    _velY = velY;
    _velZ = velZ;
    _rotZ = rotZ;
    _torpedoCtl = torpedoCtl;
    _servoCtl = servoCtl;
    _ledCtl = ledCtl;
    _mode = mode;
    
    if (_mode & MODE_RESET) {
        reset();
    }
	
	changeConstants();
	
	uint8_t sketch = (uint8_t)((_mode & MODE_SKETCH) >> 12);
	
	switch (sketch) {
		case NORMAL:
			normalOperation();
		break;
		case MOTOR:
			motorTest();
		break;
		case IMU:
			imuTest();
		break;
		case PRESSURE:
			pressureTest();
		break;
	}
}

/**
 *  Continues previously set motion
 */
void Robot::continueMotion() {
    if (_mode & MODE_RESET) {
        reset();
    }
	
	changeConstants();
	
	uint8_t sketch = (_mode & MODE_SKETCH) >> 12;
	
	switch (sketch) {
		case NORMAL:
			normalOperation();
		break;
		case MOTOR:
			motorTest();
		break;
		case IMU:
			imuTest();
		break;
		case PRESSURE:
			pressureTest();
		break;
	}
}

/**
 *  Brings all motors to a halt
 */
void Robot::stop() {
    for (int i = 0; i < 15; i++) {
        _pwmU1.setPWM(i, 0, 0);
    }
    for (int i = 8; i < 15; i++) {
        _pwmU2.setPWM(i, 0, 0);
    }
}

/**
 *  Calculates gyro offsets
 */
void Robot::calibrate() {
   int32_t sumAccX = 0;
   int32_t sumAccY = 0;
   int32_t sumAccZ = 0;

   int32_t sumGyroX = 0;
   int32_t sumGyroY = 0;
   int32_t sumGyroZ = 0;

   for (int i = 0; i < 128; i++) {

      updateMPU9150();
      delay(10);

      sumAccX  += ((int32_t) (_accX));
      sumAccY  += ((int32_t) (_accY));
      sumAccZ  += ((int32_t) (_accZ));
      sumGyroX += ((int32_t) (_gyroX));
      sumGyroY += ((int32_t) (_gyroY));
      sumGyroZ += ((int32_t) (_gyroZ));
   }

   _accOffsetX  = (int16_t) ((sumAccX +64)>>7);
   _accOffsetY  = (int16_t) ((sumAccY +64)>>7);
   _accOffsetZ  = (int16_t) ((sumAccZ +64)>>7);

   _gyroOffsetX = (int16_t) ((sumGyroX+64)>>7);
   _gyroOffsetY = (int16_t) ((sumGyroY+64)>>7);
   _gyroOffsetZ = (int16_t) ((sumGyroZ+64)>>7);
    
   Serial.print("C: ");
   Serial.print(_accOffsetX); Serial.print(" ");
   Serial.print(_accOffsetY); Serial.print(" ");
   Serial.print(_accOffsetZ); Serial.print(" ");
   Serial.print(_gyroOffsetX); Serial.print(" ");
   Serial.print(_gyroOffsetY); Serial.print(" ");
   Serial.print(_gyroOffsetZ); Serial.print(" ");
   Serial.println("");
}

int16_t Robot::getMagX() {
    return _magX;
}

int16_t Robot::getMagY() {
    return _magY;
}

int16_t Robot::getMagZ() {
    return _magZ;
}

uint16_t Robot::getPosZ() {
    return (uint16_t)_depthZ;
}

uint16_t getBatV() {
    return analogRead(A0);
}

void Robot::setDispXYRatio(double dispXYRatio) {
    _dispXYRatio = dispXYRatio;
}
void Robot::setVerticalCombinerRatio(double verticalCombinerRatio) {
    _verticalCombinerRatio = verticalCombinerRatio;
}
void Robot::setHorizontalCombinerRatio(double horizontalCombinerRatio) {
    _horizontalCombinerRatio = horizontalCombinerRatio;
}

void Robot::setOutputScaleXY(double outputScaleXY) {
    _outputScaleXY = outputScaleXY;
}

void Robot::setOutputScaleZ(double outputScaleZ) {
    _outputScaleZ = outputScaleZ;
}

void Robot::setOutputOffsetZ(double outputOffsetZ) {
    _outputOffsetZ = outputOffsetZ;
}

void Robot::changeConstants() {
        uint8_t set = (uint8_t)((_mode & MODE_COEFF_PRESET) >> 8);
        setCoeffs(_coeffs[set]);
}

void Robot::setCoeffs(CoeffSet coeffs) {
        _pidOutputX.setKP(coeffs.outputXKP);
        _pidOutputX.setKI(coeffs.outputXKI);
        _pidOutputX.setKD(coeffs.outputXKD);
        _pidOutputX.setKF(coeffs.outputXKF);
        
        _pidOutputY.setKP(coeffs.outputYKP);
        _pidOutputY.setKI(coeffs.outputYKI);
        _pidOutputY.setKD(coeffs.outputYKD);
        _pidOutputY.setKF(coeffs.outputYKF);
        
        _pidDepth.setKP(coeffs.depthKP);
        _pidDepth.setKI(coeffs.depthKI);
        _pidDepth.setKD(coeffs.depthKD);
        _pidDepth.setKF(coeffs.depthKF);
        
        _pidAngle.setKP(coeffs.angleKP);
        _pidAngle.setKI(coeffs.angleKI);
        _pidAngle.setKD(coeffs.angleKD);
        _pidAngle.setKF(coeffs.angleKF);
        
        setDispXYRatio(coeffs.dispXYRatio);
        setVerticalCombinerRatio(coeffs.verticalCombinerRatio);
        setHorizontalCombinerRatio(coeffs.horizontalCombinerRatio);
        
        setOutputScaleXY(coeffs.outputScaleXY);
        setOutputScaleZ(coeffs.outputScaleZ);
        setOutputOffsetZ(coeffs.outputOffsetZ);
}

void Robot::initializeCoeffSets() {
        //set 0 : stabilization       pid    : [10,  5,  1]/16
        //      : depth               pid    : [16        ]/16
        //      : rotation            pid    : [16,  0,  0]/16
        //      : stabilization:depth ratio  : [0.4, 0.6]
        //      : linear:rotation     ratio  : [0.8, 0.2]
        //{
        _coeffs[ 0].outputXKP = 10.0/16; _coeffs[ 0].outputXKI =  5.0/16; _coeffs[ 0].outputXKD =  1.0/16; _coeffs[ 0].outputXKF =  0.0/16;
        _coeffs[ 0].outputYKP = 10.0/16; _coeffs[ 0].outputYKI =  5.0/16; _coeffs[ 0].outputYKD =  1.0/16; _coeffs[ 0].outputYKF =  0.0/16;
        _coeffs[ 0].depthKP   = 16.0/16; _coeffs[ 0].depthKI   =  0.0/16; _coeffs[ 0].depthKD   =  0.0/16; _coeffs[ 0].depthKF   =  0.0/16;
        _coeffs[ 0].angleKP   = 16.0/16; _coeffs[ 0].angleKI   =  0.0/16; _coeffs[ 0].angleKD   =  0.0/16; _coeffs[ 0].angleKF   =  0.0/16;
        
        _coeffs[ 0].dispXYRatio = 0.80; _coeffs[ 0].verticalCombinerRatio = 0.40; _coeffs[ 0].horizontalCombinerRatio = 0.80;
        _coeffs[ 0].outputScaleXY = 4096.0; _coeffs[ 0].outputScaleZ = 4096.0; _coeffs[ 0].outputOffsetZ = 0;
        //}
        
        //set 1 : stabilization       pid    : [10,  5,  1]/16
        //      : depth               pid    : [16        ]/16
        //      : rotation            pid    : [10,  4,  2]/16
        //      : stabilization:depth ratio  : [0.4, 0.6]
        //      : linear:rotation     ratio  : [0.8, 0.2]
        //{
        _coeffs[ 1].outputXKP = 10.0/16; _coeffs[ 1].outputXKI =  5.0/16; _coeffs[ 1].outputXKD =  1.0/16; _coeffs[ 1].outputXKF =  0.0/16;
        _coeffs[ 1].outputYKP = 10.0/16; _coeffs[ 1].outputYKI =  5.0/16; _coeffs[ 1].outputYKD =  1.0/16; _coeffs[ 1].outputYKF =  0.0/16;
        _coeffs[ 1].depthKP   = 16.0/16; _coeffs[ 1].depthKI   =  0.0/16; _coeffs[ 1].depthKD   =  0.0/16; _coeffs[ 1].depthKF   =  0.0/16;
        _coeffs[ 1].angleKP   = 10.0/16; _coeffs[ 1].angleKI   =  4.0/16; _coeffs[ 1].angleKD   =  2.0/16; _coeffs[ 1].angleKF   =  0.0/16;
        
        _coeffs[ 1].dispXYRatio = 0.80; _coeffs[ 1].verticalCombinerRatio = 0.40; _coeffs[ 1].horizontalCombinerRatio = 0.80;
        _coeffs[ 1].outputScaleXY = 4096.0; _coeffs[ 1].outputScaleZ = 4096.0; _coeffs[ 1].outputOffsetZ = 0;
        //}
        
        //set 2 : stabilization       pid    : [10,  5,  1]/16
        //      : depth               pid    : [16        ]/16
        //      : rotation            pid    : [10,  5,  1]/16
        //      : stabilization:depth ratio  : [0.4, 0.6]
        //      : linear:rotation     ratio  : [0.8, 0.2]
        //{
        _coeffs[ 2].outputXKP = 10.0/16; _coeffs[ 2].outputXKI =  5.0/16; _coeffs[ 2].outputXKD =  1.0/16; _coeffs[ 2].outputXKF =  0.0/16;
        _coeffs[ 2].outputYKP = 10.0/16; _coeffs[ 2].outputYKI =  5.0/16; _coeffs[ 2].outputYKD =  1.0/16; _coeffs[ 2].outputYKF =  0.0/16;
        _coeffs[ 2].depthKP   = 16.0/16; _coeffs[ 2].depthKI   =  0.0/16; _coeffs[ 2].depthKD   =  0.0/16; _coeffs[ 2].depthKF   =  0.0/16;
        _coeffs[ 2].angleKP   = 10.0/16; _coeffs[ 2].angleKI   =  5.0/16; _coeffs[ 2].angleKD   =  1.0/16; _coeffs[ 2].angleKF   =  0.0/16;
        
        _coeffs[ 2].dispXYRatio = 0.80; _coeffs[ 2].verticalCombinerRatio = 0.40; _coeffs[ 2].horizontalCombinerRatio = 0.80;
        _coeffs[ 2].outputScaleXY = 4096.0; _coeffs[ 2].outputScaleZ = 4096.0; _coeffs[ 2].outputOffsetZ = 0;
        //}
        
        //set 3 : stabilization       pid    : [10,  5,  1]/16
        //      : depth               pid    : [16        ]/16
        //      : rotation            pid    : [12,  3,  1]/16
        //      : stabilization:depth ratio  : [0.4, 0.6]
        //      : linear:rotation     ratio  : [0.8, 0.2]
        //{
        _coeffs[ 3].outputXKP = 10.0/16; _coeffs[ 3].outputXKI =  5.0/16; _coeffs[ 3].outputXKD =  1.0/16; _coeffs[ 3].outputXKF =  0.0/16;
        _coeffs[ 3].outputYKP = 10.0/16; _coeffs[ 3].outputYKI =  5.0/16; _coeffs[ 3].outputYKD =  1.0/16; _coeffs[ 3].outputYKF =  0.0/16;
        _coeffs[ 3].depthKP   = 16.0/16; _coeffs[ 3].depthKI   =  0.0/16; _coeffs[ 3].depthKD   =  0.0/16; _coeffs[ 3].depthKF   =  0.0/16;
        _coeffs[ 3].angleKP   = 12.0/16; _coeffs[ 3].angleKI   =  3.0/16; _coeffs[ 3].angleKD   =  1.0/16; _coeffs[ 3].angleKF   =  0.0/16;
        
        _coeffs[ 3].dispXYRatio = 0.80; _coeffs[ 3].verticalCombinerRatio = 0.40; _coeffs[ 3].horizontalCombinerRatio = 0.80;
        _coeffs[ 3].outputScaleXY = 4096.0; _coeffs[ 3].outputScaleZ = 4096.0; _coeffs[ 3].outputOffsetZ = 0;
        //}
        
        //set 4
        //{
        _coeffs[ 4].outputXKP = 10.0/16; _coeffs[ 4].outputXKI =  5.0/16; _coeffs[ 4].outputXKD =  1.0/16; _coeffs[ 4].outputXKF =  0.0/16;
        _coeffs[ 4].outputYKP = 10.0/16; _coeffs[ 4].outputYKI =  5.0/16; _coeffs[ 4].outputYKD =  1.0/16; _coeffs[ 4].outputYKF =  0.0/16;
        _coeffs[ 4].depthKP   = 16.0/16; _coeffs[ 4].depthKI   =  0.0/16; _coeffs[ 4].depthKD   =  0.0/16; _coeffs[ 4].depthKF   =  0.0/16;
        _coeffs[ 4].angleKP   = 16.0/16; _coeffs[ 4].angleKI   =  0.0/16; _coeffs[ 4].angleKD   =  0.0/16; _coeffs[ 4].angleKF   =  0.0/16;
        
        _coeffs[ 4].dispXYRatio = 0.80; _coeffs[ 4].verticalCombinerRatio = 0.40; _coeffs[ 4].horizontalCombinerRatio = 0.80;
        _coeffs[ 4].outputScaleXY = 4096.0; _coeffs[ 4].outputScaleZ = 4096.0; _coeffs[ 4].outputOffsetZ = 0;
        //}
        
        //set 5
        //{
        _coeffs[ 5].outputXKP = 10.0/16; _coeffs[ 5].outputXKI =  5.0/16; _coeffs[ 5].outputXKD =  1.0/16; _coeffs[ 5].outputXKF =  0.0/16;
        _coeffs[ 5].outputYKP = 10.0/16; _coeffs[ 5].outputYKI =  5.0/16; _coeffs[ 5].outputYKD =  1.0/16; _coeffs[ 5].outputYKF =  0.0/16;
        _coeffs[ 5].depthKP   = 16.0/16; _coeffs[ 5].depthKI   =  0.0/16; _coeffs[ 5].depthKD   =  0.0/16; _coeffs[ 5].depthKF   =  0.0/16;
        _coeffs[ 5].angleKP   = 16.0/16; _coeffs[ 5].angleKI   =  0.0/16; _coeffs[ 5].angleKD   =  0.0/16; _coeffs[ 5].angleKF   =  0.0/16;
        
        _coeffs[ 5].dispXYRatio = 0.80; _coeffs[ 5].verticalCombinerRatio = 0.40; _coeffs[ 5].horizontalCombinerRatio = 0.80;
        _coeffs[ 5].outputScaleXY = 4096.0; _coeffs[ 5].outputScaleZ = 4096.0; _coeffs[ 5].outputOffsetZ = 0;
        //}     
        
        //set 6
        //{
        _coeffs[ 6].outputXKP = 10.0/16; _coeffs[ 6].outputXKI =  5.0/16; _coeffs[ 6].outputXKD =  1.0/16; _coeffs[ 6].outputXKF =  0.0/16;
        _coeffs[ 6].outputYKP = 10.0/16; _coeffs[ 6].outputYKI =  5.0/16; _coeffs[ 6].outputYKD =  1.0/16; _coeffs[ 6].outputYKF =  0.0/16;
        _coeffs[ 6].depthKP   = 16.0/16; _coeffs[ 6].depthKI   =  0.0/16; _coeffs[ 6].depthKD   =  0.0/16; _coeffs[ 6].depthKF   =  0.0/16;
        _coeffs[ 6].angleKP   = 16.0/16; _coeffs[ 6].angleKI   =  0.0/16; _coeffs[ 6].angleKD   =  0.0/16; _coeffs[ 6].angleKF   =  0.0/16;
        
        _coeffs[ 6].dispXYRatio = 0.80; _coeffs[ 6].verticalCombinerRatio = 0.40; _coeffs[ 6].horizontalCombinerRatio = 0.80;
        _coeffs[ 6].outputScaleXY = 4096.0; _coeffs[ 6].outputScaleZ = 4096.0; _coeffs[ 6].outputOffsetZ = 0;
        //}
        
        //set 7
        //{
        _coeffs[ 7].outputXKP = 10.0/16; _coeffs[ 7].outputXKI =  5.0/16; _coeffs[ 7].outputXKD =  1.0/16; _coeffs[ 7].outputXKF =  0.0/16;
        _coeffs[ 7].outputYKP = 10.0/16; _coeffs[ 7].outputYKI =  5.0/16; _coeffs[ 7].outputYKD =  1.0/16; _coeffs[ 7].outputYKF =  0.0/16;
        _coeffs[ 7].depthKP   = 16.0/16; _coeffs[ 7].depthKI   =  0.0/16; _coeffs[ 7].depthKD   =  0.0/16; _coeffs[ 7].depthKF   =  0.0/16;
        _coeffs[ 7].angleKP   = 16.0/16; _coeffs[ 7].angleKI   =  0.0/16; _coeffs[ 7].angleKD   =  0.0/16; _coeffs[ 7].angleKF   =  0.0/16;
        
        _coeffs[ 7].dispXYRatio = 0.80; _coeffs[ 7].verticalCombinerRatio = 0.40; _coeffs[ 7].horizontalCombinerRatio = 0.80;
        _coeffs[ 7].outputScaleXY = 4096.0; _coeffs[ 7].outputScaleZ = 4096.0; _coeffs[ 7].outputOffsetZ = 0;
        //}
        
        //set 8
        //{
        _coeffs[ 8].outputXKP = 10.0/16; _coeffs[ 8].outputXKI =  5.0/16; _coeffs[ 8].outputXKD =  1.0/16; _coeffs[ 8].outputXKF =  0.0/16;
        _coeffs[ 8].outputYKP = 10.0/16; _coeffs[ 8].outputYKI =  5.0/16; _coeffs[ 8].outputYKD =  1.0/16; _coeffs[ 8].outputYKF =  0.0/16;
        _coeffs[ 8].depthKP   = 16.0/16; _coeffs[ 8].depthKI   =  0.0/16; _coeffs[ 8].depthKD   =  0.0/16; _coeffs[ 8].depthKF   =  0.0/16;
        _coeffs[ 8].angleKP   = 16.0/16; _coeffs[ 8].angleKI   =  0.0/16; _coeffs[ 8].angleKD   =  0.0/16; _coeffs[ 8].angleKF   =  0.0/16;
        
        _coeffs[ 8].dispXYRatio = 0.80; _coeffs[ 8].verticalCombinerRatio = 0.40; _coeffs[ 8].horizontalCombinerRatio = 0.80;
        _coeffs[ 8].outputScaleXY = 4096.0; _coeffs[ 8].outputScaleZ = 4096.0; _coeffs[ 8].outputOffsetZ = 0;
        //}
        
        //set 9
        //{
        _coeffs[ 9].outputXKP = 10.0/16; _coeffs[ 9].outputXKI =  5.0/16; _coeffs[ 9].outputXKD =  1.0/16; _coeffs[ 9].outputXKF =  0.0/16;
        _coeffs[ 9].outputYKP = 10.0/16; _coeffs[ 9].outputYKI =  5.0/16; _coeffs[ 9].outputYKD =  1.0/16; _coeffs[ 9].outputYKF =  0.0/16;
        _coeffs[ 9].depthKP   = 16.0/16; _coeffs[ 9].depthKI   =  0.0/16; _coeffs[ 9].depthKD   =  0.0/16; _coeffs[ 9].depthKF   =  0.0/16;
        _coeffs[ 9].angleKP   = 16.0/16; _coeffs[ 9].angleKI   =  0.0/16; _coeffs[ 9].angleKD   =  0.0/16; _coeffs[ 9].angleKF   =  0.0/16;
        
        _coeffs[ 9].dispXYRatio = 0.80; _coeffs[ 9].verticalCombinerRatio = 0.40; _coeffs[ 9].horizontalCombinerRatio = 0.80;
        _coeffs[ 9].outputScaleXY = 4096.0; _coeffs[ 9].outputScaleZ = 4096.0; _coeffs[ 9].outputOffsetZ = 0;
        //}

        //set 10
        //{
        _coeffs[10].outputXKP = 10.0/16; _coeffs[10].outputXKI =  5.0/16; _coeffs[10].outputXKD =  1.0/16; _coeffs[10].outputXKF =  0.0/16;
        _coeffs[10].outputYKP = 10.0/16; _coeffs[10].outputYKI =  5.0/16; _coeffs[10].outputYKD =  1.0/16; _coeffs[10].outputYKF =  0.0/16;
        _coeffs[10].depthKP   = 16.0/16; _coeffs[10].depthKI   =  0.0/16; _coeffs[10].depthKD   =  0.0/16; _coeffs[10].depthKF   =  0.0/16;
        _coeffs[10].angleKP   = 16.0/16; _coeffs[10].angleKI   =  0.0/16; _coeffs[10].angleKD   =  0.0/16; _coeffs[10].angleKF   =  0.0/16;
        
        _coeffs[10].dispXYRatio = 0.80; _coeffs[10].verticalCombinerRatio = 0.40; _coeffs[10].horizontalCombinerRatio = 0.80;
        _coeffs[10].outputScaleXY = 4096.0; _coeffs[10].outputScaleZ = 4096.0; _coeffs[10].outputOffsetZ = 0;
        //}
        
        //set 11
        //{
        _coeffs[11].outputXKP = 10.0/16; _coeffs[11].outputXKI =  5.0/16; _coeffs[11].outputXKD =  1.0/16; _coeffs[11].outputXKF =  0.0/16;
        _coeffs[11].outputYKP = 10.0/16; _coeffs[11].outputYKI =  5.0/16; _coeffs[11].outputYKD =  1.0/16; _coeffs[11].outputYKF =  0.0/16;
        _coeffs[11].depthKP   = 16.0/16; _coeffs[11].depthKI   =  0.0/16; _coeffs[11].depthKD   =  0.0/16; _coeffs[11].depthKF   =  0.0/16;
        _coeffs[11].angleKP   = 16.0/16; _coeffs[11].angleKI   =  0.0/16; _coeffs[11].angleKD   =  0.0/16; _coeffs[11].angleKF   =  0.0/16;
        
        _coeffs[11].dispXYRatio = 0.80; _coeffs[11].verticalCombinerRatio = 0.40; _coeffs[11].horizontalCombinerRatio = 0.80;
        _coeffs[11].outputScaleXY = 4096.0; _coeffs[11].outputScaleZ = 4096.0; _coeffs[11].outputOffsetZ = 0;
        //}
        
        //set 12
        //{
        _coeffs[12].outputXKP = 10.0/16; _coeffs[12].outputXKI =  5.0/16; _coeffs[12].outputXKD =  1.0/16; _coeffs[12].outputXKF =  0.0/16;
        _coeffs[12].outputYKP = 10.0/16; _coeffs[12].outputYKI =  5.0/16; _coeffs[12].outputYKD =  1.0/16; _coeffs[12].outputYKF =  0.0/16;
        _coeffs[12].depthKP   = 16.0/16; _coeffs[12].depthKI   =  0.0/16; _coeffs[12].depthKD   =  0.0/16; _coeffs[12].depthKF   =  0.0/16;
        _coeffs[12].angleKP   = 16.0/16; _coeffs[12].angleKI   =  0.0/16; _coeffs[12].angleKD   =  0.0/16; _coeffs[12].angleKF   =  0.0/16;
        
        _coeffs[12].dispXYRatio = 0.80; _coeffs[12].verticalCombinerRatio = 0.40; _coeffs[12].horizontalCombinerRatio = 0.80;
        _coeffs[12].outputScaleXY = 4096.0; _coeffs[12].outputScaleZ = 4096.0; _coeffs[12].outputOffsetZ = 0;
        //}
        
        //set 13
        //{
        _coeffs[13].outputXKP = 10.0/16; _coeffs[13].outputXKI =  5.0/16; _coeffs[13].outputXKD =  1.0/16; _coeffs[13].outputXKF =  0.0/16;
        _coeffs[13].outputYKP = 10.0/16; _coeffs[13].outputYKI =  5.0/16; _coeffs[13].outputYKD =  1.0/16; _coeffs[13].outputYKF =  0.0/16;
        _coeffs[13].depthKP   = 16.0/16; _coeffs[13].depthKI   =  0.0/16; _coeffs[13].depthKD   =  0.0/16; _coeffs[13].depthKF   =  0.0/16;
        _coeffs[13].angleKP   = 16.0/16; _coeffs[13].angleKI   =  0.0/16; _coeffs[13].angleKD   =  0.0/16; _coeffs[13].angleKF   =  0.0/16;
        
        _coeffs[13].dispXYRatio = 0.80; _coeffs[13].verticalCombinerRatio = 0.40; _coeffs[13].horizontalCombinerRatio = 0.80;
        _coeffs[13].outputScaleXY = 4096.0; _coeffs[13].outputScaleZ = 4096.0; _coeffs[13].outputOffsetZ = 0;
        //}
        
        //set 14
        //{
        _coeffs[14].outputXKP = 10.0/16; _coeffs[14].outputXKI =  5.0/16; _coeffs[14].outputXKD =  1.0/16; _coeffs[14].outputXKF =  0.0/16;
        _coeffs[14].outputYKP = 10.0/16; _coeffs[14].outputYKI =  5.0/16; _coeffs[14].outputYKD =  1.0/16; _coeffs[14].outputYKF =  0.0/16;
        _coeffs[14].depthKP   = 16.0/16; _coeffs[14].depthKI   =  0.0/16; _coeffs[14].depthKD   =  0.0/16; _coeffs[14].depthKF   =  0.0/16;
        _coeffs[14].angleKP   = 16.0/16; _coeffs[14].angleKI   =  0.0/16; _coeffs[14].angleKD   =  0.0/16; _coeffs[14].angleKF   =  0.0/16;
        
        _coeffs[14].dispXYRatio = 0.80; _coeffs[14].verticalCombinerRatio = 0.40; _coeffs[14].horizontalCombinerRatio = 0.80;
        _coeffs[14].outputScaleXY = 4096.0; _coeffs[14].outputScaleZ = 4096.0; _coeffs[14].outputOffsetZ = 0;
        //}
        
        //set 15
        //{
        _coeffs[15].outputXKP = 10.0/16; _coeffs[15].outputXKI =  5.0/16; _coeffs[15].outputXKD =  1.0/16; _coeffs[15].outputXKF =  0.0/16;
        _coeffs[15].outputYKP = 10.0/16; _coeffs[15].outputYKI =  5.0/16; _coeffs[15].outputYKD =  1.0/16; _coeffs[15].outputYKF =  0.0/16;
        _coeffs[15].depthKP   = 16.0/16; _coeffs[15].depthKI   =  0.0/16; _coeffs[15].depthKD   =  0.0/16; _coeffs[15].depthKF   =  0.0/16;
        _coeffs[15].angleKP   = 16.0/16; _coeffs[15].angleKI   =  0.0/16; _coeffs[15].angleKD   =  0.0/16; _coeffs[15].angleKF   =  0.0/16;
        
        _coeffs[15].dispXYRatio = 0.80; _coeffs[15].verticalCombinerRatio = 0.40; _coeffs[15].horizontalCombinerRatio = 0.80;
        _coeffs[15].outputScaleXY = 4096.0; _coeffs[15].outputScaleZ = 4096.0; _coeffs[15].outputOffsetZ = 0;
        //}
}

double Robot::getDispXYRatio() {
    return _dispXYRatio;
}

double Robot::getVerticalCombinerRatio() {
    return _verticalCombinerRatio;
}

double Robot::getHorizontalCombinerRatio() {
    return _horizontalCombinerRatio;
}

double Robot::getOutputScaleXY() {
    return _outputScaleXY;
}

double Robot::getOutputScaleZ() {
    return _outputScaleZ;
}

double Robot::getOutputOffsetZ() {
    return _outputOffsetZ;
}

/**
 *  Resets all the variables
 */
void Robot::reset() {
     _velX              = 0;
     _velY              = 0;
     _velZ              = 0;
     _rotZ              = 0;
     _torpedoCtl    = 0;
     _servoCtl          = 0;
         _ledCtl                = 0;
     _mode          = 0xF1;
    
    _accOffsetX  = 625;   _accOffsetY  = -350;   _accOffsetZ  = 17240;
    _gyroOffsetX = -31;   _gyroOffsetY = -68;   _gyroOffsetZ = -260;
    
    _accX  = 0; _accY  = 0; _accZ  = 0;
    _gyroX = 0; _gyroY = 0; _gyroZ = 0;
    _pressure = 0;
    
    _accAngleX   = 0.0; _accAngleY   = 0.0;
    _gyroAngleX  = 0.0; _gyroAngleY  = 0.0;
    _combAngleX  = 0.0; _combAngleY  = 0.0;
    _dispX       = 0.0; _dispY       = 0.0; _dispZ  = 0.0;
    _filtX       = 0.0; _filtY       = 0.0; _filtZ  = 0.0;
                                            _depthZ = 0.0;

    _gyroAngleZ = 0.0;
    _combAngleZ = 0.0;
    _rotR       = 0.0;
    _filtR      = 0.0;
    _scaledVelX = 0.0; _scaledVelY = 0.0;

    for (int i=0; i<4; i++) {
       _stabZ[i] = 0.0;
       _combZ[i] = 0.0;
        
       _rotXF[i] = 0.0;
       _rotXR[i] = 0.0;
       _rotYF[i] = 0.0;
       _rotYR[i] = 0.0;
       _linXF[i] = 0.0;
       _linXR[i] = 0.0;
       _linYF[i] = 0.0;
       _linYR[i] = 0.0;
        
       _mxf[i]   = 0;
       _mxr[i]   = 0;
       _myf[i]   = 0;
       _myr[i]   = 0;
       _mzf[i]   = 0;
       _mzr[i]   = 0;
    }

    _realtime   = micros();
    _dt         = 0.0;
    
    _temp = true;
    _queueTime = 0;
    _timeSinceQueuing = 0;

        _pidOutputX.reset();
        _pidOutputY.reset();
        _pidDepth.reset();
        _pidAngle.reset();
}

/**
 *  Updates time
 */
void Robot::updateDt() {
   unsigned long currentTime = micros();        

   _dt       = ((double) (currentTime - _realtime))/1000000.0;
   _realtime = currentTime;
}

/**
 *  Polls data from MPU9150
 */
void Robot::updateMPU9150() {
    _mpu9150.getMotion9(&_accX, &_accY, &_accZ, &_gyroX, &_gyroY, &_gyroZ, &_magX, &_magY, &_magZ);
}

/**
 *  Calculates the angle along the X rotation axis
 *
 *  @return The sine of the X angle
 */
double Robot::getDispX() {
    double scaledGyroX = (double) (_gyroX - _gyroOffsetX) / 65.54;

    _accAngleX   = atan2((double)_accY, sqrt(pow((double)_accX, 2) + pow((double)_accZ, 2)));
    _gyroAngleX  = scaledGyroX * _dt * (PI/180);
    _combAngleX  = (_dispXYRatio  * (_gyroAngleX + _combAngleX)) + ((1 - _dispXYRatio) * _accAngleX);

    if (_mode & MODE_STABILIZER_DISABLE) {
        _combAngleX = 0.0;
    }

    return sin(_combAngleX);
}

/**
 *  Calculates the angle along the Y rotation axis
 *
 *  @return The sine of the Y angle
 */
double Robot::getDispY() {
    double scaledGyroY = (double) (_gyroY - _gyroOffsetY) / 65.54;

    _accAngleY   = atan2((double)_accX, sqrt(pow((double)_accY, 2) + pow((double)_accZ, 2)));
    _gyroAngleY  = scaledGyroY * _dt * (PI/180);
    _combAngleY  = (_dispXYRatio  * (_gyroAngleY + _combAngleY)) + ((1 - _dispXYRatio) * _accAngleY);
    
    if (_mode & MODE_STABILIZER_DISABLE) {
        _combAngleY = 0.0;
    }

    return sin(_combAngleY);
}

/**
 *  Calculates the angle along the Z rotation axis
 *
 *  @return The sine of the Z angle
 */
double Robot::getDispZ() {
    double scaledGyroZ = (double) (_gyroZ - _gyroOffsetZ)/65.54;

    _gyroAngleZ += (scaledGyroZ * _dt * (PI/180));
    
    return _gyroAngleZ;
}

void Robot::queueMS5541C() {
    if (_temp) {
        _ms5541C.queueD2();
    } else {
        _ms5541C.queueD1();
    }
    _queueTime = millis();
}

void Robot::readMS5541C() {
    _timeSinceQueuing = millis() - _queueTime;
    if (_timeSinceQueuing < 35) {
        delay(35 - _timeSinceQueuing);
    }
    
    if (_temp) {
        _ms5541C.readD2();
        _temp = !_temp;
    } else {
        _ms5541C.readD1();
        _pressure = _ms5541C.getPressure();
        _depthZ = _pressure / 9810;
		
		Serial.print("P: ");
		Serial.print(_pressure); Serial.print(" ");
		Serial.print(_depthZ); Serial.print(" ");
		Serial.println("");
        
		_temp = !_temp;
    }
}

/**
 *  Stabilizes the robot according to sensor readings
 */
void Robot::stabilize() {

    _dispX = getDispX();
    _dispY = getDispY();
    _dispZ = ((double)_velZ/128.0);
    
    // upon powerup, we wake up in the frozen state, till s/w commands otherwise
    double hackX = _dispX;
    double hackY = _dispY;
    double hackZ = _dispZ;
    if ((_mode & MODE_NORMAL_ENABLE)==0) {
        hackX -= _filtX;
        hackY -= _filtY;
    }
    hackZ -= _dispZ;    // ROHAN : KEEP THIS HERE TILL WE GET THE PRESSURE SENSOR WORKING
                        //         OTHERWISE STABILIZER LOOP WILL GO UNSTABLE. ONCE WE
                        //         HAVE THE SENSOR, THIS CAN GO BACK WITH THE OTHER HACKS

    if (_mode & MODE_STABILIZER_DISABLE) {
        hackX = 0.0;
                hackY = 0.0;
    }

        if (_mode & MODE_DEPTH_DISABLE) {
                hackZ = 0.0;
        }

    _filtX = _pidOutputX.compute(hackX);
    _filtY = _pidOutputY.compute(hackY);
    _filtZ = _pidDepth.compute(hackZ);

    // Map the X-Y filter outputs to the 4 metrics metricZ
    _stabZ[0] = (  _filtX) + (- _filtY); 
    _stabZ[1] = (  _filtX) + (  _filtY); 
    _stabZ[2] = (- _filtX) + (  _filtY); 
    _stabZ[3] = (- _filtX) + (- _filtY); 

    // Combine the stability control and depth control adjustments
    for (int i = 0; i < 4; i++) {

        _combZ[i] = 0.0;    

        if ((_mode & MODE_STABILIZER_DISABLE)==0) _combZ[i] += ((    _verticalCombinerRatio) * _stabZ[i]);
        if ((_mode & MODE_DEPTH_DISABLE     )==0) _combZ[i] += ((1 - _verticalCombinerRatio) * _filtZ   );
    }

    // Map the final adjustments to the different motors
    for (int i = 0; i < 4; i++) {

       double dblZ = (_combZ[i] * _outputScaleZ) + _outputOffsetZ + 0.5;

       if      (dblZ >  4095.0) { dblZ =  4095.0; }     // saturate positive
       else if (dblZ < -4095.0) { dblZ = -4095.0; }     // saturate negative

       int16_t intZ = ((int16_t) (dblZ));                       // convert to integer

       if      (intZ < 0) { _mzr[i] = -intZ; _mzf[i] =    0; }
       else if (intZ > 0) { _mzr[i] =     0; _mzf[i] = intZ; }
       else               { _mzr[i] =     0; _mzf[i] =    0; }
        
    }

    if (_mode & MODE_KILL) {
        setMotorU2(MZR1, 0);
        setMotorU2(MZR2, 0);
        setMotorU2(MZR3, 0);
        setMotorU2(MZR4, 0);
        
        setMotorU2(MZF1, 0);
        setMotorU2(MZF2, 0);
        setMotorU2(MZF3, 0);
        setMotorU2(MZF4, 0);
    } else {
        setMotorU2(MZR1, _mzr[0]);
        setMotorU2(MZR2, _mzr[1]);
        setMotorU2(MZR3, _mzr[2]);
        setMotorU2(MZR4, _mzr[3]);
        
        setMotorU2(MZF1, _mzf[0]);
        setMotorU2(MZF2, _mzf[1]);
        setMotorU2(MZF3, _mzf[2]);
        setMotorU2(MZF4, _mzf[3]);
    }
    
    // Log to serial port if flag is turned on
    if ((_mode & MODE_LOG_LEVEL) > 0) {
        
        Serial.print("S: ");
        Serial.print(_mode, HEX); Serial.print(" ");
        
        if ((_mode & MODE_LOG_LEVEL) > 2) {
            Serial.print(_accX         ); Serial.print(" ");
            Serial.print(_accY         ); Serial.print(" ");
            Serial.print(_accZ         ); Serial.print(" ");
            Serial.print(_gyroX        ); Serial.print(" ");
            Serial.print(_gyroY        ); Serial.print(" ");
            Serial.print(_velZ         ); Serial.print(" ");
            Serial.print(_pressure     ); Serial.print(" ");
            Serial.print(_accAngleX , 4); Serial.print(" ");
            Serial.print(_accAngleY , 4); Serial.print(" ");
            Serial.print(_gyroAngleX, 4); Serial.print(" ");
            Serial.print(_gyroAngleY, 4); Serial.print(" ");
            Serial.print(_combAngleX, 4); Serial.print(" ");
            Serial.print(_combAngleY, 4); Serial.print(" ");
            Serial.print(_dispX     , 4); Serial.print(" ");
            Serial.print(_dispY     , 4); Serial.print(" ");
            Serial.print(_dispZ     , 4); Serial.print(" ");
        }
        
        Serial.print(_filtX, 4); Serial.print(" ");
        Serial.print(_filtY, 4); Serial.print(" ");
        Serial.print(_filtZ, 4); Serial.print(" ");
        
        if ((_mode & MODE_LOG_LEVEL) > 1) {
            for (int i = 0; i < 4; i++) {
                Serial.print(_stabZ[i], 4); Serial.print(" ");
            }
            
            for (int i = 0; i < 4; i++) {
                Serial.print(_combZ[i], 4); Serial.print(" ");
            }
        }
        
        for (int i = 0; i < 4; i++) {
            Serial.print(_mzf[i]); Serial.print(" ");
            Serial.print(_mzr[i]); Serial.print(" ");
        }
        
        Serial.println("");
    }
    
    
}

/**
 *  Moves the robot according to values specified
 */
void Robot::move() {
    _gyroAngleZ = getDispZ();
    _combAngleZ = ((double)_rotZ * (PI/128)) - _gyroAngleZ;
    _rotR       = _combAngleZ/PI;
    
    double hackZ = _rotR;
    
    // upon powerup, we wake up in the frozen state, till s/w commands otherwise
    if ((_mode & MODE_NORMAL_ENABLE)==0) {
        hackZ -= _filtR;
    }
    
        if (_mode & MODE_ROTATION_DISABLE) {
                hackZ = 0.0;
        }

    _filtR = _pidAngle.compute(hackZ);

    if (_filtR > 0.0) {
        _rotXF[0] =  _filtR; _rotXR[2] =  _filtR;
        _rotXF[1] =  _filtR; _rotXR[3] =  _filtR;
        _rotYF[0] =  _filtR; _rotYR[2] =  _filtR;
        _rotYF[1] =  _filtR; _rotYR[3] =  _filtR;
        
        _rotXF[2] =     0.0; _rotXR[0] =     0.0;
        _rotXF[3] =     0.0; _rotXR[1] =     0.0;
        _rotYF[2] =     0.0; _rotYR[0] =     0.0;
        _rotYF[3] =     0.0; _rotYR[1] =     0.0;
    } else if (_filtR < 0.0) {
        _rotXF[0] =     0.0; _rotXR[2] =     0.0;
        _rotXF[1] =     0.0; _rotXR[3] =     0.0;
        _rotYF[0] =     0.0; _rotYR[2] =     0.0;
        _rotYF[1] =     0.0; _rotYR[3] =     0.0;
        
        _rotXF[2] = -_filtR; _rotXR[0] = -_filtR;
        _rotXF[3] = -_filtR; _rotXR[1] = -_filtR;
        _rotYF[2] = -_filtR; _rotYR[0] = -_filtR;
        _rotYF[3] = -_filtR; _rotYR[1] = -_filtR;
    } else {
        _rotXF[0] =     0.0; _rotXR[2] =     0.0;
        _rotXF[1] =     0.0; _rotXR[3] =     0.0;
        _rotYF[0] =     0.0; _rotYR[2] =     0.0;
        _rotYF[1] =     0.0; _rotYR[3] =     0.0;
        
        _rotXF[2] =     0.0; _rotXR[0] =     0.0;
        _rotXF[3] =     0.0; _rotXR[1] =     0.0;
        _rotYF[2] =     0.0; _rotYR[0] =     0.0;
        _rotYF[3] =     0.0; _rotYR[1] =     0.0;
    }
    
    _scaledVelX = (double)_velX/128.0;
    _scaledVelY = (double)_velY/128.0;

    if (_scaledVelX > 0.0) {
        _linXF[0] = _scaledVelX; _linXR[0] =          0.0;
        _linXF[1] = _scaledVelX; _linXR[1] =          0.0;
        _linXF[2] = _scaledVelX; _linXR[2] =          0.0;
        _linXF[3] = _scaledVelX; _linXR[3] =          0.0;
    } else if (_scaledVelX < 0.0) {
        _linXF[0] =         0.0; _linXR[0] = -_scaledVelX;
        _linXF[1] =         0.0; _linXR[1] = -_scaledVelX;
        _linXF[2] =         0.0; _linXR[2] = -_scaledVelX;
        _linXF[3] =         0.0; _linXR[3] = -_scaledVelX;
    } else {
        _linXF[0] =         0.0; _linXR[0] =          0.0;
        _linXF[1] =         0.0; _linXR[1] =          0.0;
        _linXF[2] =         0.0; _linXR[2] =          0.0;
        _linXF[3] =         0.0; _linXR[3] =          0.0;
    }
    
    if (_scaledVelY > 0.0) {
        _linYF[0] = _scaledVelY; _linYR[0] =          0.0;
        _linYF[1] = _scaledVelY; _linYR[1] =          0.0;
        _linYF[2] = _scaledVelY; _linYR[2] =          0.0;
        _linYF[3] = _scaledVelY; _linYR[3] =          0.0;
    } else if (_scaledVelY < 0.0) {
        _linYF[0] =         0.0; _linYR[0] = -_scaledVelY;
        _linYF[1] =         0.0; _linYR[1] = -_scaledVelY;
        _linYF[2] =         0.0; _linYR[2] = -_scaledVelY;
        _linYF[3] =         0.0; _linYR[3] = -_scaledVelY;
    } else {
        _linYF[0] =         0.0; _linYR[0] =          0.0;
        _linYF[1] =         0.0; _linYR[1] =          0.0;
        _linYF[2] =         0.0; _linYR[2] =          0.0;
        _linYF[3] =         0.0; _linYR[3] =          0.0;
    }
    
    double tmpXF[4];  
    double tmpXR[4]; 
    double tmpYF[4]; 
    double tmpYR[4]; 
    
    for (int i=0; i<4; i++) {

        tmpXF[i] = 0.0;
        tmpXR[i] = 0.0;
        tmpYF[i] = 0.0;
        tmpYR[i] = 0.0;

        if ((_mode & MODE_LINEAR_DISABLE)==0) {
            tmpXF[i] += (_horizontalCombinerRatio * _linXF[i]);
            tmpXR[i] += (_horizontalCombinerRatio * _linXR[i]);
            tmpYF[i] += (_horizontalCombinerRatio * _linYF[i]);
            tmpYR[i] += (_horizontalCombinerRatio * _linYR[i]);
        }

        if ((_mode & MODE_ROTATION_DISABLE)==0) {
            tmpXF[i] += ((1 - _horizontalCombinerRatio) * _rotXF[i]);
            tmpXR[i] += ((1 - _horizontalCombinerRatio) * _rotXR[i]);
            tmpYF[i] += ((1 - _horizontalCombinerRatio) * _rotYF[i]);
            tmpYR[i] += ((1 - _horizontalCombinerRatio) * _rotYR[i]);
        }

        tmpXF[i] = (_outputScaleXY * tmpXF[i]) + 0.5;
        tmpXR[i] = (_outputScaleXY * tmpXR[i]) + 0.5;
        tmpYF[i] = (_outputScaleXY * tmpYF[i]) + 0.5;
        tmpYR[i] = (_outputScaleXY * tmpYR[i]) + 0.5;
        
        if (tmpXF[i] > 4095.0) tmpXF[i] = 4095.0;
        if (tmpXR[i] > 4095.0) tmpXR[i] = 4095.0;
        if (tmpYF[i] > 4095.0) tmpYF[i] = 4095.0;
        if (tmpYR[i] > 4095.0) tmpYR[i] = 4095.0;
        
        _mxf[i] = ((int16_t)tmpXF[i]);
        _mxr[i] = ((int16_t)tmpXR[i]);
        _myf[i] = ((int16_t)tmpYF[i]);
        _myr[i] = ((int16_t)tmpYR[i]);
    }

    if (_mode & MODE_KILL) {
        setMotorU1(MXR1, 0);
        setMotorU1(MXR2, 0);
        setMotorU1(MXR3, 0);
        setMotorU1(MXR4, 0);

        setMotorU1(MXF1, 0);
        setMotorU1(MXF2, 0);
        setMotorU1(MXF3, 0);
        setMotorU1(MXF4, 0);

        setMotorU1(MYR1, 0);
        setMotorU1(MYR2, 0);
        setMotorU1(MYR3, 0);
        setMotorU1(MYR4, 0);

        setMotorU1(MYF1, 0);
        setMotorU1(MYF2, 0);
        setMotorU1(MYF3, 0);
        setMotorU1(MYF4, 0);
    } else {
        setMotorU1(MXR1, _mxr[0]);
        setMotorU1(MXR2, _mxr[1]);
        setMotorU1(MXR3, _mxr[2]);
        setMotorU1(MXR4, _mxr[3]);

        setMotorU1(MXF1, _mxf[0]);
        setMotorU1(MXF2, _mxf[1]);
        setMotorU1(MXF3, _mxf[2]);
        setMotorU1(MXF4, _mxf[3]);

        setMotorU1(MYR1, _myr[0]);
        setMotorU1(MYR2, _myr[1]);
        setMotorU1(MYR3, _myr[2]);
        setMotorU1(MYR4, _myr[3]);

        setMotorU1(MYF1, _myf[0]);
        setMotorU1(MYF2, _myf[1]);
        setMotorU1(MYF3, _myf[2]);
        setMotorU1(MYF4, _myf[3]);
    }


    // Log to serial port if flag is turned on
    if ((_mode & MODE_LOG_LEVEL) > 0) {

        Serial.print("M: ");
                Serial.print(_mode, HEX); Serial.print(" ");
                
        if ((_mode & MODE_LOG_LEVEL) > 2) {
            Serial.print(_gyroZ);         Serial.print(" ");
            Serial.print(_gyroAngleZ, 4); Serial.print(" ");
            Serial.print(_combAngleZ, 4); Serial.print(" ");
            Serial.print(_rotR      , 4); Serial.print(" ");
            Serial.print(_velX);          Serial.print(" ");
            Serial.print(_velY);          Serial.print(" ");
        }

        Serial.print(_filtR     , 4); Serial.print(" ");
        Serial.print(_scaledVelX, 4); Serial.print(" ");
        Serial.print(_scaledVelY, 4); Serial.print(" ");
        
        if ((_mode & MODE_LOG_LEVEL) > 1) {
            for (int i = 0; i < 4; i++) {
                Serial.print(_rotXF[i], 4); Serial.print(" ");
                Serial.print(_rotXR[i], 4); Serial.print(" ");
            }

            for (int i = 0; i < 4; i++) {
                Serial.print(_rotYF[i], 4); Serial.print(" ");
                Serial.print(_rotYR[i], 4); Serial.print(" ");
            }

            for (int i = 0; i < 4; i++) {
                Serial.print(_linXF[i], 4); Serial.print(" ");
                Serial.print(_linXR[i], 4); Serial.print(" ");
            }

            for (int i = 0; i < 4; i++) {
                Serial.print(_linYF[i], 4); Serial.print(" ");
                Serial.print(_linYR[i], 4); Serial.print(" ");
            }
        }
        
        for (int i = 0; i < 4; i++) {
            Serial.print(_mxf[i]); Serial.print(" ");
            Serial.print(_mxr[i]); Serial.print(" ");
        }

        for (int i = 0; i < 4; i++) {
            Serial.print(_myf[i]); Serial.print(" ");
            Serial.print(_myr[i]); Serial.print(" ");
        }
        
        Serial.println("");
    }

    
}

void Robot::normalOperation() {
	queueMS5541C();
	updateMPU9150();
	updateDt();
	stabilize();
	move();
	readMS5541C();
}

void Robot::motorTest() {
	stop();
	for (int i = 15; i > -1; i--) {
    
    _pwmU1.setPWM(i, 0, 2048);
    
    Serial.print("U1: "); Serial.println(i);
    Serial.println(_pwmU1.getMode1(), HEX);
    Serial.println(_pwmU1.getMode2(), HEX);
    
    delay(5000);
    _pwmU1.setPWM(i, 0, 0);
    delay(1000);
  }
  for (int i = 15; i > 7; i--) {
    
    _pwmU2.setPWM(i, 0, 2048);
    
    Serial.print("U2: "); Serial.println(i);
    Serial.println(_pwmU2.getMode1(), HEX);
    Serial.println(_pwmU2.getMode2(), HEX);
    delay(5000);
    _pwmU2.setPWM(i, 0, 0);
    delay(1000);
  }
}

void Robot::imuTest() {
	
	stop();
	updateMPU9150();

	Serial.print(_accX); Serial.print(" ");
	Serial.print(_accY); Serial.print(" ");
	Serial.print(_accZ); Serial.print(" ");
	Serial.print(_gyroX); Serial.print(" ");
	Serial.print(_gyroY); Serial.print(" ");
	Serial.print(_gyroZ); Serial.print(" ");
	Serial.print(_magX); Serial.print(" ");
	Serial.print(_magY); Serial.print(" ");
	Serial.print(_magZ); Serial.print(" ");
	Serial.println("");
}

void Robot::pressureTest() {
	_ms5541C.queueD2();
	delay(35);
	_ms5541C.readD2();
	_ms5541C.queueD1();
	delay(35);
	_ms5541C.readD1();
 
	Serial.println(_ms5541C.getTemperature());
	Serial.println(_ms5541C.getPressure());
}


/**
 *  Sets a motor on U1 to a desired value
 *
 *  @param motor Which motor to set
 *  @param value What speed to drive the motor at
 */
void Robot::setMotorU1(MotorU1 motor, int16_t value) {
    _pwmU1.setPWM(motor, motor * 256, ((motor * 256) + value) % 4096);
}

/**
 *  Sets a motor on U2 to a desired value
 *
 *  @param motor Which motor to set
 *  @param value What speed to drive the motor at
 */
void Robot::setMotorU2(MotorU2 motor, int16_t value) {
    _pwmU2.setPWM(motor, motor * 256, ((motor * 256) + value) % 4096);
}

















