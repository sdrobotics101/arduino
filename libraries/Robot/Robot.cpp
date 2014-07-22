//
//  Robot.cpp
//  
//
//  Created by Rahul Salvi on 7/19/14.
//
//

#include "Robot.h"

/**
 *  Robot constructor
 *
 *  @param mpuAddr          I2C Address of MPU9150
 *  @param pwmU1Addr        I2C Address of PCA9685 on U1
 *  @param pwmU2Addr        I2C Address of PCA9685 on U2
 *  @param pidOutputXKP     Kp value for PID output X controller
 *  @param pidOutputXKI     Ki value for PID output X controller
 *  @param pidOutputXKD     Kd value for PID output X controller
 *  @param pidOutputXKF     Kf value for PID output X controller
 *  @param pidOutputYKP     Kp value for PID output Y controller
 *  @param pidOutputYKI     Ki value for PID output Y controller
 *  @param pidOutputYKD     Kd value for PID output Y controller
 *  @param pidOutputYKF     Kf value for PID output Y controller
 *  @param pidDepthKP       Kp value for PID depth controller
 *  @param pidDepthKI       Ki value for PID depth controller
 *  @param pidDepthKD       Kd value for PID depth controller
 *  @param pidDepthKF       Kf value for PID depth controller
 *  @param verticalCombinerRatio Vertical combiner constant
 *  @param dispXYRatio      Complementary filter constant
 *  @param outputScaleZ     Output scale Z
 *  @param outputOffsetZ    Output offset Z
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
     _velX      = 0;
     _velY      = 0;
     _velZ      = 0;
     _rotX      = 0;
     _rotY      = 0;
     _rotZ      = 0;
     _posZ      = 0;
    
     _torpedoCtl    = 0;
     _servoCtl[6]   = 0;
    
    _accOffsetX  = 0;   _accOffsetY  = 0;   _accOffsetZ  = 0;
    _gyroOffsetX = 0;   _gyroOffsetY = 0;   _gyroOffsetZ = 0;
    
    _accX  = 0; _accY  = 0; _accZ  = 0;
    _gyroX = 0; _gyroY = 0; _gyroZ = 0;
    
    _accAngleX   = 0.0; _accAngleY   = 0.0;
    _gyroAngleX  = 0.0; _gyroAngleY  = 0.0;
    _combAngleX  = 0.0; _combAngleY  = 0.0;
    _dispX       = 0.0; _dispY       = 0.0;
    _filtX       = 0.0; _filtY       = 0.0;
    
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
    
    _mode = 1; //set back to 0 for final
}

/**
 *  Initializes I2C bus, sensors, actuators
 */
void Robot::begin() {
    Wire.begin();
    Serial.println("Wire Initialized");
    
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

/**
 *  Input to create desired motion
 *
 *  @param velX       Velocity X
 *  @param velY       Velocity Y
 *  @param velZ       Velocity Z
 *  @param rotX       Rotation X
 *  @param rotY       Rotation Y
 *  @param rotZ       Rotation Z
 *  @param posZ       Position Z
 *  @param torpedoCtl Torpedo control
 *  @param servoCtl   Servo control
 *  @param mode       Mode control
 */
void Robot::setMotion(int8_t velX,
                      int8_t velY,
                      int8_t velZ,
                      int8_t rotX,
                      int8_t rotY,
                      int8_t rotZ,
                      int16_t posZ,
                      int8_t torpedoCtl,
                      int8_t servoCtl[6],
                      uint8_t mode)
{
    _velX = velX;
    _velY = velY;
    _velZ = velZ;
    _rotX = rotX;
    _rotY = rotY;
    _rotZ = rotZ;
    _posZ = posZ;
    _torpedoCtl = torpedoCtl;
    _mode = mode;
    for (int i = 0; i < 6; i++) {
        _servoCtl[i] = servoCtl[i];
    }
    
    updateMPU9150();
    updateDt();
    stabilize();
    //Write velX, velY, rotX, rotY
    //Other commands
    
}

/**
 *  Continues previously set motion
 */
void Robot::continueMotion() {
    updateMPU9150();
    updateDt();
    stabilize();
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
    _mpu9150.getMotion6(&_accX, &_accY, &_accZ, &_gyroX, &_gyroY, &_gyroZ);
}

/**
 *  Calculates the angle along the X rotation axis
 *
 *  @return The sine of the X angle
 */
double Robot::getDispX() {
    double scaledGyroX = (double) (_gyroX - _gyroOffsetX) / 65.54;

    _accAngleX   = atan2((double)_accY, sqrt(pow((double)_accX, 2) + pow((double)_accZ, 2)));
    _gyroAngleX	 = scaledGyroX * _dt * (PI/180);
    _combAngleX  = (_dispXYRatio  * (_gyroAngleX + _combAngleX)) + ((1 - _dispXYRatio) * _accAngleX);

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
    _gyroAngleY	 = scaledGyroY * _dt * (PI/180);
    _combAngleY  = (_dispXYRatio  * (_gyroAngleY + _combAngleY)) + ((1 - _dispXYRatio) * _accAngleY);
    
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

/**
 *  Stabilizes the robot according to sensor readings
 */
void Robot::stabilize() {

    _dispX = getDispX();
    _dispY = getDispY();

    double hackX = _dispX;
    double hackY = _dispY;
    
    // upon powerup, we wake up in the frozen state, till s/w commands otherwise
    if ((_mode & (1<<2))==0) {
        hackX -= _filtX;
        hackY -= _filtY;
    }

    _filtX = _pidOutputX.compute(hackX);
    _filtY = _pidOutputY.compute(hackY);

    // Map the X-Y filter outputs to the 4 metrics metricZ
    _stabZ[0] = (  _filtX) + (- _filtY); 
    _stabZ[1] = (  _filtX) + (  _filtY); 
    _stabZ[2] = (- _filtX) + (  _filtY); 
    _stabZ[3] = (- _filtX) + (- _filtY); 

    // Combine the stability control and depth control adjustments
    for (int i = 0; i < 4; i++) {
        _combZ[i] = _stabZ[i];
    }

    // Map the final adjustments to the different motors
    for (int i = 0; i < 4; i++) {

       double dblZ = (_combZ[i] * _outputScaleZ) + _outputOffsetZ + 0.5;

       if      (dblZ >  4095.0) { dblZ =  4095.0; }	// saturate positive
       else if (dblZ < -4095.0) { dblZ = -4095.0; }	// saturate negative

       int16_t intZ = ((int16_t) (dblZ));			// convert to integer

       if      (intZ < 0) { _mzr[i] = -intZ; _mzf[i] =    0; }
       else if (intZ > 0) { _mzr[i] =     0; _mzf[i] = intZ; }
       else               { _mzr[i] =     0; _mzf[i] =    0; }
        
    }
    
    setMotorU2(MZR1, _mzr[0]);
    setMotorU2(MZR2, _mzr[1]);
    setMotorU2(MZR3, _mzr[2]);
    setMotorU2(MZR4, _mzr[3]);
    
    setMotorU2(MZF1, _mzf[0]);
    setMotorU2(MZF2, _mzf[1]);
    setMotorU2(MZF3, _mzf[2]);
    setMotorU2(MZF4, _mzf[3]);
    
    
    // Log to serial port if flag is turned on
    if ((_mode & 0x3) > 0) {
        
        Serial.print("S: ");
        
        if ((_mode & 0x3) > 2) {
            Serial.print(_accX ); Serial.print(" ");
            Serial.print(_accY ); Serial.print(" ");
            Serial.print(_accZ ); Serial.print(" ");
            Serial.print(_gyroX); Serial.print(" ");
            Serial.print(_gyroY); Serial.print(" ");
            Serial.print(_gyroZ); Serial.print(" ");
            Serial.print(_accAngleX , 4); Serial.print(" ");
            Serial.print(_accAngleY , 4); Serial.print(" ");
            Serial.print(_gyroAngleX, 4); Serial.print(" ");
            Serial.print(_gyroAngleY, 4); Serial.print(" ");
            Serial.print(_combAngleX, 4); Serial.print(" ");
            Serial.print(_combAngleY, 4); Serial.print(" ");
            Serial.print(_dispX, 4); Serial.print(" ");
            Serial.print(_dispY, 4); Serial.print(" ");
        }
        
        Serial.print(_filtX, 4); Serial.print(" ");
        Serial.print(_filtY, 4); Serial.print(" ");
        
        if ((_mode & 0x3) > 1) {
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

void Robot::move() {
    _gyroAngleZ = getDispZ();
    _combAngleZ = ((double)_rotZ * (PI/128)) - _gyroAngleZ;
    _rotR = sin(_combAngleZ);
    
    double hackZ = _rotR;
    
    // upon powerup, we wake up in the frozen state, till s/w commands otherwise
    if ((_mode & (1<<2))==0) {
        hackZ -= _filtR;
    }
    
    _filtR = _pidAngle.compute(hackZ);
    
    if (_filtR > 0.0) {
        _rotXF[0] =  _filtR; _rotXR[2] =  _filtR;
        _rotXF[1] =  _filtR; _rotXR[3] =  _filtR;
        _rotYF[0] =  _filtR; _rotYR[0] =  _filtR;
        _rotYF[1] =  _filtR; _rotYR[1] =  _filtR;
        
        _rotXF[2] =     0.0; _rotXR[0] =     0.0;
        _rotXF[3] =     0.0; _rotXR[1] =     0.0;
        _rotYF[2] =     0.0; _rotXR[0] =     0.0;
        _rotYF[3] =     0.0; _rotXR[1] =     0.0;
    }
    else if (_filtR < 0.0) {
        _rotXF[0] =     0.0; _rotXR[2] =     0.0;
        _rotXF[1] =     0.0; _rotXR[3] =     0.0;
        _rotYF[0] =     0.0; _rotYR[0] =     0.0;
        _rotYF[1] =     0.0; _rotYR[1] =     0.0;
        
        _rotXF[2] = -_filtR; _rotXR[0] = -_filtR;
        _rotXF[3] = -_filtR; _rotXR[1] = -_filtR;
        _rotYF[2] = -_filtR; _rotXR[0] = -_filtR;
        _rotYF[3] = -_filtR; _rotXR[1] = -_filtR;
    }
    else {
        _rotXF[0] =     0.0; _rotXR[2] =     0.0;
        _rotXF[1] =     0.0; _rotXR[3] =     0.0;
        _rotYF[0] =     0.0; _rotYR[0] =     0.0;
        _rotYF[1] =     0.0; _rotYR[1] =     0.0;
        
        _rotXF[2] =     0.0; _rotXR[0] =     0.0;
        _rotXF[3] =     0.0; _rotXR[1] =     0.0;
        _rotYF[2] =     0.0; _rotXR[0] =     0.0;
        _rotYF[3] =     0.0; _rotXR[1] =     0.0;
    }
    
    _scaledVelX = (double)_velX/128;
    _scaledVelY = (double)_velY/128;
    
    if (_scaledVelX > 0.0) {
        _linXF[0] = _scaledVelX; _linXR[0] =          0.0;
        _linXF[1] = _scaledVelX; _linXR[1] =          0.0;
        _linXF[2] = _scaledVelX; _linXR[2] =          0.0;
        _linXF[3] = _scaledVelX; _linXR[3] =          0.0;
    }
    else if (_scaledVelX < 0.0) {
        _linXF[0] =         0.0; _linXR[0] = -_scaledVelX;
        _linXF[1] =         0.0; _linXR[1] = -_scaledVelX;
        _linXF[2] =         0.0; _linXR[2] = -_scaledVelX;
        _linXF[3] =         0.0; _linXR[3] = -_scaledVelX;
    }
    else {
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
    }
    else if (_scaledVelY < 0.0) {
        _linYF[0] =         0.0; _linYR[0] = -_scaledVelY;
        _linYF[1] =         0.0; _linYR[1] = -_scaledVelY;
        _linYF[2] =         0.0; _linYR[2] = -_scaledVelY;
        _linYF[3] =         0.0; _linYR[3] = -_scaledVelY;
    }
    else {
        _linXF[0] =         0.0; _linYR[0] =          0.0;
        _linXF[1] =         0.0; _linYR[1] =          0.0;
        _linXF[2] =         0.0; _linYR[2] =          0.0;
        _linXF[3] =         0.0; _linYR[3] =          0.0;
    }
    
    
    double tmpXF[4];  
    double tmpXR[4]; 
    double tmpYF[4]; 
    double tmpYR[4]; 
    
    for (int i=0; i<4; i++) {
        
        tmpXF[i] = (_outputScaleXY * ((_horizontalCombinerRatio * _linXF[i]) + ((1 - _horizontalCombinerRatio) * _rotXF[i]))) + 0.5;
        tmpXR[i] = (_outputScaleXY * ((_horizontalCombinerRatio * _linXR[i]) + ((1 - _horizontalCombinerRatio) * _rotXR[i]))) + 0.5;
        tmpYF[i] = (_outputScaleXY * ((_horizontalCombinerRatio * _linYF[i]) + ((1 - _horizontalCombinerRatio) * _rotYF[i]))) + 0.5;
        tmpYR[i] = (_outputScaleXY * ((_horizontalCombinerRatio * _linYR[i]) + ((1 - _horizontalCombinerRatio) * _rotYR[i]))) + 0.5;
        
        if (tmpXF[i] > 4095.0) tmpXF[i] = 4095.0;
        if (tmpXR[i] > 4095.0) tmpXR[i] = 4095.0;
        if (tmpYF[i] > 4095.0) tmpYF[i] = 4095.0;
        if (tmpYR[i] > 4095.0) tmpYR[i] = 4095.0;
        
        _mxf[i] = ((int16_t)tmpXF[i]);
        _mxr[i] = ((int16_t)tmpXR[i]);
        _myf[i] = ((int16_t)tmpYF[i]);
        _myr[i] = ((int16_t)tmpYR[i]);
    }
    
    
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

















