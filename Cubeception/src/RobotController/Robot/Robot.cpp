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
     _velX      = 0;
     _velY      = 0;
     _velZ      = 0;
     _rotX      = 0;
     _rotY      = 0;
     _rotZ      = 0;
     _posZ      = 0;
    
     _torpedoCtl    = 0;
     for (int i=0; i<6; i++) {
        _servoCtl[i] = 0;
     }
     _mode          = 1;
    
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
    move();
}

/**
 *  Continues previously set motion
 */
void Robot::continueMotion() {
    updateMPU9150();
    updateDt();
    stabilize();
    move();
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
    _dispZ = ((double)_posZ/8.0) - ((double)_pressure*(14.0/65536)*10.1976);

    
    // upon powerup, we wake up in the frozen state, till s/w commands otherwise
    double hackX = _dispX;
    double hackY = _dispY;
    double hackZ = _dispZ;
    if ((_mode & MODE_NORMAL_ENABLE)==0) {
        hackX -= _filtX;
        hackY -= _filtY;
    }
    hackZ -= _dispZ;	// ROHAN : KEEP THIS HERE TILL WE GET THE PRESSURE SENSOR WORKING
                        //         OTHERWISE STABILIZER LOOP WILL GO UNSTABLE. ONCE WE
			//         HAVE THE SENSOR, THIS CAN GO BACK WITH THE OTHER HACKS

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
    if ((_mode & MODE_LOG_LEVEL) > 0) {
        
        Serial.print("S: ");
        
        if ((_mode & MODE_LOG_LEVEL) > 2) {
            Serial.print(_accX         ); Serial.print(" ");
            Serial.print(_accY         ); Serial.print(" ");
            Serial.print(_accZ         ); Serial.print(" ");
            Serial.print(_gyroX        ); Serial.print(" ");
            Serial.print(_gyroY        ); Serial.print(" ");
            Serial.print(_posZ         ); Serial.print(" ");
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
    _rotR       = sin(_combAngleZ);
    
    double hackZ = _rotR;
    
    // upon powerup, we wake up in the frozen state, till s/w commands otherwise
    if ((_mode & MODE_NORMAL_ENABLE)==0) {
        hackZ -= _filtR;
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
    }
    else if (_filtR < 0.0) {
        _rotXF[0] =     0.0; _rotXR[2] =     0.0;
        _rotXF[1] =     0.0; _rotXR[3] =     0.0;
        _rotYF[0] =     0.0; _rotYR[2] =     0.0;
        _rotYF[1] =     0.0; _rotYR[3] =     0.0;
        
        _rotXF[2] = -_filtR; _rotXR[0] = -_filtR;
        _rotXF[3] = -_filtR; _rotXR[1] = -_filtR;
        _rotYF[2] = -_filtR; _rotYR[0] = -_filtR;
        _rotYF[3] = -_filtR; _rotYR[1] = -_filtR;
    }
    else {
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

    // Log to serial port if flag is turned on
    if ((_mode & MODE_LOG_LEVEL) > 0) {

        Serial.print("M: ");

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

















