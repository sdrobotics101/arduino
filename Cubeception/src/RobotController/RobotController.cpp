//
//  RobotController.cpp
//  
//
//  Created by Rahul Salvi on 7/19/14.
//
//

#include "RobotController.h"

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
 *  @param rxSerialPort            Which serial port to read from
 *  @param txSerialPort            Which serial port to write to
 *  @param badPacketThreshold      The number of bad packets allowed before error
 *
 *  @return Nothing
 */
RobotController::RobotController(uint8_t mpuAddr,
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
                                 
                                 USARTClass &rxSerialPort,
                                 USARTClass &txSerialPort,
                                 
                                 uint8_t badPacketThreshold) :

                                    _robot(mpuAddr,
                                           pwmU1Addr,
                                           pwmU2Addr,
                                           
                                           pidOutputXKP,
                                           pidOutputXKI,
                                           pidOutputXKD,
                                           pidOutputXKF,
                                           
                                           pidOutputYKP,
                                           pidOutputYKI,
                                           pidOutputYKD,
                                           pidOutputYKF,
                                           
                                           pidDepthKP,
                                           pidDepthKI,
                                           pidDepthKD,
                                           pidDepthKF,
                                           
                                           pidAngleKP,
                                           pidAngleKI,
                                           pidAngleKD,
                                           pidAngleKF,
                                           
                                           dispXYRatio,
                                           verticalCombinerRatio,
                                           horizontalCombinerRatio,
                                           
                                           outputScaleXY,
                                           outputScaleZ,
                                           outputOffsetZ),

                                    _packetController(rxSerialPort,
                                                      txSerialPort),

                                    _badPacketThreshold(badPacketThreshold)
{
    _packetStatus   = NOT_ENOUGH_DATA;
    _badPacketCount = 0;
}

/**
 *  Initializes RobotController
 */
void RobotController::begin() {
    Serial.begin(115200);
    if (Serial) {
        Serial.println("Serial Initialized");
    } else {
        Serial.println("Serial Failed to Initialize"); //How can this actually output?
    }
    
    _robot.begin();
    _packetController.begin();
    Serial.println("RobotController Initialized");
}

/**
 *  Executes a single cycle of the robot loop
 */
void RobotController::executeCycle() {
    _packetStatus = _packetController.listen();

    switch (_packetStatus) {
        case VALID_PACKET:
            _robot.setMotion(_packetController.getS8(VELX),
                             _packetController.getS8(VELY),
                             _packetController.getS8(VELZ),
                             _packetController.getS8(ROTZ),
                             _packetController.getU8(TORPEDOCTL),
                             _packetController.getU8(SERVOCTL),
                             _packetController.getU8(LEDCTL),
							 _packetController.getU16(MODE));
        break;
            
        case INVALID_PACKET:
            _badPacketCount += 3;
            _robot.continueMotion();
        break;
            
        case NO_HEADER_FOUND:
            _badPacketCount += 2;
            _robot.continueMotion();
        break;
            
        case NOT_ENOUGH_DATA:
            _badPacketCount += 1;
            _robot.continueMotion();
        break;
            
    }
    
    if (_badPacketThreshold > -1) {
        if (_badPacketCount > _badPacketThreshold) {
            _robot.stop();
            _badPacketCount = 0;
        }
    }
	
    _packetController.setS16(MAGX, _robot.getMagX());
    _packetController.setS16(MAGY, _robot.getMagY());
    _packetController.setS16(MAGZ, _robot.getMagZ());
    _packetController.setU16(POSZ, _robot.getPosZ());
    _packetController.setU16(HEALTH, 1);
    _packetController.setU8(BATV, analogRead(A0) / 16);
    _packetController.send();
}

void RobotController::calibrate() {
	_robot.calibrate();
}

void RobotController::stop() {
	_robot.stop();
}














