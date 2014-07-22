//
//  RobotController.cpp
//  
//
//  Created by Rahul Salvi on 7/19/14.
//
//

#include "RobotController.h"

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

void RobotController::executeCycle() {
    _packetStatus = _packetController.listen();

    switch (_packetStatus) {
        case VALID_PACKET:
            int8_t servoCtl[6];
            for (int i = 0; i < 6; i++) {
                servoCtl[i] = _packetController.get8((RXIndex)(SERVOCTL+i));
            }
            _robot.setMotion(_packetController.get8(VELX),
                             _packetController.get8(VELY),
                             _packetController.get8(VELZ),
                             _packetController.get8(ROTX),
                             _packetController.get8(ROTY),
                             _packetController.get8(ROTZ),
                             _packetController.get16(POSZ),
                             _packetController.get8(TORPEDOCTL),
                             servoCtl,
                             _packetController.get8(RXSPARE));
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
}
















