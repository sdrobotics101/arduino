//
//  RobotController.h
//  
//
//  Created by Rahul Salvi on 7/19/14.
//
//

#ifndef ____RobotController__
#define ____RobotController__

#include "Arduino.h"

#include <vector>
#include <string>
#include <sstream>

#include "Robot/Robot.h"
#include "PacketController/PacketController.h"

/**
 *  Complete controller for robot and communications
 */
class RobotController {
public:
    RobotController(uint8_t mpuAddr,
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
                    
                    uint16_t badPacketTimeout);
    void begin();
    void executeCycle();
	void calibrate();
	void stop();
private:
    
    void setConstants(std::vector<double> vect);
    
    const uint16_t _badPacketTimeout;
    uint16_t      _packetCount;
    
    unsigned long _startTime;
    unsigned long _currentTime;
    
    Robot _robot;
    PacketController _packetController;
};


#endif /* defined(____RobotController__) */