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

#include "Robot.h"
#include "PacketController.h"

class RobotController {
public:
    RobotController();
    void begin();
    void executeCycle();
};


#endif /* defined(____RobotController__) */