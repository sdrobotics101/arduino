//
//  Cubeception.h
//  
//
//  Created by Rahul Salvi on 7/22/14.
//
//

#ifndef ____Cubeception__
#define ____Cubeception__

#include "Arduino.h"

#include "RobotController/RobotController.h"

#define MPU_ADDR ((uint8_t)0x68)
#define PWMU1_ADDR ((uint8_t)0x79)
#define PWMU2_ADDR ((uint8_t)0x71)

#define PID_OUTPUT_XKP ((double)10/16)
#define PID_OUTPUT_XKI ((double)5/16)
#define PID_OUTPUT_XKD ((double)1/16)
#define PID_OUTPUT_XKF ((double)0/16)
                       
#define PID_OUTPUT_YKP ((double)10/16)
#define PID_OUTPUT_YKI ((double)5/16)
#define PID_OUTPUT_YKD ((double)1/16)
#define PID_OUTPUT_YKF ((double)0/16)
                       
#define PID_DEPTH_KP ((double)16/16)
#define PID_DEPTH_KI ((double)0/16)
#define PID_DEPTH_KD ((double)0/16)
#define PID_DEPTH_KF ((double)0/16)
                     
#define PID_ANGLE_KP ((double)8/16)
#define PID_ANGLE_KI ((double)8/16)
#define PID_ANGLE_KD ((double)0/16)
#define PID_ANGLE_KF ((double)0/16)
                     
#define DISP_XY_RATIO ((double)0.8)
#define VERTICAL_COMBINER_RATIO ((double)0.4)
#define HORIZONTAL_COMBINER_RATIO ((double)0.9)
                                  
#define OUTPUT_SCALE_XY ((double)4096.0)
#define OUTPUT_SCALE_Z ((double)4096.0)
#define OUTPUT_OFFSET_Z ((double)0.0)
                        
#define RX_SERIAL_PORT Serial3
#define TX_SERIAL_PORT Serial3

#define BAD_PACKET_THRESHOLD ((uint8_t)100)

#endif /* defined(____Cubeception__) */