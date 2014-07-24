#include <Wire.h>
#include <Cubeception.h>

RobotController robotController(MPU_ADDR,
                                PWMU1_ADDR,
                                PWMU2_ADDR,
                                
                                PID_OUTPUT_XKP,
                                PID_OUTPUT_XKI,
                                PID_OUTPUT_XKD,
                                PID_OUTPUT_XKF,
                                
                                PID_OUTPUT_YKP,
                                PID_OUTPUT_YKI,
                                PID_OUTPUT_YKD,
                                PID_OUTPUT_YKF,
                                
                                PID_DEPTH_KP,
                                PID_DEPTH_KI,
                                PID_DEPTH_KD,
                                PID_DEPTH_KF,
                                
                                PID_ANGLE_KP,
                                PID_ANGLE_KI,
                                PID_ANGLE_KD,
                                PID_ANGLE_KF,
                                
                                DISP_XY_RATIO,
                                VERTICAL_COMBINER_RATIO,
                                HORIZONTAL_COMBINER_RATIO,
                                
                                OUTPUT_SCALE_XY,
                                OUTPUT_SCALE_Z,
                                OUTPUT_OFFSET_Z,
                                
                                RX_SERIAL_PORT,
                                TX_SERIAL_PORT,
                                
                                BAD_PACKET_THRESHOLD);
                                

void setup() {
  robotController.begin();
  robotController.stop();
}

void loop() {
  robotController.executeCycle();

}
