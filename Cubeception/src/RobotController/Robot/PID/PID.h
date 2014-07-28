//
//  PID.h
//  
//
//  Created by Rahul Salvi on 7/19/14.
//
//

#ifndef ____PID__
#define ____PID__

#include <stdint.h>
#include <math.h>

#define MAX_SAT_VAL 20.0

/**
 *  A class for controlling PID loops
 */
class PID {
public:
    PID(double kp, double ki, double kd, double kf);
    double compute(double err);
    void reset();
    
    void setKP(double kp);
    void setKI(double ki);
    void setKD(double kd);
    void setKF(double kf);
    
    double getKP();
    double getKI();
    double getKD();
    double getKF();
    
private:
    double _kp;
    double _ki;
    double _kd;
    double _kf;
    double _stateI;
    double _stateD;
};


#endif /* defined(____PID__) */
