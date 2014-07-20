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

class PID {
public:
    PID(double kp, double ki, double kd, double kf);
    double compute(double err);
    
private:
    double _kp;
    double _ki;
    double _kd;
    double _kf;
    double stateI = 0;
    double stateD = 0;
};


#endif /* defined(____PID__) */