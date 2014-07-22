//
//  PID.cpp
//  
//
//  Created by Rahul Salvi on 7/19/14.
//
//

#include "PID.h"

/**
 *  Constructor
 */
PID::PID(double kp, double ki, double kd, double kf) {
    _stateI = 0;
    _stateD = 0;

    _kp = kp;
    _ki = ki;
    _kd = kd;
    _kf = kf;
}

/**
 *  Computes the output given the error
 *
 *  @param err The error in the system
 *
 *  @return The output to drive at
 */
double PID::compute(double err) {
    double xp, xi, xd;
    
    xp = _kp * err;
    xi = (_ki * err) + _stateI;
    xd = _kd * (err - _stateD);
    
    _stateI = xi;
    _stateD = err;
    
    return xp + xi + xd + _kf;
}
