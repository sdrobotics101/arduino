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
PID::PID(double kp,
         double ki,
         double kd,
         double kf) :
            _kp(kp),
            _ki(ki),
            _kd(kd),
            _kf(kf)
{
    _stateI = 0.0;
    _stateD = 0.0;
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
    
    if (xp > MAX_SAT_VAL) {
        xp = MAX_SAT_VAL;
    } else if (xp < (-1 * MAX_SAT_VAL)) {
        xp = (-1 * MAX_SAT_VAL);
    }
    
    if (xi > MAX_SAT_VAL) {
        xi = MAX_SAT_VAL;
    } else if (xi < (-1 * MAX_SAT_VAL)) {
        xi = (-1 * MAX_SAT_VAL);
    }
    
    if (xd > MAX_SAT_VAL) {
        xd = MAX_SAT_VAL;
    } else if (xd < (-1 * MAX_SAT_VAL)) {
        xd = (-1 * MAX_SAT_VAL);
    }
    _stateI = xi;
    _stateD = err;
    
    return xp + xi + xd + _kf;
}

void PID::reset() {
    _stateI = 0.0;
    _stateD = 0.0;
}