#include "PID.h"
#include "math.h"
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  
}

void PID::UpdateError(double cte) {
  
  d_error = cte - p_error;
  i_error += cte;
  p_error = cte;

}

void PID::SetLimit(double lower_lim, double upper_lim){
  lower_limit = lower_lim;
  upper_limit = upper_lim;
}


double PID::TotalError(){
  
  double output = -Kp * p_error - Kd * d_error - Ki * i_error;

  if(limit_output){
    if(output < lower_limit){
      output = lower_limit;
    }
    else if(output > upper_limit){
      output = upper_limit;
    }
  }
  return output;
}

