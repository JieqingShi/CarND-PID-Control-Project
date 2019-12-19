#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error = cte - p_error;
  i_error += cte;
  p_error = cte;

}

double PID::TotalError(){
  /**
   * TODO: Calculate and return the total error
   */
  double output = -Kp * p_error - Kd * d_error - Ki * i_error;

  if(output < -1){
    output = -1;
  }
  else if(output > 1){
    output = 1;
  }
  return output;
}


double PID::Twiddle(double tol, double param, double err){
  // Let's run it only on Kp first
  double dp = 1;
  double delta_plus = 1.1;
  double delta_minus = 0.9;
  double best_err = 10000;

  if(dp > tol){
    param += dp;  //bump up
    if(err < best_err){
      best_err = err;  // bumping up helped improving the error
      dp *= delta_plus;
    }
    else{
      param -= 2*dp;  // bumping up did not help improve the error -> thus going down
      if(err < best_err){
        best_err = err;
        dp *= delta_plus;
      }
      else{
        param += dp;
        dp *= delta_minus;
      }
    }

  }
  return param;
}