#include "PID.h"
#include "math.h"
#include <iostream>

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
  Kp_twiddle = Kp_;
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
  cross_track_error = (double)cte;

}

double PID::TotalError(){
  /**
   * TODO: Calculate and return the total error
   */
  // double output = -Kp * p_error - Kd * d_error - Ki * i_error;
  double output = -Kp_twiddle * p_error - Kd * d_error - Ki * i_error;

  if(output < -1){
    output = -1;
  }
  else if(output > 1){
    output = 1;
  }
  return output;
}

double PID::Twiddle(double param, double err){
  // todo: get rid of param, use Kp attribute directly
  // Let's run it only on Kp first
  // std::cout<<"Current Kp_twiddle = "<<Kp_twiddle<<std::endl;
  // std::cout<<"Current dp = "<<dp<<std::endl;
  if(dp > 0.0005){
    param += dp;  //bump up
    // std::cout<<"Trying bumping up param to "<<param<<std::endl;
    if(abs(err) < best_err){
      // std::cout<<"Bumping up was helpful: Error is "<< err << " which is lower than " << best_err <<std::endl;
      best_err = err;  // bumping up helped improving the error
      dp *= delta_plus;
    }
    else{
      param -= 2*dp;  // bumping up did not help improve the error -> thus going down
      // std::cout<<"Trying bumping down param to "<<param<<std::endl;
      if(abs(err) < best_err){
        // std::cout<<"Bumping down was helpful: Error is "<< err << " which is lower than " << best_err <<std::endl;
        best_err = err;
        dp *= delta_plus;
      }
      else{
        // std::cout<<"Both were not helpful: Decreasing interval"<<std::endl;
        param += dp;
        // std::cout<<"Setting new param to "<<param<<std::endl;
        dp *= delta_minus;
      }
    }
  }
  return param;
}
// double PID::Twiddle(double param, double err){
//   // todo: get rid of param, use Kp attribute directly
//   // Let's run it only on Kp first

//   if(dp > 0.005){
//     param += dp;  //bump up
//     if(abs(err) < best_err){
//       best_err = err;  // bumping up helped improving the error
//       dp *= delta_plus;
//     }
//     else{
//       param -= 2*dp;  // bumping up did not help improve the error -> thus going down
//       if(abs(err) < best_err){
//         best_err = err;
//         dp *= delta_plus;
//       }
//       else{
//         param += dp;
//         dp *= delta_minus;
//       }
//     }
//     std::cout<<"Param has changed = "<<param<<std::endl;
//   }
//   return param;
// }