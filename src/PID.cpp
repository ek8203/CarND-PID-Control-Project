#include "PID.h"
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

  /*
  * Coefficients
  */
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  /*
  * Errors
  */
  p_error = 0;;
  i_error = 0;
  d_error = 0;

  n_step = 0;
  best_error = 0;
}

void PID::UpdateError(double cte) {

  // p_error holds the previous cte
  d_error  = cte - p_error;
  p_error  = cte;
  i_error += cte;

  // reset the I value if CTE is 0
  if(cte == 0)
    i_error = 0;

  // limit max I-term to prevent wind-up
  if(i_error > 1)  i_error = 1;
  else if(i_error < -1)  i_error = -1;

  if (n_step >= 100)  best_error += (cte * cte) / n_step;

  n_step++;

}

double PID::TotalError() {

  return Kp_*p_error + Kd_*d_error + Ki_*i_error;
}

double PID::GetOutput() {

  double output = -TotalError();

  // check the limits [-1,1]
  if(output > 1.0)
    output = 1.0;
  else if(output < -1.0)
    output = -1.0;

  return output;
}
