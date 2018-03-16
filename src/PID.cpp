#include "PID.h"
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/
enum {
  IDLE,
  INC,
  DEC,
  DONE
};

PID::PID() {
  /*
  * Coefficients
  */
  Kp_ = 0;
  Ki_ = 0;
  Kd_ = 0;

  /*
  * Errors
  */
  p_error = 0;;
  i_error = 0;
  d_error = 0;

  n_step = 0;
  best_error = 0;

  twiddle_state = IDLE;
}

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
  sum_error = 0;

  dp[0] = 0.01;
  dp[1] = 0.01;
  dp[2] = 0.01;
  p_idx = 0;

  twiddle_state = IDLE;
}

void PID::UpdateError(double cte) {

  // p_error holds the previous cte
  d_error  = cte - p_error;
  p_error  = cte;
  i_error += cte;

  // reset the I value if CTE is 0
  if(cte == 0)  i_error = 0;

  // limit max I-term to prevent wind-up
  if(i_error > 1)  i_error = 1;
  else if(i_error < -1)  i_error = -1;

  if(twiddle_state != DONE) {
    // ignore the fist 100 steps
    if (++n_step > 100)
       sum_error += cte * cte;
  }
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

bool PID::Twiddle(double tolerance) {

  bool reset = false;

  if(n_step > 300) {

    double  p[3];
    double  error;

    // get
    p[0] = Kp_;
    p[1] = Ki_;
    p[2] = Kd_;

    switch (twiddle_state)  {
      case  DONE:
        break;

      case  IDLE:
        best_error = sum_error/300;
        if(best_error > tolerance)  {
          p[p_idx] += dp[p_idx];
          twiddle_state = INC;
        }
        else {
          twiddle_state = DONE;
        }
        reset = true;
        break;

      case  INC:
        error = sum_error/300;
        if(error < best_error)  {
          best_error = error;
          if(best_error > tolerance)  {
            dp[p_idx] *= 1.1;
            if(p_idx < 2) {
              p_idx++;
            }
            else  {
              p_idx = 0; // start over
            }
            p[p_idx] += dp[p_idx];
          }
          else  {
            twiddle_state = DONE;
          }
        }
        else  {
          p[p_idx] -= 2*dp[p_idx];  // get back
          twiddle_state = DEC;
        }
        reset = true;
        break;

      case  DEC:
        error = sum_error/300;
        if(error < best_error)  {
          best_error = error;
          if(best_error > tolerance)  {
            dp[p_idx] *= 1.1;
          }
          else
            twiddle_state = DONE;
        }
        else  {
          p[p_idx] += dp[p_idx]; // get back
          dp[p_idx] *= 0.9;
        }

        if(p_idx < 2) {
          p_idx++;  // do next param
        }
        else  {
          p_idx = 0; // start over
        }

        p[p_idx] += dp[p_idx];
        twiddle_state = INC;

        reset = true;
        break;

      default:
        twiddle_state = DONE;
        break;
    }

    // get error for the next 100 steps
    n_step = 0;
    sum_error = 0;

    // set
    Kp_ = p[0];
    Ki_ = p[1];
    Kd_ = p[2];

    if (reset == true)  {
      p_error = 0;;
      i_error = 0;
      d_error = 0;
    }

    return reset;
  }

  return reset;
}
