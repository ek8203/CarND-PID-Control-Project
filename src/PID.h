#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;

  // error update step number
  int n_step;

  // square average error
  double best_error;
  double sum_error;

  // twiddle params
  int twiddle_state;
  double  dp[3];
  int p_idx;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
   * Calculate PID output
   */
  double GetOutput();

  /*
   * Parameter Optimization
   */
  bool Twiddle(double tolerance);
};

#endif /* PID_H */
