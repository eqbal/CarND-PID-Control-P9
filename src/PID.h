#ifndef PID_H
#define PID_H

#include <iostream>
#include <cmath>

#define DEBUG 0

class PID {
 public:

  // Coefficients
  double Kp;
  double Ki;
  double Kd;

  PID();

  virtual ~PID();
  void Init(double target, double Kp, double Ki, double Kd);
  void Update(double error, double x);

  // Getters
  double GetTotalError() const;
  double GetAveragedError() const;
  double GetCorrection() const;

private:
  double correction = 0;
  double last_x_ = -1;

  double target_ = 0;
  double old_error_ = 0;
  double error_sum = 0;
  double error_sum_abs = 0;
  double error_int = 0;

  //Overflows not handled
  size_t counter = 0;

  void calculate(double error, double dt);
};

#endif /* PID_H */
