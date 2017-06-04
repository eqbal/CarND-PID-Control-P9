#include "PID.h"

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->target_ = target;
  this->counter = 0;
}

void PID::UpdateError(double cte) {

  if(this->last_x_ < 0){
    this->last_x_ = x;
    this->old_error_ = error;
    return;
  }

  double dx = x - this->last_x_;
  calculate(error, dx);

  this->old_error_ = error;
  this->last_x_ = x;
}

void PID::calculate(double error, double dt) {

  double p, i, d, diff_t;

  this->error_sum += error;
  this->error_sum_abs += std::fabs(error);
  this->error_int += 0.5 * dt * (error+this->old_error_);

  this->counter++;

  //Calculate P part
  p = -this->Kp * error;

  //Calculate I part
  i = this->Ki * this->error_int;

  //Calculate D part
  diff_t = error - this->old_error_;
  d = this->Kd* (diff_t)*dt;

  //Calculate correction
  correction = this->target_ + (p-i-d);

  if (DEBUG) {
    std::cout << "Pid(" << error << "," << dt << ") = "
              << correction << " = " << this->target_  << " + "
              << -Kp << "*" << error << " - "
              << Ki <<  "*" << this->error_int << " - "
              << Kd << "*" << diff_t << std::endl;
  }
}

double PID::GetTotalError() const{
  return this->error_sum;
}

double PID::GetAveragedError() const {
  return this->error_sum_abs / counter;
}

double PID::GetCorrection() const {
  return this->correction;
}
