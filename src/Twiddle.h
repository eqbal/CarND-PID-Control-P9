#ifndef SRC_TWIDDLE_H_
#define SRC_TWIDDLE_H_

class Twiddle{

  constexpr static size_t PARAMS = 3;
  double best_errors[PARAMS] = {-1,-1,-1};
  double errors[PARAMS];
  double p_[PARAMS] = {0,0,0};
  double dp_[PARAMS] = {0,0,0};
  bool rerun_[PARAMS] = {false, false, false};
  size_t idx_ = 0;
  PID& pid;
  size_t counter = 0;
  const size_t window = 500;

  public:

  Twiddle(PID &pid) : pid(pid){
    p_[0] = pid.Kp;
    p_[1] = pid.Ki;
    p_[2] = pid.Kd;

    dp_[0] = p_[0]*0.5;
    dp_[1] = p_[1]*0.5;
    dp_[2] = p_[2]*0.5;
  }

  void update(double cte)
  {
    double error = cte*cte;
    counter++;

    for(size_t i = 0;i < PARAMS; ++i)
      errors[idx_] += error;

    if((counter % (window)) != 0)
      return;

    counter = 0;

    if(best_errors[idx_] < 0)
    {
      //Update pid parameters
      p_[idx_] += dp_[idx_];
      updateParameters();

      best_errors[idx_] = errors[idx_];
      resetError();
      return;
    }

    if(errors[idx_] < best_errors[idx_] && !rerun_[idx_])
    {
      best_errors[idx_] = errors[idx_] ;
      resetError();
      dp_[idx_] *= 1.1;
    }
    else
    {
      if(!rerun_[idx_])
      {
        p_[idx_] = p_[idx_] - 2*dp_[idx_];
        rerun_[idx_] = true;
        updateParameters();
        resetError();
        return;
      }
      else
      {
        //Evaluate Rerun
        if(errors[idx_] < best_errors[idx_])
        {
          best_errors[idx_] = errors[idx_] ;
          dp_[idx_] *= 1.1;
        }
        else
        {
          p_[idx_] += dp_[idx_];
          dp_[idx_] *= 0.9;
        }

        rerun_[idx_] = false;
      }

      resetError();
    }

    idx_ = (idx_ + 1) % PARAMS;

    //Update next pid parameters
    p_[idx_] += dp_[idx_];
    updateParameters();
  }

  void updateParameters()
  {
    pid.Kp = p_[0];
    pid.Ki = p_[1];
    pid.Kd = p_[2];
  }

  void resetError()
  {
    for(size_t i = 0; i < PARAMS; ++i)
      errors[i] = 0;
  }
};

#endif /* SRC_TWIDDLE_H_ */
