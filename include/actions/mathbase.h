#pragma once

#include <cmath>

//PID类
class PID {
protected:
  double errsum;
  double last_err;
  double max_errsum;
  double kp;
  double ki;
  double kd;
public:
  PID(double kp, double ki, double kd, double max_errsum);
  double get_kp(void);
  double get_ki(void);
  double get_kd(void);
  void set_kp(double kp);
  void set_ki(double ki);
  void set_kd(double kd);
  double calc(double err);
};

//within
double abs_within(double min, double max, double val);
double within(double min, double max, double val);