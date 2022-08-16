#include <actions/mathbase.h>
#include <display/debug.h>

#pragma region PID

PID::PID(double kp, double ki, double kd, double max_errsum) 
  : errsum(0), last_err(0), max_errsum(max_errsum),
    kp(kp), ki(ki), kd(kd) {
}

double PID::get_kp(void) {
  return this->kp;
}

double PID::get_ki(void) {
  return this->ki;
}

double PID::get_kd(void) {
  return this->kd;
}

void PID::set_kp(double kp) {
  this->kp = kp;
}

void PID::set_ki(double ki) {
  this->ki = ki;
}

void PID::set_kd(double kd) {
  this->kd = kd;
}

#define ERR_MIN (0.01)

double PID::calc(double err, double dt) {
  INIT_DEBUG();

  if (fabs(err) <= (max_errsum / 10)) {
    errsum += err * dt;
  }
  double d_err = (err - last_err) / dt;
  last_err = err;

  if (fabs(err) < ERR_MIN) {
    errsum = 0;
  }

  if (fabs(errsum) > max_errsum) {
    errsum = 0;
  }

  DPRINTF (PID_LINE, PID_COLUMN, "errsum: %.4lf ; d_err: %.4lf", errsum, d_err);

  double proportion = kp * err;
  double integral = ki * errsum;
  double differential = kd * d_err;

  return proportion + integral + differential;
}

#pragma endregion

double abs_within(double min, double max, double val) {
  double abs_val = fabs(val);

  if (abs_val < min) {
    if (val > 0) return min;
    else return -min;
  }
  if (abs_val > max) {
    if (val > 0) return max;
    else return -max;
  }

  return val;
}

double within(double min, double max, double val) {
  if (min > val) {
    return min;
  }
  if (max < val) {
    return max;
  }
  return val;
}