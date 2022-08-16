#include "actions.h"
#include "debug.h"

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

double PID::calc(double err) {
  INIT_DEBUG();

  errsum += err * WAIT_TIME;
  double d_err = (err - last_err) / WAIT_TIME;
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

//----------------------------------------

RotateAction::RotateAction(turnType dir, double angles, double kp, double ki, double kd, double max_errsum)
  : dir(dir), angles(angles), pid(kp, ki, kd, max_errsum) {
}

#define ROTATE_QUALITY (0.5)

//左转角度减
//右转角度加
bool RotateAction::act(void) {
  INIT_DEBUG();

  if (dir == left) {
    angles = -angles;
  }

  double origin = Drivetrain.rotation(degrees);
  double target = origin + angles;

  double current = Drivetrain.rotation(degrees);
  double error = target - current;

  double raw_factor = pid.calc(error);
  double factor = abs_within(0, 1, raw_factor); //将factor控制到-1~1之间
  double velocity = factor * VELOCITY_PERCENT;

  Drivetrain.turn(right, velocity, percent);

  while (fabs(error) > ROTATE_QUALITY && (! force_exit)) {
    DPRINTF (ROTATE_ACTION_LINE1, ROTATE_ACTION_COLUMN, "error: %.4lf ; velocity: %.4lf", error, velocity);
    DPRINTF (ROTATE_ACTION_LINE2, ROTATE_ACTION_COLUMN, "current: %.4lf ; target: %.4lf", current, target);

    Drivetrain.turn(right, velocity, percent);

    wait(WAIT_TIME, msec);
  
    current = Drivetrain.rotation(degrees);
    error = target - current;

    raw_factor = pid.calc(error);
    factor = abs_within(0, 1, raw_factor); //将factor控制到-1~1之间
    velocity = factor * VELOCITY_PERCENT;
  }

  Drivetrain.stop(hold);

  DPRINTF (ROTATE_ACTION_LINE1, ROTATE_ACTION_COLUMN, "error: %.4lf ; velocity: %.4lf", error, velocity);
  DPRINTF (ROTATE_ACTION_LINE2, ROTATE_ACTION_COLUMN, "current: %.4lf ; target: %.4lf", current, target);

  return true;
}

PID *RotateAction::getPID(void) {
  return &this->pid;
}

//----------------------------------------

DriveAction::DriveAction(directionType dir, double distance, double kp, double ki, double kd, double max_errsum) 
  : dir(dir), distance(distance), pid(kp, ki, kd, max_errsum) {
}

bool DriveAction::act(void) {
  if (dir == reverse) {
    distance = -distance;
  }

  // Drivetrain.setDriveVelocity(VELOCITY_PERCENT, percent);
  // Drivetrain.driveFor(forward, distance, inches, true);

  Drivetrain.stop(hold);
  return  true;
}

PID *DriveAction::getPID(void) {
  return &this->pid;
}

//----------------------------------------

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