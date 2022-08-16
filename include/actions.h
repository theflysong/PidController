#pragma once

#include "irobot_config.h"

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

class Action {
public:
  virtual bool act(void) = 0;
  virtual PID *getPID(void) = 0;
};

class RotateAction : public Action {
protected:
  turnType dir;
  double angles;
  PID pid;
public:
  RotateAction(turnType dir, double angles, double kp, double ki, double kd, double max_errsum);
  virtual bool act(void) override final;
  virtual PID *getPID(void) override final;
};

class DriveAction : public Action {
protected:
  directionType dir;
  double distance;
  PID pid;
public:
  DriveAction(directionType dir, double distance, double kp, double ki, double kd, double max_errsum);
  virtual bool act(void) override final;
  virtual PID *getPID(void) override final;
};

double abs_within(double min, double max, double val);
double within(double min, double max, double val);