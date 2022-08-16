#pragma once

#include <devices/irobot_config.h>
#include <actions/mathbase.h>

#pragma region actions

//行为基类
class Action {
public:
  virtual bool act(void) = 0;
  virtual PID *getPID(void) = 0;
};

#pragma endregion

//旋转行为
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

//移动行为
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

#pragma endregion