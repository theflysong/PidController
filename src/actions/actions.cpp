#include <actions/actions.h>
#include <display/debug.h>

#pragma region actions

#pragma region rotate

//旋转行为
RotateAction::RotateAction(turnType dir, double angles, double kp, double ki, double kd, double max_errsum)
  : dir(dir), angles(angles), pid(kp, ki, kd, max_errsum) {
}

//修正角度
double fix_angle(double target, double angle) {
  double angle2 = angle + 360;
  double angle3 = angle - 360;
  double err1 = fabs(target - angle);
  double err2 = fabs(target - angle2);
  double err3 = fabs(target - angle3);

  if (err1 > err2) {
    if (err2 > err3) {
      return angle3;
    }
    return angle2;
  }
  else {
    if (err1 > err3) {
      return angle3;
    }
    return angle;
  }
}

#define ROTATE_QUALITY (0.5)
#define MIN_SPEED (0.1)
#define MAX_SPEED (1)

//左转角度减
//右转角度加
bool RotateAction::act(void) {
  INIT_DEBUG();

  if (dir == left) {
    angles = -angles;
  }

  double origin = Drivetrain.rotation(degrees); //起始角度
  double target = origin + angles; //目标角度

  double current = fix_angle(target, Drivetrain.rotation(degrees)); //当前角度
  double error = target - current; //误差
  double raw_factor = pid.calc(error); //因子
  double factor = abs_within(0, 1, raw_factor); //将factor控制到-1~1之间
  double velocity = factor * VELOCITY_PERCENT;

  Drivetrain.turn(right, velocity, percent);

  //直到达到精度或者强制退出时才结束
  while (fabs(error) > ROTATE_QUALITY && (! force_exit)) {  
    current = fix_angle(target, Drivetrain.rotation(degrees)); //当前角度
    error = target - current; //误差

    raw_factor = pid.calc(error); 
    factor = abs_within(MIN_SPEED, MAX_SPEED, raw_factor); //将factor控制到-1~1之间
    velocity = factor * VELOCITY_PERCENT; //速度

    DPRINTF (ROTATE_ACTION_LINE1, ROTATE_ACTION_COLUMN, "error: %.4lf ; velocity: %.4lf", error, velocity);
    DPRINTF (ROTATE_ACTION_LINE2, ROTATE_ACTION_COLUMN, "current: %.4lf ; target: %.4lf", current, target);

    Drivetrain.turn(right, velocity, percent); //运行
  }

  Drivetrain.stop(brake); //刹车
  return true;
}

PID *RotateAction::getPID(void) {
  return &this->pid;
}

#pragma endregion
#pragma region drive

DriveAction::DriveAction(directionType dir, double distance, double kp, double ki, double kd, double max_errsum) 
  : dir(dir), distance(distance), pid(kp, ki, kd, max_errsum) {
}

bool DriveAction::act(void) {
  INIT_DEBUG();

  if (dir == reverse) {
    distance = -distance;
  }

  double target = distance;

  double current = 0;
  double error = target - current;
  double raw_factor = pid.calc(error);
  double factor = abs_within(MIN_SPEED, MAX_SPEED, raw_factor);
  double velocity = factor * VELOCITY_PERCENT;

  Drivetrain.drive(forward, velocity, percent);

  while (fabs(error) > 0.5 && (! force_exit)) {
    wait(1, msec);
    current += Drivetrain.distance(velocity, 1, msec);
    
    error = target - current;
    raw_factor = pid.calc(error);
    factor = abs_within(MIN_SPEED, MAX_SPEED, raw_factor);
    velocity = factor * VELOCITY_PERCENT;

    Drivetrain.drive(forward, velocity, percent);

    DPRINTF (DRIVE_ACTION_LINE1, DRIVE_ACTION_COLUMN, "error: %.4lf ; velocity: %.4lf", error, velocity);
    DPRINTF (DRIVE_ACTION_LINE2, DRIVE_ACTION_COLUMN, "current: %.4lf ; target: %.4lf", current, target);
  }

  Drivetrain.stop(brake); //刹车
  return  true;
}

PID *DriveAction::getPID(void) {
  return &this->pid;
}

#pragma endregion

#pragma endregion