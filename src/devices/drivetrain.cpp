#include <devices/drivetrain.h>

namespace robot {
  Drivetrain::Drivetrain(motor &MLF, motor &MLB, motor &MRF, motor &MRB, inertial &Inertial,
     double wheel_diameter, double gear_ratio, double max_rpm)
    : MLF(MLF), MLB(MLB), MRF(MRF), MRB(MRB), Inertial(Inertial),
      wheel_circ(wheel_diameter * 3.14159), gear_ratio(gear_ratio), max_rpm(max_rpm)
  {
  }

  //行驶
  void Drivetrain::drive(directionType dir, double velocity, percentUnits unit) {
    if (dir == reverse) {
      velocity = -velocity;
    }

    MLF.setVelocity(velocity, unit);
    MLB.setVelocity(velocity, unit);
    MRF.setVelocity(velocity, unit);
    MRB.setVelocity(velocity, unit);

    MLF.spin(forward);
    MLB.spin(forward);
    MRF.spin(forward);
    MRB.spin(forward);
  }

  //停止
  void Drivetrain::stop(brakeType mode) {
    MLF.stop(mode);
    MLB.stop(mode);
    MRF.stop(mode);
    MRB.stop(mode);
  }
  
  //转动
  void Drivetrain::spin(directionType dir, double lvelocity, double rvelocity, percentUnits unit) {
    if (dir == reverse) {
      lvelocity = -lvelocity;
      rvelocity = -rvelocity;
    }

    MLF.setVelocity(lvelocity, unit);
    MLB.setVelocity(lvelocity, unit);
    MRF.setVelocity(rvelocity, unit);
    MRB.setVelocity(rvelocity, unit);

    MLF.spin(forward);
    MLB.spin(forward);
    MRF.spin(forward);
    MRB.spin(forward);
  }

  //旋转
  void Drivetrain::turn(turnType dir, double velocity, percentUnits unit) {
    if (dir == right) {
      velocity = -velocity;
    }
    spin(forward, velocity, -velocity, unit);
  }
  
  //获得旋转角
  double Drivetrain::rotation(rotationUnits units) {
    double angles = Inertial.rotation(units);
    if (units == degrees) {
      while (angles >= 360) angles -= 360;
      while (angles <= -360) angles += 360;

      if (angles < 0)
        angles += 360;
    }
    return angles;
  }
  
  double Drivetrain::distance(double percent, double time, timeUnits unit) {
    if (unit == msec) {
      time /= 1000;
    }
    double motor_rps = percent * max_rpm / 6000;
    double wheel_rps = motor_rps * gear_ratio;
    return wheel_rps * time * wheel_circ;
  }

  //校准
  void Drivetrain::calibrate(void) {
    Inertial.calibrate();
  }

  //温度
  double Drivetrain::temperature(void) {
    double sum = 0;
    sum += MLF.temperature(celsius);
    sum += MLB.temperature(celsius);
    sum += MRF.temperature(celsius);
    sum += MRB.temperature(celsius);
    return sum / 4;
  }
}