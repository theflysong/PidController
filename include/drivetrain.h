#pragma once

#include "vex.h"

namespace robot {
  class Drivetrain {
    motor &MLF;
    motor &MLB;
    motor &MRF;
    motor &MRB;
    inertial &Inertial;
    double wheel_circ;
    double gear_ratio;
  public:
    Drivetrain(motor &MLF, motor &MLB, motor &MRF, motor &MRB, inertial &Inertial,
     double wheel_diameter, double gear_ratio);
    void drive(directionType dir, double velocity, percentUnits unit);
    void stop(brakeType mode);
    void spin(directionType dir, double lvelocity, double rvelocity, percentUnits unit);
    void turn(turnType dir, double velocity, percentUnits unit);
    double rotation(rotationUnits units);
    void calibrate(void);
    double temperature(void);
  };
}