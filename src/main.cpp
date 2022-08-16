/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\admin                                            */
/*    Created:      Sun Aug 14 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// MotorLF              motor         1               
// MotorLB              motor         11              
// MotorRF              motor         10              
// MotorRB              motor         20              
// Inertial             inertial      13              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "irobot_config.h"
#include "actions.h"
#include "setup_animation.h"
#include "debug.h"
#include "PIDTest.h"

using namespace vex;

bool force_exit = false;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  Drivetrain.calibrate();
  Controller1.Screen.clearScreen();

  show_animation();

  register_controllers();
}
