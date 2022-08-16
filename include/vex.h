/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <v5.h>
#include <v5_vcs.h>

#include <robot-config.h>

//速率
#define VELOCITY_PERCENT (100)

//等待时间
#define WAIT_TIME (20)
#define WAIT_TIME_S (0.02)

#define TEST_PID_ENABLE
// #define AUTONOMOUS_ENABLsE
// #define MANUAL_ENABLE

//强制退出标志
extern bool force_exit;

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)