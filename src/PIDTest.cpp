#include "irobot_config.h"
#include "actions.h"
#include "debug.h"

//-----------------

#define ACTION_NUM (10)


//                                       kp       ki      kd    max error
Action *ALL_ACTION[ACTION_NUM] = {
  new RotateAction(right  , 45.00000, 0.12200, 0.01010, 0.01000, 45.00000),
  new DriveAction (forward, 02.82843, 0.00000, 0.00010, 0.00000,  2.82843),
  new RotateAction(left   , 78.69007, 0.00000, 0.00010, 0.00000, 78.69007),
  new RotateAction(right  , 78.69007, 0.00000, 0.00010, 0.00000, 78.69007),
  new DriveAction (forward, 00.70711, 0.00000, 0.00010, 0.00000,  0.70711),
  new RotateAction(left   , 90.00000, 0.00000, 0.00010, 0.00000, 90.00000),
  new RotateAction(right  , 90.00000, 0.00000, 0.00010, 0.00000, 90.00000),
  new DriveAction (forward, 02.12132, 0.00000, 0.00010, 0.00000,  2.12132),
  new RotateAction(left   , 23.19859, 0.00000, 0.00010, 0.00000, 23.19859),
  new RotateAction(right  , 23.19859, 0.00000, 0.00010, 0.00000, 23.19859)
};

int cur_action = 0;

#define DELTA_NUM (10)

double deltas[DELTA_NUM] = {
  0.0001, 0.0005,
  0.0010, 0.0050,
  0.0100, 0.0500,
  0.1000, 0.5000,
  1.0000, 5.0000
};
int cur_delta = 10;

double delta = 0.100000;
bool running = false;

namespace Modes {
  constexpr int START = 0;
  constexpr int ACTION = 0;
  constexpr int DELTA = 1;
  constexpr int KP = 2;
  constexpr int KI = 3;
  constexpr int KD = 4;
  constexpr int LAST = 4;
};

int mode = Modes::ACTION;

//-----------------

void printPIDArgs(void) {
  INIT_DEBUG();

  PID *pid = ALL_ACTION[cur_action]->getPID();

  CPRINTF(KP_KI_LINE, KP_COLUMN, "kp:%.4lf", pid->get_kp());
  CPRINTF(KP_KI_LINE, KI_COLUMN, "ki:%.4lf", pid->get_ki());
  CPRINTF(KD_DELTA_LINE, KD_COLUMN, "kd:%.4lf", pid->get_kd());
  CPRINTF(KD_DELTA_LINE, DELTA_COLUMN, "dt:%.4lf", delta);
}

void printCurAction(void) {
  INIT_DEBUG();

  CPRINTF(MODE_RUNNING_ACTION_LINE, ACTION_COLUMN, "%4d", cur_action);
}

void printCurMode(void) {
  INIT_DEBUG();

  const char *modename;
  switch (mode) {
    case Modes::ACTION: modename = "ACTION"; break;
    case Modes::DELTA: modename  = " DELTA"; break;
    case Modes::KP: modename     = "  KP  "; break;
    case Modes::KD: modename     = "  KD  "; break;
    case Modes::KI: modename     = "  KI  "; break;
    default: modename            = "??????"; break;
  }

  CPRINTF(MODE_RUNNING_ACTION_LINE, MODE_COLUMN, "%s", modename);
}

void printState(void) {
  INIT_DEBUG();
  CPRINTF(MODE_RUNNING_ACTION_LINE, RUNNING_COLUMN, "%s", running ? "RUNNING" : "  DONE ");
}

void printScreen() {
  Controller1.Screen.clearScreen();
  printPIDArgs();
  printCurAction();
  printCurMode();
  printState();
}

//==========================================================

void upAction(void) {
  cur_action += 1;
  if (cur_action >= ACTION_NUM) {
    cur_action = 0;
  }

  printScreen();
}

void downAction(void) {
  cur_action -= 1;
  if (cur_action < 0) {
    cur_action = ACTION_NUM - 1;
  }
  
  printScreen();
}

//-----------------------

void upKp(void) {
  PID *pid = ALL_ACTION[cur_action]->getPID();
  pid->set_kp(pid->get_kp() + delta);

  printScreen();
}

void downKp(void) {
  PID *pid = ALL_ACTION[cur_action]->getPID();
  pid->set_kp(pid->get_kp() - delta);

  printScreen();
}

void upKi(void) {
  PID *pid = ALL_ACTION[cur_action]->getPID();
  pid->set_ki(pid->get_ki() + delta);

  printScreen();
}

void downKi(void) {
  PID *pid = ALL_ACTION[cur_action]->getPID();
  pid->set_ki(pid->get_ki() - delta);

  printScreen();
}

void upKd(void) {
  PID *pid = ALL_ACTION[cur_action]->getPID();
  pid->set_kd(pid->get_kd() + delta);

  printScreen();
}

void downKd(void) {
  PID *pid = ALL_ACTION[cur_action]->getPID();
  pid->set_kd(pid->get_kd() - delta);

  printScreen();
}

void upDelta(void) {
  cur_delta ++;
  if (cur_delta >= DELTA_NUM) {
    cur_delta = 0;
  }
  delta = deltas[cur_delta];

  printScreen();
}

void downDelta(void) {
  cur_delta --;
  if (cur_delta < 0) {
    cur_delta = DELTA_NUM - 1;
  }
  delta = deltas[cur_delta];

  printScreen();
}

//------------------------------------------------------------

void doUp(void) {
  if (mode == Modes::ACTION) {
    upAction();
  }
  else if (mode == Modes::KP) {
    upKp();
  }
  else if (mode == Modes::KI) {
    upKi();
  }
  else if (mode == Modes::KD) {
    upKd();
  }
  else if (mode == Modes::DELTA) {
    upDelta();
  }
}

void doDown(void) {
  if (mode == Modes::ACTION) {
    downAction();
  }
  else if (mode == Modes::KP) {
    downKp();
  }
  else if (mode == Modes::KI) {
    downKi();
  }
  else if (mode == Modes::KD) {
    downKd();
  }
  else if (mode == Modes::DELTA) {
    downDelta();
  }
}

//------------------------------------------------------------

void upMode(void) {
  mode ++;
  if (mode > Modes::LAST) {
    mode = Modes::START;
  }
  printScreen();
}

void downMode(void) {
  mode --;
  if (mode < Modes::START) {
    mode = Modes::LAST;
  }
  printScreen();
}

//==========================================================

void act(void) {
  running = true;
  printScreen();
  ALL_ACTION[cur_action]->act();
  running = false;
  printScreen();
  force_exit = false;
}

void force_stop(void) {
  Drivetrain.stop(hold);
  force_exit = true;
}

void register_controllers(void) {
  Controller1.ButtonA.pressed(act);
  Controller1.ButtonB.pressed(force_stop);
  Controller1.ButtonUp.pressed(doUp);
  Controller1.ButtonDown.pressed(doDown);
  Controller1.ButtonRight.pressed(upMode);
  Controller1.ButtonLeft.pressed(downMode);

  printScreen();
}