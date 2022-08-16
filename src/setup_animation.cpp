#include "setup_animation.h"
#include "irobot_config.h"

//#define SHOW_ANIMATION_ENABLE

//width = 480
//columns = 48
//height = 240
//lines = 12
//center = (240, 120)
void show_animation(void) {
#ifdef SHOW_ANIMATION_ENABLE
  Brain.Screen.setPenColor(blue);
  Brain.Screen.drawLine(180,  60, 300,  60);
  Brain.Screen.drawLine(180, 180, 300, 180);

  Brain.Screen.setPenColor(red);
  Brain.Screen.drawLine(200,  80, 280,  80);
  Brain.Screen.drawLine(200, 160, 280, 160);

  Brain.Screen.drawLine(220, 120, 260, 120);

  Brain.Screen.drawLine(280, 80, 200, 160);
  wait(2, sec);
  Brain.Screen.clearScreen();
#endif
}