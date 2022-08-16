#pragma once

#include "vex.h"
#include <cstdio>

#define INIT_DEBUG() char buffer[256]

#define DPRINTF(lines, columns, FMT, ...) sprintf(buffer, FMT, __VA_ARGS__); \
                                          Brain.Screen.setCursor(lines + 1, columns + 1); \
                                          Brain.Screen.print(buffer) 

#define CPRINTF(lines, columns, FMT, ...) sprintf(buffer, FMT, __VA_ARGS__); \
                                          Controller1.Screen.setCursor(lines + 1, columns + 1); \
                                          Controller1.Screen.print(buffer)

//Controller
#define MODE_RUNNING_ACTION_LINE (0)
#define KP_KI_LINE     (1)
#define KD_DELTA_LINE (2)

#define MODE_COLUMN (0)
#define RUNNING_COLUMN (7)
#define ACTION_COLUMN (14)

#define KP_COLUMN (0)
#define KI_COLUMN (10)
#define KD_COLUMN (0)
#define DELTA_COLUMN (10)

//Brain
#define ROTATE_ACTION_LINE1 (0)
#define ROTATE_ACTION_LINE2 (1)
#define ROTATE_ACTION_COLUMN (0)
#define PID_LINE (2)
#define PID_COLUMN (0)