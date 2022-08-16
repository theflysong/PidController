#pragma once

#include <vex.h>
#include <cstdio>

#define INIT_DEBUG() char buffer[256]

//打印到主控
#define DPRINTF(lines, columns, FMT, ...) sprintf(buffer, FMT, __VA_ARGS__); \
                                          Brain.Screen.setCursor(lines + 1, columns + 1); \
                                          Brain.Screen.print(buffer) 

#ifdef TEST_PID_ENABLE

//打印到手柄
#define CPRINTF(lines, columns, FMT, ...) sprintf(buffer, FMT, __VA_ARGS__); \
                                          Controller1.Screen.setCursor(lines + 1, columns + 1); \
                                          Controller1.Screen.print(buffer)

#else

#define CPRINTF(lines, columns, FMT, ...)

#endif

#pragma region Controller

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

#pragma endregion
#pragma region brain

#define ROTATE_ACTION_LINE1 (0)
#define ROTATE_ACTION_LINE2 (1)
#define ROTATE_ACTION_COLUMN (0)

#define DRIVE_ACTION_LINE1 (2)
#define DRIVE_ACTION_LINE2 (3)
#define DRIVE_ACTION_COLUMN (0)

#define PID_LINE (10)
#define PID_COLUMN (0)

#pragma endregion