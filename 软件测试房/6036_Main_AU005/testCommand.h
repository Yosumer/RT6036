#ifndef __TEST_COMAND_H__
#define __TEST_COMAND_H__
#include "ControlBox.h"
#define LINGO_NONE        H10_KEY_NONE
#define LINGO_AIRBAG      0x01
#define LINGO_RESET       0x02
#define LINGO_ENG         0x03
#define LINGO_MENU        0x04
#define LINGO_PROGRAM     0x05
#define LINGO_INPUT       0x06
#define LINGO_ROLLER_TEST 0x07
#define LINGO_KNEAD_TEST  0x08

#define LINGO_SLIDE_TEST  0x09
#define LINGO_BACK_TEST   0x0A
#define LINGO_LEG_TEST    0x0B
#define LINGO_FLEX_TEST   0x0C
#define LINGO_WALK_TEST   0x0D
#define LINGO_KNOCK_TEST  0x0E
#define LINGO_LED_TEST    0x0F


#define LINGO_PROGRAM_BY_BLUETOOTH    0x10
#define LINGO_BLUETOOTH_BR115200      0x11
#define LINGO_MUSIC_TEST  0x12
#define LINGO_HEAT_TEST   0x13
#define LINGO_ONE_KEY_TEST    0x14
#define LINGO_TOTAL_TIME      0x15


#define SYS_KEY_RELEASE   0x40
#define SYS_KEY_UP        1
#define SYS_KEY_DOWN      2
#define SYS_KEY_LEFT      3
#define SYS_KEY_RIGHT     4
#define SYS_KEY_ENTER     5
#define SYS_KEY_UP_RELEASE        (SYS_KEY_UP | SYS_KEY_RELEASE)
#define SYS_KEY_DOWN_RELEASE      (SYS_KEY_DOWN | SYS_KEY_RELEASE)
#define SYS_KEY_LEFT_RELEASE      (SYS_KEY_LEFT | SYS_KEY_RELEASE)
#define SYS_KEY_RIGHT_RELEASE     (SYS_KEY_RIGHT | SYS_KEY_RELEASE)
#define SYS_KEY_ENTER_RELEASE     (SYS_KEY_ENTER | SYS_KEY_RELEASE)

/*
LINGO_AIRBAG 的key为气囊和气泵
气囊为1-24 气泵为1 2 3
  气泵1  气泵2 气泵3
1 9   17   
2 10  18
3 11  19
4 12  20
5 13  21
6 14  22
7 15  23
8 16  24
*/


#endif