#ifndef __WAISTHOT_H__
#define __WAISTHOT_H__

#define WAIST_HEAT_PORT   gpioPortB
#define WAIST_HEAT_BIT    2
#define WAIST_HEAT_MODE   gpioModePushPull

void WaistHeat_Initial_IO(void);
void WaistHeat_On(void);
void WaistHeat_Off(void);

#endif
