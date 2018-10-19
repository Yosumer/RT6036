#ifndef __SYSTEM_H__
#define __SYSTEM_H__

//extern bool  bTimer2MS;
extern bool bVibrateEnable;
extern unsigned int Vibrate_Pwm_Speed;
void System_Initial_IO(void);
void System_DelayXms(unsigned int ulData);
#endif
