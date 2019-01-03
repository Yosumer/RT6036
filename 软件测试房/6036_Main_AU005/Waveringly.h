#ifndef __WAVERINGLY_H__
#define __WAVERINGLY_H__

#define WAVE_MOTOR_PORT            gpioPortA
#define WAVE_MOTOR_BIT             0
#define WAVE_MOTOR_PORT_MODE       gpioModePushPull

#define WAVE_MOTOR_TIMER           TIMER0
#define WAVE_MOTOR_TIMER_CHANNEL   0
#define WAVE_MOTOR_ROUTE_EN        TIMER_ROUTE_CC0PEN
#define WAVE_MOTOR_ROUTE_LOCATION  TIMER_ROUTE_LOCATION_LOC0

#define WAVE_MOTOR_PRESCALE        timerPrescale8
#define WAVE_MOTOR_DEFAULT_TOP     131


#define WAVE_LEVEL0 WAVE_MOTOR_DEFAULT_TOP    //0%
#define WAVE_LEVEL1 60                          //40%
#define WAVE_LEVEL2 30                          //50%
#define WAVE_LEVEL3 0                  //65%
//#define WAVE_LEVEL4 26                  //80%
//#define WAVE_LEVEL5 0                   //100%

#define WAVE_MOTOR_CMU_TIMER       cmuClock_TIMER0

#define WAVE_MOTOR_Timer_CCInit     \
{                                   \
    timerEventEveryEdge,            \
    timerEdgeBoth,                  \
    timerPRSSELCh0,                 \
    timerOutputActionNone,          \
    timerOutputActionNone,          \
    timerOutputActionToggle,        \
    timerCCModePWM,                 \
    false,                          \
    false,                          \
    false,                          \
    false,                          \
} 

#define WAVE_MOTOR_Timer_Init       \
{                                   \
    true,                           \
    true,                           \
    WAVE_MOTOR_PRESCALE,            \
    timerClkSelHFPerClk,            \
    timerInputActionNone,           \
    timerInputActionNone,           \
    timerModeUp,                    \
    false,                          \
    false,                          \
    false,                          \
    false,                          \
}


#define WAVE_MOTOR_PWM_CLOSE      0
#define WAVE_MOTOR_PWM_OPEN       1

#define WAVE_MOTOR_PWM_20KHZ      20000

#define WAVE_MOTOR_POLAR_NEGATIVE  0
#define WAVE_MOTOR_POLAR_POSITIVE  1


void Waveringly_Initial_IO(void);
void Waveringly_Set_Pwm_Data(unsigned long ulDuty);
bool WaveMotor_IsRun(void);
#endif
