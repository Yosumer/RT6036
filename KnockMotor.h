#ifndef __KNOCK_MOTOR_H__
#define __KNOCK_MOTOR_H__

#define KNOCK_MOTOR_ENBL_PORT   gpioPortA  
#define KNOCK_MOTOR_ENBL_BIT    9
#define KNOCK_MOTOR_ENBL_MODE   gpioModePushPull

#define KNOCK_MOTOR_FAULT_PORT   gpioPortC  
#define KNOCK_MOTOR_FAULT_BIT    4
#define KNOCK_MOTOR_FAULT_MODE   gpioModeInputPullFilter

#define KNOCK_MOTOR_PHASE_PORT   gpioPortC
#define KNOCK_MOTOR_PHASE_BIT    2
#define KNOCK_MOTOR_PHASE_MODE   gpioModePushPull

#define KNOCK_MOTOR_DECAY_PORT   gpioPortC
#define KNOCK_MOTOR_DECAY_BIT    3
#define KNOCK_MOTOR_DECAY_MODE   gpioModePushPull

#define KNOCK_MOTOR_CURRENT_PORT   gpioPortB
#define KNOCK_MOTOR_CURRENT_BIT    12
#define KNOCK_MOTOR_CURRENT_MODE   gpioModePushPull

#define KNOCK_MOTOR_RESET_PORT   gpioPortC
#define KNOCK_MOTOR_RESET_BIT    5
#define KNOCK_MOTOR_RESET_MODE   gpioModePushPull

#define KNOCK_MOTOR_TIMER           TIMER2
#define KNOCK_MOTOR_TIMER_CHANNEL   1
#define KNOCK_MOTOR_ROUTE_EN        TIMER_ROUTE_CC1PEN
#define KNOCK_MOTOR_ROUTE_LOCATION  TIMER_ROUTE_LOCATION_LOC0

#define KNOCK_MOTOR_PRESCALE        timerPrescale4
#define KNOCK_MOTOR_DEFAULT_TOP     131

#define KNOCK_MOTOR_CMU_TIMER       cmuClock_TIMER1

#define KNOCK_MOTOR_Timer_CCInit     \
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

#define KNOCK_MOTOR_Timer_Init      \
{                                   \
    true,                           \
    true,                           \
    KNOCK_MOTOR_PRESCALE,            \
    timerClkSelHFPerClk,            \
    false,                          \
    false,                          \
    timerInputActionNone,           \
    timerInputActionNone,           \
    timerModeUp,                    \
    false,                          \
    false,                          \
    false,                          \
    false,                          \
}

#define KNOCK_MOTOR_PWM_CLOSE      0
#define KNOCK_MOTOR_PWM_OPEN       1

#define KNOCK_MOTOR_PWM_20KHZ      20000

#define KNOCK_MOTOR_POLAR_NEGATIVE  0
#define KNOCK_MOTOR_POLAR_POSITIVE  1

#define KNOCK_SPEED0_PWM 	0       //0%
#define KNOCK_SPEED1_PWM 	70//300
#define KNOCK_SPEED2_PWM 	82//350
#define KNOCK_SPEED3_PWM 	94//400
#define KNOCK_SPEED4_PWM 	106//450
#define KNOCK_SPEED5_PWM 	118//500
#define KNOCK_SPEED6_PWM 	132//550
#define KNOCK_SET_VOLTAGE       250000 
enum
{
  KNOCK_MOTOR_CURRENT_HIGH,
  KNOCK_MOTOR_CURRENT_LOW
};

enum{
  KNOCK_MOTOR_NORMAL,
  KNOCK_MOTOR_FAIL
};

void KnockMotor_Initial_IO(void);
void KnockMotor_Set_Pwm_Data(unsigned long ulDuty);
void KnockMotor_ClockRun(void);
void KnockMotor_UnClockRun(void);
void KnockMotor_Break(void);
void KnockMotor_Reset(void);
int KnockMotor_Get_Fault(void);
void KnockMotor_SetCurrent(unsigned int current);
unsigned int KnockMotor_GetCurrent(void);
void KnockMotor_AdcCurrent(void);
void KnockMotor_DecCurrent(void);

#endif
