#ifndef __HOT_ROOLER_H__
#define __HOT_ROOLER_H__

#define ROLLER_MOTOR_ENBL_PORT   gpioPortA  
#define ROLLER_MOTOR_ENBL_BIT    2
#define ROLLER_MOTOR_ENBL_MODE   gpioModePushPull

#define ROLLER_MOTOR_FAULT_PORT   gpioPortB  
#define ROLLER_MOTOR_FAULT_BIT    0
#define ROLLER_MOTOR_FAULT_MODE   gpioModeInputPullFilter

#define ROLLER_MOTOR_PHASE_PORT   gpioPortA
#define ROLLER_MOTOR_PHASE_BIT    5
#define ROLLER_MOTOR_PHASE_MODE   gpioModePushPull

#define ROLLER_MOTOR_DECAY_PORT   gpioPortA
#define ROLLER_MOTOR_DECAY_BIT    6
#define ROLLER_MOTOR_DECAY_MODE   gpioModePushPull

#define ROLLER_MOTOR_RESET_PORT   gpioPortB
#define ROLLER_MOTOR_RESET_BIT    1
#define ROLLER_MOTOR_RESET_MODE   gpioModePushPull

#define ROLLER_MOTOR_TIMER           TIMER0
#define ROLLER_MOTOR_TIMER_CHANNEL   2
#define ROLLER_MOTOR_ROUTE_EN        TIMER_ROUTE_CC2PEN
#define ROLLER_MOTOR_ROUTE_LOCATION  TIMER_ROUTE_LOCATION_LOC0

#define ROLLER_MOTOR_PRESCALE        timerPrescale8
#define ROLLER_MOTOR_DEFAULT_TOP     131

#define ROLLER_MOTOR_CMU_TIMER       cmuClock_TIMER0

#define ROLLER_SPEED_STOP     0
#define ROLLER_SPEED_SLOW     75
#define ROLLER_SPEED_MID      95
#define ROLLER_SPEED_FAST     ROLLER_MOTOR_DEFAULT_TOP

#define ROLLER_MOTOR_Timer_CCInit     \
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

#define ROLLER_MOTOR_Timer_Init       \
{                                   \
    true,                           \
    true,                           \
    ROLLER_MOTOR_PRESCALE,            \
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
enum
{
  ROLLER_MOTOR_CURRENT_HIGH,
  ROLLER_MOTOR_CURRENT_LOW
};
enum
{
  ROLLER_MOTOR_NORMAL,
  ROLLER_MOTOR_FAIL
};

#define ROLLER_MOTOR_PWM_CLOSE      0
#define ROLLER_MOTOR_PWM_OPEN       1

#define ROLLER_MOTOR_PWM_20KHZ      20000

#define ROLLER_MOTOR_POLAR_NEGATIVE  0
#define ROLLER_MOTOR_POLAR_POSITIVE  1
#define HOT_ROLLER_DEFAULT_TOP  130

#define ROLLER_SET_VOLTAGE    250000 //À©´ó10000±¶

extern unsigned int displayPWM;

void HotRooler_Initial_IO(void);
bool RollerMotor_IsRun(void);
void RollerMotor_Set_Current(int current);
void RollerMotor_ClockRun(void);
void RollerMotor_UnClockRun(void);
void RollerMotor_Break(void);
void RollerMotor_Reset(void);
void RollerMotor_Reset_Cancel(void);
int RollerMotor_Get_Fault(void);
void RollerMotor_Control(unsigned int ulDuty,unsigned int phase);
unsigned long RollerMotor_Get_Pwm_Data(void);
unsigned char ReadRollerPhase(void);
#endif
