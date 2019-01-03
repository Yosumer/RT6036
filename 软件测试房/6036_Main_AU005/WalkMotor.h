#ifndef __WALK_MOTOR_H__
#define __WALK_MOTOR_H__

#define WALK_MOTOR_ENBL_PORT   gpioPortA    //PWM
#define WALK_MOTOR_ENBL_BIT    8
#define WALK_MOTOR_ENBL_MODE   gpioModePushPull

#define WALK_MOTOR_PHASE_PORT   gpioPortB
#define WALK_MOTOR_PHASE_BIT    4
#define WALK_MOTOR_PHASE_MODE   gpioModePushPull

#define WALK_MOTOR_DECAY_PORT   gpioPortB
#define WALK_MOTOR_DECAY_BIT    5
#define WALK_MOTOR_DECAY_MODE   gpioModePushPull

#define WALK_MOTOR_FAULT_PORT   gpioPortB
#define WALK_MOTOR_FAULT_BIT    6
#define WALK_MOTOR_FAULT_MODE   gpioModeInputPullFilter

#define WALK_MOTOR_RESET_PORT   gpioPortC
#define WALK_MOTOR_RESET_BIT    0
#define WALK_MOTOR_RESET_MODE   gpioModePushPull

#define STATE_RUN_WALK_DOWN     0
#define STATE_RUN_WALK_UP       1
#define STATE_WALK_IDLE         2
#define STATE_RUN_WALK_POSITION 3
#define WALK_SET_VOLTAGE     250000//fww

#define WALK_MOTOR_TIMER           TIMER2
#define WALK_MOTOR_TIMER_CHANNEL   0
#define WALK_MOTOR_ROUTE_EN        TIMER_ROUTE_CC0PEN
#define WALK_MOTOR_ROUTE_LOCATION  TIMER_ROUTE_LOCATION_LOC0

#define WALK_MOTOR_PRESCALE        timerPrescale4
#define WALK_MOTOR_DEFAULT_TOP     131

#define WALK_MOTOR_CMU_TIMER       cmuClock_TIMER1

#define WALK_MOTOR_Timer_CCInit     \
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

#define WALK_MOTOR_Timer_Init      \
{                                   \
    true,                           \
    true,                           \
    WALK_MOTOR_PRESCALE,            \
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
  WALK_MOTOR_POWER_ON, 
  WALK_MOTOR_POWER_OFF 
};


enum
{
 WALK_MOTOR_CURRENT_HIGH,
 WALK_MOTOR_CURRENT_LOW
};

enum
{
 WALK_MOTOR_NORMAL,
 WALK_MOTOR_FAIL
};
unsigned int WalkRelay_Get(void);
unsigned int WalkPower_Get(void);
void WalkMotor_Initial_IO(void);

void WalkMotor_Set_Current(int current);
int WalkMotor_Get_Fault(void);
unsigned char WalkMotor_Control(unsigned char nFinalWalkPadMotorState,unsigned short stopPosition);
void WalkMotor_10ms_Int(void);
#endif

