#ifndef __BACKPAD_H__
#define __BACKPAD_H__
//TIM1 CC0 LOCATION 1
#define BACK_MOTOR_ENBL_PORT   gpioPortE
#define BACK_MOTOR_ENBL_BIT    10
#define BACK_MOTOR_ENBL_MODE   gpioModePushPull

#define BACK_MOTOR_PHASE_PORT   gpioPortD
#define BACK_MOTOR_PHASE_BIT    11
#define BACK_MOTOR_PHASE_MODE   gpioModePushPull

#define BACK_MOTOR_DECAY_PORT   gpioPortD
#define BACK_MOTOR_DECAY_BIT    12
#define BACK_MOTOR_DECAY_MODE   gpioModePushPull

#define BACK_MOTOR_FAULT_PORT   gpioPortE
#define BACK_MOTOR_FAULT_BIT    8
#define BACK_MOTOR_FAULT_MODE   gpioModeInputPullFilter

#define BACK_MOTOR_RESET_PORT   gpioPortE
#define BACK_MOTOR_RESET_BIT    9
#define BACK_MOTOR_RESET_MODE   gpioModePushPull

#define BACK_MOTOR_AT_MID     0
#define BACK_MOTOR_AT_BOTTOM  1
#define BACK_MOTOR_AT_TOP     2
//靠背电动缸最大运行时间ms
#define BACK_MOTOR_MAX_POSITION 2300

//靠背电动缸最大计数位置
#define BACK_MOTOR_MAX_POSITION_PULSE 474

#define BACK_MIN_SPEED        3
//停止0，刹车1，加速2，缓刹车3，缓加速4
#define BACK_SPEED_STATE_STOP         0
#define BACK_SPEED_STATE_BREAK        1
#define BACK_SPEED_STATE_UP           2
#define BACK_SPEED_STATE_SLOW_BREAK   3
#define BACK_SPEED_STATE_SLOW_UP      4

#define STATE_RUN_BACK_DOWN   0
#define STATE_RUN_BACK_UP     1
#define STATE_BACK_IDLE       2

#define BACK_MOTOR_TIMER           TIMER1
#define BACK_MOTOR_TIMER_CHANNEL   0
#define BACK_MOTOR_ROUTE_EN        TIMER_ROUTE_CC0PEN
#define BACK_MOTOR_ROUTE_LOCATION  TIMER_ROUTE_LOCATION_LOC1

#define BACK_MOTOR_PRESCALE        timerPrescale4
#define BACK_MOTOR_DEFAULT_TOP     100//100//fww 100 charge to 131
#define BACK_SET_VOLTAGE    250000 // 25V

#define BACK_MOTOR_Timer_CCInit     \
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
/*
#define BACK_MOTOR_Timer_Init     \
{                                   \
    true,                           \
    true,                           \
    BACK_MOTOR_PRESCALE,            \
    timerClkSelHFPerClk,            \
    timerInputActionNone,           \
    timerInputActionNone,           \
    timerModeUp,                    \
    false,                          \
    false,                          \
    false,                          \
    false,                          \
} */
#define BACK_MOTOR_Timer_Init      \
{                                   \
    true,                           \
    true,                           \
    BACK_MOTOR_PRESCALE,            \
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
  BACK_MOTOR_POWER_ON, 
  BACK_MOTOR_POWER_OFF 
};
enum
{
  BACK_MOTOR_GO_UP, 
  BACK_MOTOR_GO_DOWN 
};

enum
{
 BACK_MOTOR_CURRENT_HIGH,
 BACK_MOTOR_CURRENT_LOW
};

enum
{
 BACK_MOTOR_NORMAL,
 BACK_MOTOR_FAIL
};
int BackPower_On(void);
void BackPower_Off(void);
unsigned int BackRelay_Get(void);
//unsigned int BackPower_Get(void);
void BackMotor_Initial_IO(void);
void BackMotor_Up(void);
void BackMotor_Down(void);
void BackMotor_Break(void);
void BackMotor_Reset(void);
void BackMotor_Reset_Cancel(void);
int BackMotor_Get_Fault(void);
void BackMotor_10ms_int(void);
void BackMotor_Proce(void);
unsigned int BackMotor_Get_Position(void);
unsigned char BackMotor_Control(unsigned char nFinalBackPadMotorState);

int BackMotor_GetPower(void);
int BackMotor_GetDirection(void);
void BackMotor_10ms_int(void);
void setBackPadRockingEnable(bool flag);
void BackMotor_Set_Route(void);
void BackMotor_Set_PWM(unsigned long ulDuty);

#endif
