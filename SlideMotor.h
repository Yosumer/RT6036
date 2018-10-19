#ifndef __SLIDE_MOTOR_H__
#define __SLIDE_MOTOR_H__

#define SLIDE_MOTOR_FAULT_PORT   gpioPortE
#define SLIDE_MOTOR_FAULT_BIT    13
#define SLIDE_MOTOR_FAULT_MODE   gpioModeInputPullFilter

#define SLIDE_MOTOR_ENBL_PORT   gpioPortE
#define SLIDE_MOTOR_ENBL_BIT    12
#define SLIDE_MOTOR_ENBL_MODE   gpioModePushPull

#define SLIDE_MOTOR_PHASE_PORT   gpioPortF
#define SLIDE_MOTOR_PHASE_BIT    6
#define SLIDE_MOTOR_PHASE_MODE   gpioModePushPull

#define SLIDE_MOTOR_DECAY_PORT   gpioPortD
#define SLIDE_MOTOR_DECAY_BIT    10
#define SLIDE_MOTOR_DECAY_MODE   gpioModePushPull

#define SLIDE_MOTOR_RESET_PORT   gpioPortE
#define SLIDE_MOTOR_RESET_BIT    14
#define SLIDE_MOTOR_RESET_MODE   gpioModePushPull

#define  SLIDE_MOTOR_AT_FORWARD   0
#define  SLIDE_MOTOR_AT_BACKWARD  1
#define  SLIDE_MOTOR_AT_MID       2
//前滑电动缸最大运行时间ms
#define SLIDE_MOTOR_MAX_POSITION 1600

#define STATE_RUN_SLIDE_BACKWARD        0   
#define STATE_RUN_SLIDE_FORWARD         1 
#define STATE_SLIDE_IDLE                2 
#define SLIDE_MOTOR_TIMER               BACK_MOTOR_TIMER
#define SLIDE_MOTOR_CHANNEL             2
#define SLIDE_MOTOR_ROUTE_EN            TIMER_ROUTE_CC2PEN
#define SLIDE_MOTOR_ROUTE_LOCATION      TIMER_ROUTE_LOCATION_LOC1
#define SLIDE_MOTOR_DEFAULT_TOP         BACK_MOTOR_DEFAULT_TOP
#define SLIDE_SET_VOLTAGE    250000 //25V

#define SLIDE_SPEED_STOP     0
#define SLIDE_SPEED_SLOW     85
#define SLIDE_SPEED_MID      110
#define SLIDE_SPEED_FAST     131


#define SLIDE_MOTOR_PRESCALE     timerPrescale8

#define SLIDE_MOTOR_Timer_CCInit     \
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


#define SLIDE_MOTOR_Timer_Init       \
{                                   \
    true,                           \
    true,                           \
    SLIDE_MOTOR_PRESCALE,            \
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
  SLIDE_MOTOR_POWER_ON, 
  SLIDE_MOTOR_POWER_OFF 
};

enum
{
  SLIDE_MOTOR_GO_FORWARD, 
  SLIDE_MOTOR_GO_BACKWARD 
};

enum
{
 SLIDE_MOTOR_CURRENT_HIGH,
 SLIDE_MOTOR_CURRENT_LOW
};

enum
{
 SLIDE_MOTOR_NORMAL,
 SLIDE_MOTOR_FAIL
};

void ZeroMotor_Initial_IO(void);
void SlideMotor_Reset(void);
void SlideMotor_Reset_Cancel(void);
int SlideMotorPower_On(void);
void SlideMotorPower_Off(void);
void SlideMotor_Forward(void);
void SlideMotor_Backward(void);
void SlideMotor_Break(void);
int SlideMotor_Get_Fault(void);
unsigned int SlideMotor_Get_Location(void);
unsigned char SlideMotorControl(unsigned char nFinalZeroPadMotorState);
unsigned char VibrateLeftMotorControl(unsigned char nFinalZeroPadMotorState,unsigned char speed);

int SlideMotor_GetPower(void);
int SlideMotor_GetDirection(void);
void SlideMotor_10ms_Int(void);

void SlideMotor_Set_Route(void);
void SlideMotor_Set_PWM(unsigned long ulDuty);

#endif
