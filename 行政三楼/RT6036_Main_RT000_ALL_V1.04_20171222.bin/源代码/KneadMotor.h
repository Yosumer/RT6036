#ifndef __KNEAD_MOTOR_H__
#define __KNEAD_MOTOR_H__

#define KNEAD_MOTOR_ENBL_PORT   gpioPortA  
#define KNEAD_MOTOR_ENBL_BIT    10
#define KNEAD_MOTOR_ENBL_MODE   gpioModePushPull

#define KNEAD_MOTOR_FAULT_PORT   gpioPortA  
#define KNEAD_MOTOR_FAULT_BIT    13
#define KNEAD_MOTOR_FAULT_MODE   gpioModeInputPullFilter

#define KNEAD_MOTOR_PHASE_PORT   gpioPortA
#define KNEAD_MOTOR_PHASE_BIT    11
#define KNEAD_MOTOR_PHASE_MODE   gpioModePushPull

#define KNEAD_MOTOR_DECAY_PORT   gpioPortA
#define KNEAD_MOTOR_DECAY_BIT    12
#define KNEAD_MOTOR_DECAY_MODE   gpioModePushPull

#define KNEAD_MOTOR_CURRENT_PORT   gpioPortB
#define KNEAD_MOTOR_CURRENT_BIT    11
#define KNEAD_MOTOR_CURRENT_MODE   gpioModePushPull

#define KNEAD_MOTOR_RESET_PORT   gpioPortA
#define KNEAD_MOTOR_RESET_BIT    14
#define KNEAD_MOTOR_RESET_MODE   gpioModePushPull

#define KNEAD_MOTOR_TIMER           TIMER2
#define KNEAD_MOTOR_TIMER_CHANNEL   2
#define KNEAD_MOTOR_ROUTE_EN        TIMER_ROUTE_CC2PEN
#define KNEAD_MOTOR_ROUTE_LOCATION  TIMER_ROUTE_LOCATION_LOC0

#define KNEAD_MOTOR_PRESCALE        timerPrescale4
#define KNEAD_MOTOR_DEFAULT_TOP     131

#define KNEAD_MOTOR_CMU_TIMER       cmuClock_TIMER1

#define KNEAD_MOTOR_Timer_CCInit     \
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

#define KNEAD_MOTOR_Timer_Init       \
{                                   \
    true,                           \
    true,                           \
    KNEAD_MOTOR_PRESCALE,            \
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

#define KNEAD_MOTOR_PWM_CLOSE      0
#define KNEAD_MOTOR_PWM_OPEN       1

#define KNEAD_MOTOR_PWM_20KHZ      20000

#define KNEAD_MOTOR_POLAR_NEGATIVE  0
#define KNEAD_MOTOR_POLAR_POSITIVE  1

#define KNEAD_SPEED0_PWM  	0 //0% 100%:560
#define KNEAD_SPEED1_PWM 	75//70//300
#define KNEAD_SPEED2_PWM 	87//82//60//350
#define KNEAD_SPEED3_PWM 	99//94//45//400
#define KNEAD_SPEED4_PWM 	111//106//30//450
#define KNEAD_SPEED5_PWM 	123//118//15//500
#define KNEAD_SPEED6_PWM 	132//132//0//560

#define STATE_KNEAD_CLOCK_RUN     1
#define STATE_KNEAD_UNCLOCK_RUN   2
#define STATE_KNEAD_IDLE          0

#define KNEAD_SET_VOLTAGE       260000

enum
{
  KNEAD_MOTOR_CURRENT_HIGH,
  KNEAD_MOTOR_CURRENT_LOW
};
enum
{
  KNEAD_MOTOR_NORMAL,
  KNEAD_MOTOR_FAIL
};
void KneadMotor_Initial_IO(void);
void KneadMotorBrake_On(void);
void KneadMotorBrake_Off(void);
bool KneadMotor_IsRun(void);
void KneadMotor_Set_Current(int current);

void KneadMotor_ClockRun(void);
void KneadMotor_UnClockRun(void);
void KneadMotor_Break(void);
void KneadMotor_Reset(void);
int KneadMotor_Get_Fault(void);
void KneadMotor_Proce(void);
void KneadMotorUpdateSpeed(unsigned int speed);
unsigned int KneadMotor_Control(unsigned int nMotorState,unsigned int PWM);
void KneadMotor_10ms_Int(void);
void KneadMotor_SetCurrent(int current);
unsigned int KneadMotor_GetCurrent(void);
void KneadMotor_AdcCurrent(void);
void KneadMotor_DecCurrent(void);

#endif

