#ifndef __ZERO_MOTOR_H__
#define __ZERO_MOTOR_H__

#define ZERO_MOTOR_FAULT_PORT   gpioPortE
#define ZERO_MOTOR_FAULT_BIT    9
#define ZERO_MOTOR_FAULT_MODE   gpioModeInputPullFilter

#define ZERO_MOTOR_ENBL_PORT   gpioPortE
#define ZERO_MOTOR_ENBL_BIT    10
#define ZERO_MOTOR_ENBL_MODE   gpioModePushPull

#define ZERO_MOTOR_PHASE_PORT   gpioPortE
#define ZERO_MOTOR_PHASE_BIT    11
#define ZERO_MOTOR_PHASE_MODE   gpioModePushPull

#define ZERO_MOTOR_DECAY_PORT   gpioPortE
#define ZERO_MOTOR_DECAY_BIT    12
#define ZERO_MOTOR_DECAY_MODE   gpioModePushPull

#define ZERO_MOTOR_RESET_PORT   gpioPortE
#define ZERO_MOTOR_RESET_BIT    13
#define ZERO_MOTOR_RESET_MODE   gpioModePushPull

#define  SLIDE_MOTOR_AT_FORWARD   0
#define  SLIDE_MOTOR_AT_BACKWARD  1
#define  SLIDE_MOTOR_AT_MID       2
//前滑电动缸最大运行时间ms
#define SLIDE_MOTOR_MAX_POSITION 1600

#define STATE_RUN_SLIDE_BACKWARD 0   
#define STATE_RUN_SLIDE_FARWARD  1 
#define STATE_SLIDE_IDLE           2 

enum
{
 ZERO_MOTOR_CURRENT_HIGH,
 ZERO_MOTOR_CURRENT_LOW
};

enum
{
 ZERO_MOTOR_NORMAL,
 ZERO_MOTOR_FAIL
};

void ZeroMotor_Initial_IO(void);
void ZeroRelay_On(void);
void ZeroRelay_Off(void);
int ZeroPower_On(void);
void ZeroPower_Off(void);
unsigned int ZeroRelay_Get(void);
unsigned int ZeroPower_Get(void);

void ZeroMotor_Up(void);
void ZeroMotor_Down(void);
void ZeroMotor_Break(void);
void SlideMotor_Break(void);
void ZeroMotor_Reset(void);
int ZeroMotor_Get_Fault(void);
void SlideMotor_Forward(void);
void SlideMotor_Backward(void);
int SlideMotorPower_On(void);
void SlideMotorPower_Off(void);
#endif
