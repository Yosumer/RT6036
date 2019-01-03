#include "efm32.h"
#include "efm32_chip.h"
#include "efm32_gpio.h"
#include "efm32_timer.h"
#include "EFM32_def.h"
#include "EFM32_types.h"
#include "input.h"
#include "power.h"
#include "ZeroMotor.h"
void ZeroMotor_Initial_IO(void)
{
  GPIO_PinModeSet(ZERO_MOTOR_RESET_PORT, ZERO_MOTOR_RESET_BIT, ZERO_MOTOR_RESET_MODE, 0);
  GPIO_PinModeSet(ZERO_MOTOR_ENBL_PORT, ZERO_MOTOR_ENBL_BIT, ZERO_MOTOR_ENBL_MODE, 0);
  GPIO_PinModeSet(ZERO_MOTOR_PHASE_PORT, ZERO_MOTOR_PHASE_BIT, ZERO_MOTOR_PHASE_MODE, 0);
  GPIO_PinModeSet(ZERO_MOTOR_DECAY_PORT, ZERO_MOTOR_DECAY_BIT, ZERO_MOTOR_DECAY_MODE, 0);
  GPIO_PinModeSet(ZERO_MOTOR_FAULT_PORT, ZERO_MOTOR_FAULT_BIT, ZERO_MOTOR_FAULT_MODE, 1);
}
void ZeroMotor_Reset(void)
{
  GPIO_PinOutClear(ZERO_MOTOR_RESET_PORT, ZERO_MOTOR_RESET_BIT);
}
void ZeroMotor_Reset_Cancel(void)
{
  GPIO_PinOutSet(ZERO_MOTOR_RESET_PORT, ZERO_MOTOR_RESET_BIT);
}

void ZeroRelay_On(void)
{
  GPIO_PinOutSet(ZERO_MOTOR_PHASE_PORT, ZERO_MOTOR_PHASE_BIT);
}
void ZeroRelay_Off(void)
{
  GPIO_PinOutClear(ZERO_MOTOR_PHASE_PORT, ZERO_MOTOR_PHASE_BIT);
}
int ZeroPower_On(void)
{
  int val = 0;
  Power_On();
  if(GPIO_PinOutGet(ZERO_MOTOR_ENBL_PORT,ZERO_MOTOR_ENBL_BIT))
  { //端口已经high
    if(ZeroMotor_Get_Fault() == ZERO_MOTOR_FAIL)
    {
      ZeroMotor_Reset();
      __no_operation();
      __no_operation();
      ZeroMotor_Reset_Cancel();
      __no_operation();
      if(ZeroMotor_Get_Fault() != ZERO_MOTOR_FAIL) val = 0;
    }
  }
  else
  {
   GPIO_PinOutSet(ZERO_MOTOR_ENBL_PORT, ZERO_MOTOR_ENBL_BIT);
   val =  1;
  }
  return val;
}
int SlideMotorPower_On(void)
{
  int val = 0;
  Power_On();
  if(GPIO_PinOutGet(ZERO_MOTOR_ENBL_PORT,ZERO_MOTOR_ENBL_BIT))
  {   //端口已经输出high
    if(ZeroMotor_Get_Fault() == ZERO_MOTOR_FAIL)
    {
      ZeroMotor_Reset();
      __no_operation();
      __no_operation();
      ZeroMotor_Reset_Cancel();
      __no_operation();
      if(ZeroMotor_Get_Fault() != ZERO_MOTOR_FAIL) val = 0;
    }
  }
  else
  {
   GPIO_PinOutSet(ZERO_MOTOR_ENBL_PORT, ZERO_MOTOR_ENBL_BIT);
   val =  1;
  }
  return val;
}
void SlideMotorPower_Off(void)
{
  GPIO_PinOutClear(ZERO_MOTOR_ENBL_PORT, ZERO_MOTOR_ENBL_BIT);
}
void ZeroPower_Off(void)
{
  GPIO_PinOutClear(ZERO_MOTOR_ENBL_PORT, ZERO_MOTOR_ENBL_BIT);
}
unsigned int ZeroRelay_Get(void)
{
  return(GPIO_PinOutGet(ZERO_MOTOR_PHASE_PORT, ZERO_MOTOR_PHASE_BIT));
}
unsigned int ZeroPower_Get(void)
{
  return(GPIO_PinOutGet(ZERO_MOTOR_PHASE_PORT, ZERO_MOTOR_PHASE_BIT));
}

/*
void ZeroMotor_Set_Current(int current)
{
  if(current == ZERO_MOTOR_CURRENT_HIGH)
  {
   GPIO_PinOutSet(ZERO_MOTOR_I4_PORT, ZERO_MOTOR_I4_BIT); 
  }
  if(current == ZERO_MOTOR_CURRENT_LOW)
  {
   GPIO_PinOutClear(ZERO_MOTOR_I4_PORT, ZERO_MOTOR_I4_BIT); 
  }
}
*/
void ZeroMotor_Up(void)
{
  GPIO_PinOutSet(ZERO_MOTOR_RESET_PORT, ZERO_MOTOR_RESET_BIT);
  GPIO_PinOutSet(ZERO_MOTOR_PHASE_PORT, ZERO_MOTOR_PHASE_BIT);
}
void ZeroMotor_Down(void)
{
  GPIO_PinOutSet(ZERO_MOTOR_RESET_PORT, ZERO_MOTOR_RESET_BIT);
  GPIO_PinOutClear(ZERO_MOTOR_PHASE_PORT, ZERO_MOTOR_PHASE_BIT);
}

void SlideMotor_Forward(void)
{
  GPIO_PinOutSet(ZERO_MOTOR_RESET_PORT, ZERO_MOTOR_RESET_BIT);
  GPIO_PinOutClear(ZERO_MOTOR_PHASE_PORT, ZERO_MOTOR_PHASE_BIT);
}
void SlideMotor_Backward(void)
{
  GPIO_PinOutSet(ZERO_MOTOR_RESET_PORT, ZERO_MOTOR_RESET_BIT);
  GPIO_PinOutSet(ZERO_MOTOR_PHASE_PORT, ZERO_MOTOR_PHASE_BIT);
}
void ZeroMotor_Break(void)
{
  GPIO_PinOutClear(ZERO_MOTOR_DECAY_PORT, ZERO_MOTOR_DECAY_BIT);
  GPIO_PinOutClear(ZERO_MOTOR_ENBL_PORT, ZERO_MOTOR_ENBL_BIT);
}
void SlideMotor_Break(void)
{
  GPIO_PinOutClear(ZERO_MOTOR_DECAY_PORT, ZERO_MOTOR_DECAY_BIT);
  GPIO_PinOutClear(ZERO_MOTOR_ENBL_PORT, ZERO_MOTOR_ENBL_BIT);
}
int ZeroMotor_Get_Fault(void)
{
  if(GPIO_PinInGet(ZERO_MOTOR_FAULT_PORT, ZERO_MOTOR_FAULT_BIT))
    return ZERO_MOTOR_NORMAL;
  return ZERO_MOTOR_FAIL;
}

//返回靠背电动缸的大位置 最高，最低或中间
unsigned int SlideMotor_Get_Location(void)
{
  if(Input_GetSlideForwardSwitch() == REACH_SLIDE_LIMIT)   return SLIDE_MOTOR_AT_FORWARD;
  if(Input_GetSlideBackwardSwitch() == REACH_SLIDE_LIMIT)   return SLIDE_MOTOR_AT_BACKWARD;
  return SLIDE_MOTOR_AT_MID;
}

unsigned char SlideMotorControl(unsigned char nFinalZeroPadMotorState)
{
  unsigned char nRetVal ;
  nRetVal = FALSE ;
  switch(nFinalZeroPadMotorState)
  {
  case STATE_RUN_SLIDE_BACKWARD:
    nCurZeroPadMotorState = STATE_RUN_SLIDE_BACKWARD;
    if(Input_GetSlideBackwardSwitch() == REACH_ZERO_LIMIT)
    {
      nRetVal = TRUE ;
      //nZeroMotorRemoveTime = MAX_ZERO_MOTOR_REMOVE_TIME;
      ZeroMotor_Break();
      break;
    }
    SlideMotor_Backward();
    bZeroPadMotorPowerFlag = TRUE;
    break ;
  case STATE_RUN_SLIDE_FORWARD:
    nCurZeroPadMotorState = STATE_RUN_SLIDE_FORWARD;
    if(Input_GetSlideForwardSwitch() == REACH_ZERO_LIMIT)
    {
      nRetVal = TRUE ;
      nZeroMotorRemoveTime = 0;
      ZeroMotor_Break();
      break;
    }
    SlideMotor_Forward();
    bZeroPadMotorPowerFlag = TRUE;
    break ;
  case STATE_SLIDE_IDLE:
    nCurZeroPadMotorState = STATE_IDLE;
    nRetVal = TRUE ;
    ZeroMotor_Break();
    bZeroPadMotorPowerFlag = FALSE;
    break ;
  default://异常处理
    break ;
  }
  //电源部分的处理
  if(bZeroPadMotorPowerFlag == TRUE)
  {
    SlideMotorPower_On();
  }
  else
  {
    SlideMotorPower_Off();
  }
  return nRetVal ;
}