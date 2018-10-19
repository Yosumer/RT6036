#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "EFM32_def.h"
#include "EFM32_types.h"
#include "input.h"
#include "MassageStatus.h"
#include "memory.h"
#include "LED_RGB.h"
#include "ADC_Single.h"
#include "SlideMotor.h"
#include "BackPad.h"

static unsigned int motorStatus = MOTOR_STOP_BREAK;
static volatile unsigned int motorBreakTime;

void ZeroMotor_Initial_IO(void)
{
  GPIO_PinModeSet(SLIDE_MOTOR_RESET_PORT, SLIDE_MOTOR_RESET_BIT, SLIDE_MOTOR_RESET_MODE, 0);
  GPIO_PinModeSet(SLIDE_MOTOR_ENBL_PORT, SLIDE_MOTOR_ENBL_BIT, SLIDE_MOTOR_ENBL_MODE, 0);
  GPIO_PinModeSet(SLIDE_MOTOR_PHASE_PORT, SLIDE_MOTOR_PHASE_BIT, SLIDE_MOTOR_PHASE_MODE, 0);
  GPIO_PinModeSet(SLIDE_MOTOR_DECAY_PORT, SLIDE_MOTOR_DECAY_BIT, SLIDE_MOTOR_DECAY_MODE, 0);
  GPIO_PinModeSet(SLIDE_MOTOR_FAULT_PORT, SLIDE_MOTOR_FAULT_BIT, SLIDE_MOTOR_FAULT_MODE, 1);

  TIMER_InitCC_TypeDef timerCCInit = SLIDE_MOTOR_Timer_CCInit;
  TIMER_InitCC(SLIDE_MOTOR_TIMER, SLIDE_MOTOR_CHANNEL, &timerCCInit);
  TIMER_TopSet(SLIDE_MOTOR_TIMER, SLIDE_MOTOR_DEFAULT_TOP);
  TIMER_CompareBufSet(SLIDE_MOTOR_TIMER, SLIDE_MOTOR_CHANNEL, 0);

  TIMER_Init_TypeDef timerInit = SLIDE_MOTOR_Timer_Init;
  TIMER_Init(SLIDE_MOTOR_TIMER, &timerInit);
  SLIDE_MOTOR_TIMER->ROUTE |= (SLIDE_MOTOR_ROUTE_EN | \
    SLIDE_MOTOR_ROUTE_LOCATION); 
  TIMER_CompareBufSet(SLIDE_MOTOR_TIMER, SLIDE_MOTOR_CHANNEL, 80);
}
void SlideMotor_Reset(void)
{
  GPIO_PinOutClear(SLIDE_MOTOR_RESET_PORT, SLIDE_MOTOR_RESET_BIT);
}
void SlideMotor_Reset_Cancel(void)
{
  GPIO_PinOutSet(SLIDE_MOTOR_RESET_PORT, SLIDE_MOTOR_RESET_BIT);
}

unsigned int SlideMotor_VoltageAdj(void)
{
  unsigned short adc24;
   ADC_Get_Voltage(ADC_V24,&adc24);     
  unsigned int duty = SLIDE_MOTOR_DEFAULT_TOP;
  
  if(adc24 < SLIDE_SET_VOLTAGE/100) 
  {
    return duty; 
  }
  unsigned int yushu = SLIDE_SET_VOLTAGE % adc24;
  duty = SLIDE_SET_VOLTAGE / adc24;
  if(yushu > adc24/2) duty++;
  return duty; 
}
int SlideMotorPower_On(void)
{
   unsigned long  ulDuty;
   ulDuty = SlideMotor_VoltageAdj();
   if(SlideMotor_Get_Fault() == SLIDE_MOTOR_FAIL)
    {
      SlideMotor_Reset();
      __no_operation();
      __no_operation();
      SlideMotor_Reset_Cancel();
    }
   SlideMotor_Set_PWM(ulDuty);
   motorStatus = MOTOR_RUN;
   return 1;
}

void SlideMotorPower_Off(void)
{
  if(motorStatus == MOTOR_STOP_BREAK)
  {
    GPIO_PinOutClear(SLIDE_MOTOR_ENBL_PORT, SLIDE_MOTOR_ENBL_BIT);  
  }
  else
  {
    SlideMotor_Set_PWM(0);
  }
}

void SlideMotor_Forward(void)
{
  GPIO_PinOutSet(SLIDE_MOTOR_RESET_PORT, SLIDE_MOTOR_RESET_BIT);
  GPIO_PinOutClear(SLIDE_MOTOR_PHASE_PORT, SLIDE_MOTOR_PHASE_BIT);
  GPIO_PinOutSet(SLIDE_MOTOR_DECAY_PORT,SLIDE_MOTOR_DECAY_BIT);
}
void SlideMotor_Backward(void)
{
  GPIO_PinOutSet(SLIDE_MOTOR_RESET_PORT, SLIDE_MOTOR_RESET_BIT);
  GPIO_PinOutSet(SLIDE_MOTOR_PHASE_PORT, SLIDE_MOTOR_PHASE_BIT);
  GPIO_PinOutSet(SLIDE_MOTOR_DECAY_PORT,SLIDE_MOTOR_DECAY_BIT);
}

void SlideMotor_10ms_Int(void)
{
  motorBreakTime++; 
}

void SlideMotor_Break(void)
{
  SlideMotorPower_Off();    //关闭马达电源
  if(motorStatus == MOTOR_RUN)
  {
   GPIO_PinOutSet(SLIDE_MOTOR_DECAY_PORT, SLIDE_MOTOR_DECAY_BIT); //使马达端口处于高阻，此时马达属于惯性滑行
   motorStatus = MOTOR_STOP_HZ;
   motorBreakTime = 0;
  }
  if(motorStatus == MOTOR_STOP_HZ)
  {
    if(motorBreakTime < MOTOR_STOP_HZ_TIME) return;
  }
  GPIO_PinOutClear(SLIDE_MOTOR_DECAY_PORT, SLIDE_MOTOR_DECAY_BIT);  //短路马达，保持马达在刹车状态
  motorStatus = MOTOR_STOP_BREAK;
  if(SLIDE_MOTOR_TIMER->ROUTE == (SLIDE_MOTOR_ROUTE_EN | SLIDE_MOTOR_ROUTE_LOCATION)) 
  {
    SLIDE_MOTOR_TIMER->ROUTE &= ~(SLIDE_MOTOR_ROUTE_EN | SLIDE_MOTOR_ROUTE_LOCATION); 
  }
}
int SlideMotor_Get_Fault(void)
{
  if(GPIO_PinInGet(SLIDE_MOTOR_FAULT_PORT, SLIDE_MOTOR_FAULT_BIT))
    return SLIDE_MOTOR_NORMAL;
  return SLIDE_MOTOR_FAIL;
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
  static unsigned int position = 0;
  unsigned char nRetVal ;
  bool bPowerFlag;
  nRetVal = FALSE ;
  
  bool enable = ReadEEByte(USER_DATA_BASE + SLIDE_MOTOR_ENABLE_ADDRESS);
  if(!enable) 
  {
    SlideMotorPower_Off();
    SlideMotor_Break();
    return TRUE;
  }
  switch(nFinalZeroPadMotorState)
  {
  case STATE_RUN_SLIDE_BACKWARD:
    if(Input_GetSlideBackwardSwitch() == REACH_SLIDE_LIMIT || position == 1) 
    {
      position = 1;
      nRetVal = TRUE ;
      SlideMotor_Break();
      bPowerFlag = FALSE;
      break;
    }
    position = 0;
    //SlideMotor_Set_Route();
    SlideMotor_Backward();
    bPowerFlag = TRUE;
    break ;
  case STATE_RUN_SLIDE_FORWARD:
    if(Input_GetSlideForwardSwitch() == REACH_SLIDE_LIMIT || position == 2)
    {
      position = 2;
      bPowerFlag = FALSE;
      nRetVal = TRUE ;
      SlideMotor_Break();
      break;
    }
    position = 0;
    //SlideMotor_Set_Route();
    SlideMotor_Forward();
    bPowerFlag = TRUE;
    break ;
  case STATE_SLIDE_IDLE:
    nRetVal = TRUE ;
    //SlideMotor_Set_Route();
    SlideMotor_Break();
    bPowerFlag = FALSE;
    break ;
  default://异常处理
    break ;
  }
  //电源部分的处理
  if(bPowerFlag == TRUE)
  {
    SlideMotorPower_On();
  }
  else
  {
    SlideMotorPower_Off();
  }
  return nRetVal ;
}

unsigned char VibrateLeftMotorControl(unsigned char nFinalZeroPadMotorState,unsigned char speed)
{
  static unsigned int position = 0;
  unsigned char nRetVal ;
  bool bPowerFlag;
  nRetVal = FALSE ;
  
  bool enable = ReadEEByte(USER_DATA_BASE + SLIDE_MOTOR_ENABLE_ADDRESS);
  if(!enable) 
  {
    SlideMotorPower_Off();
    SlideMotor_Break();
    return TRUE;
  }
  switch(nFinalZeroPadMotorState)
  {
  case STATE_RUN_SLIDE_BACKWARD:
    if(Input_GetSlideBackwardSwitch() == REACH_SLIDE_LIMIT || position == 1) 
    {
      position = 1;
      nRetVal = TRUE ;
      SlideMotor_Break();
      bPowerFlag = FALSE;
      break;
    }
    position = 0;
    //SlideMotor_Set_Route();
    SlideMotor_Backward();
    bPowerFlag = TRUE;
    break ;
  case STATE_RUN_SLIDE_FORWARD:
    if(Input_GetSlideForwardSwitch() == REACH_SLIDE_LIMIT || position == 2)
    {
      position = 2;
      bPowerFlag = FALSE;
      nRetVal = TRUE ;
      SlideMotor_Break();
      break;
    }
    position = 0;
    //SlideMotor_Set_Route();
    SlideMotor_Forward();
    bPowerFlag = TRUE;
    break ;
  case STATE_SLIDE_IDLE:
    nRetVal = TRUE ;
    //SlideMotor_Set_Route();
    SlideMotor_Break();
    bPowerFlag = FALSE;
    break ;
  default://异常处理
    break ;
  }
  //电源部分的处理
  if(bPowerFlag == TRUE)
  {
    SlideMotor_Set_PWM(speed);
  }
  else
  {
    SlideMotor_Set_PWM(0);
  }
  return nRetVal ;
}

int SlideMotor_GetPower(void)
{
  if(SLIDE_MOTOR_TIMER->ROUTE == (SLIDE_MOTOR_ROUTE_EN | SLIDE_MOTOR_ROUTE_LOCATION)) 
  {
    unsigned long  ulDuty;
    ulDuty = TIMER_CompareBufGet(SLIDE_MOTOR_TIMER, SLIDE_MOTOR_CHANNEL); 
    if(ulDuty > 0) return SLIDE_MOTOR_POWER_ON;
    else return SLIDE_MOTOR_POWER_OFF; 
  }
  
  if(GPIO_PinOutGet(SLIDE_MOTOR_ENBL_PORT,SLIDE_MOTOR_ENBL_BIT))
  {
    return SLIDE_MOTOR_POWER_ON; 
  }
  return SLIDE_MOTOR_POWER_OFF; 
}
int SlideMotor_GetDirection(void)
{
  if(GPIO_PinOutGet(SLIDE_MOTOR_PHASE_PORT,SLIDE_MOTOR_PHASE_BIT))
  {
   return SLIDE_MOTOR_GO_BACKWARD; 
  }
  return SLIDE_MOTOR_GO_FORWARD; 
}

void SlideMotor_Set_Route(void)
{
  if(SLIDE_MOTOR_TIMER->ROUTE != (SLIDE_MOTOR_ROUTE_EN | SLIDE_MOTOR_ROUTE_LOCATION))
  {
    TIMER_Init_TypeDef timerInit = SLIDE_MOTOR_Timer_Init;
    TIMER_Init(SLIDE_MOTOR_TIMER, &timerInit);
    SLIDE_MOTOR_TIMER->ROUTE = (SLIDE_MOTOR_ROUTE_EN | SLIDE_MOTOR_ROUTE_LOCATION); 
    SlideMotor_Set_PWM(0);
  }
}

void SlideMotor_Set_PWM(unsigned long ulDuty)
{
  TIMER_CompareBufSet(SLIDE_MOTOR_TIMER, SLIDE_MOTOR_CHANNEL, ulDuty);
}