#include "EFM32_def.h"
#include "EFM32_types.h"
#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "input.h"
#include "valve.h"
#include "power.h"
#include "LegMotor.h"
#include "MassageStatus.h"
//140602
#include "timer.h"
#include "FlexPad.h"
#include "Hot_Roller.h"

static bool bFalg ; 
//static bool Flex_100ms_Flag;
//static unsigned int ResetTime;
void FlexMotor_Reset(void);
void FlexMotor_Reset_Cancel(void);
//static bool FlexMotorEnable = false;//fww
//static unsigned int w_FlexAdjStep;//fww
static unsigned int nFlexMotorRunStateOld = STATE_FLEX_IDLE ;
//static unsigned char nFlexMotorInFlag, nFlexMotorInFlag1 ; //1: runin ; 0:run out or idle
//unsigned int FlexMotor_ResetTime;
unsigned char Accident_Happen_Flag, Accident_Happen_Flag1 ;  //1: accident ; 0: everything is ok
void FlexMotor_Initial_IO(void)
{
  GPIO_PinModeSet(FLEX_MOTOR_RESET_PORT, FLEX_MOTOR_RESET_BIT, FLEX_MOTOR_RESET_MODE, 0);
  GPIO_PinModeSet(FLEX_MOTOR_ENBL_PORT, FLEX_MOTOR_ENBL_BIT, FLEX_MOTOR_ENBL_MODE, 0);
  GPIO_PinModeSet(FLEX_MOTOR_PHASE_PORT, FLEX_MOTOR_PHASE_BIT, FLEX_MOTOR_PHASE_MODE, 0);
  GPIO_PinModeSet(FLEX_MOTOR_DECAY_PORT, FLEX_MOTOR_DECAY_BIT, FLEX_MOTOR_DECAY_MODE, 0);
  GPIO_PinModeSet(FLEX_MOTOR_CURRENT_PORT, FLEX_MOTOR_CURRENT_BIT, FLEX_MOTOR_CURRENT_MODE, 1);
  GPIO_PinModeSet(FLEX_MOTOR_FAULT_PORT, FLEX_MOTOR_FAULT_BIT, FLEX_MOTOR_FAULT_MODE, 1);
  
  TIMER_InitCC_TypeDef timerCCInit = FLEX_MOTOR_Timer_CCInit;
  
  //FLEX_MOTOR
  TIMER_InitCC(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL, &timerCCInit);
  TIMER_InitCC(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CUR_CHANNEL, &timerCCInit);
  
  //Hot_Roller
  TIMER_InitCC(ROLLER_MOTOR_TIMER, ROLLER_MOTOR_TIMER_CHANNEL, &timerCCInit);
  
  
  TIMER_TopSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_DEFAULT_TOP);
  
  //FLEX_MOTOR
  TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL, 0);
  
  //Hot_Roller
  TIMER_CompareBufSet(ROLLER_MOTOR_TIMER, ROLLER_MOTOR_TIMER_CHANNEL, 0);
  
  TIMER_Init_TypeDef timerInit = FLEX_MOTOR_Timer_Init;  

  TIMER_Init(FLEX_MOTOR_TIMER, &timerInit);          
  
  FLEX_MOTOR_TIMER->ROUTE |= (FLEX_MOTOR_ROUTE_EN | FLEX_MOTOR_ROUTE_CUR_EN | \
    ROLLER_MOTOR_ROUTE_EN | FLEX_MOTOR_ROUTE_LOCATION); 
  
  TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CUR_CHANNEL, FLEX_CURRENT_4A); //最大电流4A(80/131*3.3)
  /*
  TIMER_InitCC_TypeDef timerCCInit = FLEX_MOTOR_Timer_CCInit;

  TIMER_InitCC(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL, &timerCCInit);
  TIMER_InitCC(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CUR_CHANNEL, &timerCCInit);
  
  TIMER_TopSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_DEFAULT_TOP);
  
  TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL, 0);
  TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CUR_CHANNEL, 0);
  
  TIMER_Init_TypeDef timerInit = FLEX_MOTOR_Timer_Init;

  TIMER_Init(FLEX_MOTOR_TIMER, &timerInit);
  
  FLEX_MOTOR_TIMER->ROUTE |= (FLEX_MOTOR_ROUTE_EN | FLEX_MOTOR_ROUTE_LOCATION); 
  FLEX_MOTOR_TIMER->ROUTE |= (FLEX_MOTOR_ROUTE_CUR_EN | FLEX_MOTOR_ROUTE_LOCATION); 

  TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CUR_CHANNEL, FLEX_CURRENT_4A); //最大电流4A(80/131*3.3)
  */
}
/*
void FlexMotor_Data_Init(void)
{
  nFlexMotorInFlag = 0 ;
  nFlexMotorInFlag1 = 0 ;
}
*/
void FlexMotor_10ms_Int(void)
{
  bFalg = true; 
}
void FlexkMotor_Set_Pwm_Data(unsigned long ulDuty)
{
  unsigned int duty ;
  if(ulDuty == 0)
  {
    TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL, ulDuty);
    return;
  }
  duty = TIMER_CompareBufGet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL);
  if(ulDuty == duty)
  {
   if(FlexMotor_Get_Fault() == FLEX_MOTOR_NORMAL) return;
    FlexMotor_Reset();
    __no_operation();
    __no_operation();
    FlexMotor_Reset_Cancel();
   return; 
  }
  if(!bFalg) return;
  bFalg = false;
  if(duty < ulDuty)
  {
    if(duty < FLEX_MOTOR_DEFAULT_TOP/2)
      duty = FLEX_MOTOR_DEFAULT_TOP/2;
    else duty++;
  }
  TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL, duty);
}

void FlexkMotor_Set_Pwm(unsigned long ulDuty)
{
    TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL, ulDuty);
}

int FlexPower_On(unsigned char speed)
{
  FlexkMotor_Set_Pwm_Data(speed);
  /*
  int val = 0;
  Power_On();
  if(GPIO_PinOutGet(FLEX_MOTOR_ENBL_PORT,FLEX_MOTOR_ENBL_BIT))
  { //端口已经high
    if(FlexMotor_Get_Fault() == FLEX_MOTOR_FAIL)
    {
      FlexMotor_Reset();
      __no_operation();
      __no_operation();
      FlexMotor_Reset_Cancel();
      __no_operation();
      if(FlexMotor_Get_Fault() != FLEX_MOTOR_FAIL) val = 0;
    }
  }
  else
  {
   GPIO_PinOutSet(FLEX_MOTOR_ENBL_PORT, FLEX_MOTOR_ENBL_BIT);
   val =  1;
  }
  */
  return 0;
}

void FlexPower_Off(void)
{
  //GPIO_PinOutClear(FLEX_MOTOR_ENBL_PORT, FLEX_MOTOR_ENBL_BIT);
  FlexkMotor_Set_Pwm_Data(0);
}

unsigned int FlexPower_Get(void)
{
  unsigned long  ulDuty;
  ulDuty = TIMER_CompareBufGet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL);
  if(ulDuty > 0) return FLEX_POWER_ON;
  else return FLEX_POWER_OFF;
}

void FlexMotor_Out(void)
{
 // Power_On();
  GPIO_PinOutSet(FLEX_MOTOR_RESET_PORT, FLEX_MOTOR_RESET_BIT);
  GPIO_PinOutClear(FLEX_MOTOR_PHASE_PORT, FLEX_MOTOR_PHASE_BIT);
  GPIO_PinOutClear(FLEX_MOTOR_DECAY_PORT, FLEX_MOTOR_DECAY_BIT);
  //140531
  //nFlexMotorInFlag = 0 ;
  
}

void FlexMotor_In(void)
{
 // Power_On();
  GPIO_PinOutClear(FLEX_MOTOR_DECAY_PORT, FLEX_MOTOR_DECAY_BIT);
  GPIO_PinOutSet(FLEX_MOTOR_RESET_PORT, FLEX_MOTOR_RESET_BIT);
  GPIO_PinOutSet(FLEX_MOTOR_PHASE_PORT, FLEX_MOTOR_PHASE_BIT);
  //140531
  //nFlexMotorInFlag = 1 ;
  //140603
  //nFlexMotorInFlag1 = 1 ;
}

void FlexMotor_Break(void)
{
  GPIO_PinOutClear(FLEX_MOTOR_DECAY_PORT, FLEX_MOTOR_DECAY_BIT);
  //140531
  //nFlexMotorInFlag = 0 ;
 // GPIO_PinOutClear(FLEX_MOTOR_ENBL_PORT, FLEX_MOTOR_ENBL_BIT);
  FlexkMotor_Set_Pwm_Data(0);
}

void FlexMotor_Reset(void)
{
  GPIO_PinOutClear(FLEX_MOTOR_RESET_PORT, FLEX_MOTOR_RESET_BIT);
}

void FlexMotor_Reset_Cancel(void)
{
  GPIO_PinOutSet(FLEX_MOTOR_RESET_PORT, FLEX_MOTOR_RESET_BIT);
}

int FlexMotor_Get_Fault(void)
{
  if(GPIO_PinInGet(FLEX_MOTOR_FAULT_PORT, FLEX_MOTOR_FAULT_BIT))
    return FLEX_MOTOR_NORMAL;
  return FLEX_MOTOR_FAIL;
}

//FlexPad motor control function
//正常运行中返回0，遇到行程开关或其它原因停止返回1

unsigned char FlexMotor_Control(unsigned char nFinalFlexPadMotorState,unsigned char speed,unsigned char current)
{
  static unsigned int position = 0;
  unsigned char nRetVal ;
  
  bool bPowerFlag;
  nRetVal = FLEX_RUN;
  //current = FLEX_CURRENT_2A;  //temp test
  //140603
  if(nFlexMotorRunStateOld != nFinalFlexPadMotorState)
  {
    nFlexMotorRunStateOld = nFinalFlexPadMotorState ;
    position = 0 ;
  }
  
  switch(nFinalFlexPadMotorState)
  {
   case STATE_RUN_FLEX_IN:
    if(Input_GetFlexInSwitch() == REACH_FLEX_LIMIT || position == 1)
    {
      position = 1;
      bPowerFlag = FALSE;
      nRetVal = FLEX_STOP_AT_IN ;
      FlexMotor_Break();
      break;
    }
    
    if((Input_GetFlexFootSwitch() == FOOT_SWITCH_ON) )
    {  //碰到脚了
      bPowerFlag = FALSE;
      nRetVal = FLEX_STOP_AT_FOOT ;
      FlexMotor_Break();
      break;
    }
    //140531
    Clear_Accident_flag() ;
    position = 0;
    bPowerFlag = TRUE;
    FlexMotor_In();
    
    break ;
   case STATE_RUN_FLEX_RESET: //强制收回
    if(Input_GetFlexInSwitch() == REACH_FLEX_LIMIT || position == 1)
    {
      position = 1;
      bPowerFlag = FALSE;
      nRetVal = FLEX_STOP_AT_IN ;
      FlexMotor_Break();
      break;
    }
    position = 0;
    bPowerFlag = TRUE;
    FlexMotor_In();
    
    break ;
    
   case STATE_RUN_FLEX_OUT:  //上行
    if(Input_GetFlexOutSwitch() == REACH_FLEX_LIMIT || position == 2)
    {
      //140531
      Clear_Accident_flag() ;
      
      position = 2;
      bPowerFlag = FALSE;
      FlexMotor_Break();
      nRetVal = FLEX_STOP_AT_OUT ;
      break;
    }
    if(Input_GetFlexAngleSwitch() == LEGANGLE_SWITCH_ON)
    { //小于15度
      bPowerFlag = FALSE;
      FlexMotor_Break();
      nRetVal = FLEX_STOP_AT_ANGLE ;
      break;
    }
    if(Input_GetFlexGroundSwitch() == LEGGROUND_SWITCH_ON)
    { //碰到地面了
      bPowerFlag = FALSE;
      FlexMotor_Break();
      nRetVal = FLEX_STOP_AT_GROUND;
      break;
    }
    if(Input_GetFlexFootSwitch() == FOOT_SWITCH_OFF)
    {  //碰不到脚了
      bPowerFlag = FALSE;
      nRetVal = FLEX_STOP_AT_FOOT_LEAVE ;
      FlexMotor_Break();
      break;
    }
    //140603
    //Timer_Counter_Clear(FLEX_TIME_CHANNEL) ;
    position = 0;
    FlexMotor_Out();
    bPowerFlag = TRUE;
    break ;
   case STATE_RUN_FLEX_MANUAL_OUT:  //小腿手动伸出
    if(Input_GetFlexOutSwitch() == REACH_FLEX_LIMIT || position == 2)
    {
      nRetVal = FLEX_STOP_AT_OUT ;
      Clear_Accident_flag() ;
      
      position = 2;
      bPowerFlag = FALSE;
      FlexMotor_Break();
      
      break;
    }
    if(Input_GetFlexGroundSwitch() == LEGGROUND_SWITCH_ON)
    { //碰到地面了
      bPowerFlag = FALSE;
      FlexMotor_Break();
      nRetVal = FLEX_STOP_AT_GROUND;
      break;
    }
    if(Input_GetFlexAngleSwitch() == LEGANGLE_SWITCH_ON)
    { //小于15度
      bPowerFlag = FALSE;
      FlexMotor_Break();
      nRetVal = FLEX_STOP_AT_ANGLE ;
      break;
    }
    Timer_Counter_Clear(FLEX_TIME_CHANNEL) ;
    position = 0;
    FlexMotor_Out();
    bPowerFlag = TRUE;
    break ;  
   case STATE_RUN_FLEX_TEST_OUT:  //小腿伸出直到碰到行程开关，测试用
   // if(Input_GetFlexOutSwitch() == REACH_FLEX_LIMIT || position == 2)
   // {
   //   position = 2;
   //   bPowerFlag = FALSE;
   //   FlexMotor_Break();
   //   nRetVal = FLEX_STOP_AT_OUT ;
   //   break;
   // }
    position = 0;
    FlexMotor_Out();
    bPowerFlag = TRUE;
    break ;    
   case STATE_FLEX_IDLE:
    //140603
    Timer_Counter_Clear(FLEX_TIME_CHANNEL) ;
    //140526
    position = 0;
    bPowerFlag = FALSE;
    FlexMotor_Break();
    nRetVal = FLEX_STOP_AT_IDLE;
    break ;
   default://异常处理
    break ;
  }
  //电源部分的处理
  if(bPowerFlag == TRUE)
  {
    FlexPower_On(speed);
    //TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CUR_CHANNEL, current); 
  }
  else
  {
    FlexPower_Off();
  }
  return nRetVal ;
}

unsigned char VibrateRightMotorControl(unsigned char nFinalFlexPadMotorState,unsigned char speed)
{
  static unsigned int position = 0;
  unsigned char nRetVal ;
  
  bool bPowerFlag;
  nRetVal = FLEX_RUN;
  //current = FLEX_CURRENT_2A;  //temp test
  //140603
  if(nFlexMotorRunStateOld != nFinalFlexPadMotorState)
  {
    nFlexMotorRunStateOld = nFinalFlexPadMotorState ;
    position = 0 ;
  }
  
  switch(nFinalFlexPadMotorState)
  {
   case STATE_RUN_FLEX_IN:
    if(Input_GetFlexInSwitch() == REACH_FLEX_LIMIT || position == 1)
    {
      position = 1;
      bPowerFlag = FALSE;
      nRetVal = FLEX_STOP_AT_IN ;
      FlexMotor_Break();
      break;
    }
    
    if((Input_GetFlexFootSwitch() == FOOT_SWITCH_ON) )
    {  //碰到脚了
      bPowerFlag = FALSE;
      nRetVal = FLEX_STOP_AT_FOOT ;
      FlexMotor_Break();
      break;
    }
    //140531
    Clear_Accident_flag() ;
    position = 0;
    bPowerFlag = TRUE;
    FlexMotor_In();
    
    break ;
   case STATE_RUN_FLEX_RESET: //强制收回
    if(Input_GetFlexInSwitch() == REACH_FLEX_LIMIT || position == 1)
    {
      position = 1;
      bPowerFlag = FALSE;
      nRetVal = FLEX_STOP_AT_IN ;
      FlexMotor_Break();
      break;
    }
    position = 0;
    bPowerFlag = TRUE;
    FlexMotor_In();
    
    break ;
    
   case STATE_RUN_FLEX_OUT:  //上行
    if(Input_GetFlexOutSwitch() == REACH_FLEX_LIMIT || position == 2)
    {
      //140531
      Clear_Accident_flag() ;
      
      position = 2;
      bPowerFlag = FALSE;
      FlexMotor_Break();
      nRetVal = FLEX_STOP_AT_OUT ;
      break;
    }
    if(Input_GetFlexAngleSwitch() == LEGANGLE_SWITCH_ON)
    { //小于15度
      bPowerFlag = FALSE;
      FlexMotor_Break();
      nRetVal = FLEX_STOP_AT_ANGLE ;
      break;
    }
    if(Input_GetFlexGroundSwitch() == LEGGROUND_SWITCH_ON)
    { //碰到地面了
      bPowerFlag = FALSE;
      FlexMotor_Break();
      nRetVal = FLEX_STOP_AT_GROUND;
      break;
    }
    if(Input_GetFlexFootSwitch() == FOOT_SWITCH_OFF)
    {  //碰不到脚了
      bPowerFlag = FALSE;
      nRetVal = FLEX_STOP_AT_FOOT_LEAVE ;
      FlexMotor_Break();
      break;
    }
    //140603
    //Timer_Counter_Clear(FLEX_TIME_CHANNEL) ;
    position = 0;
    FlexMotor_Out();
    bPowerFlag = TRUE;
    break ;
   case STATE_RUN_FLEX_MANUAL_OUT:  //小腿手动伸出
    if(Input_GetFlexOutSwitch() == REACH_FLEX_LIMIT || position == 2)
    {
      nRetVal = FLEX_STOP_AT_OUT ;
      Clear_Accident_flag() ;
      
      position = 2;
      bPowerFlag = FALSE;
      FlexMotor_Break();
      
      break;
    }
    if(Input_GetFlexGroundSwitch() == LEGGROUND_SWITCH_ON)
    { //碰到地面了
      bPowerFlag = FALSE;
      FlexMotor_Break();
      nRetVal = FLEX_STOP_AT_GROUND;
      break;
    }
    if(Input_GetFlexAngleSwitch() == LEGANGLE_SWITCH_ON)
    { //小于15度
      bPowerFlag = FALSE;
      FlexMotor_Break();
      nRetVal = FLEX_STOP_AT_ANGLE ;
      break;
    }
    Timer_Counter_Clear(FLEX_TIME_CHANNEL) ;
    position = 0;
    FlexMotor_Out();
    bPowerFlag = TRUE;
    break ;  
   case STATE_RUN_FLEX_TEST_OUT:  //小腿伸出直到碰到行程开关，测试用
   // if(Input_GetFlexOutSwitch() == REACH_FLEX_LIMIT || position == 2)
   // {
   //   position = 2;
   //   bPowerFlag = FALSE;
   //   FlexMotor_Break();
   //   nRetVal = FLEX_STOP_AT_OUT ;
   //   break;
   // }
    position = 0;
    FlexMotor_Out();
    bPowerFlag = TRUE;
    break ;    
   case STATE_FLEX_IDLE:
    //140603
    Timer_Counter_Clear(FLEX_TIME_CHANNEL) ;
    //140526
    position = 0;
    bPowerFlag = FALSE;
    FlexMotor_Break();
    nRetVal = FLEX_STOP_AT_IDLE;
    break ;
   default://异常处理
    break ;
  }
  //电源部分的处理
  if(bPowerFlag == TRUE)
  {
    FlexkMotor_Set_Pwm(speed);
    //TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CUR_CHANNEL, speed); 
  }
  else
  {
    FlexPower_Off();
  }
  return nRetVal ;
}
//自动找脚程序 
/*******************fww*********************
void FlexMotorFollowingFood(void)
{
  //141125
  static uint8_t ucPowerOff = FALSE;
  //
  if(!FlexMotorEnable)
  {
    //141125
    ucPowerOff = FALSE;
    //
    return;
  }
  if(LegPower_Get())
  {
    //141125
    ucPowerOff = FALSE;
    //
    return;  //小腿上下电动缸还在运行中
  }
  //141125
  if(ucPowerOff == FALSE)
  {
    Power_3V3_Off();
    ucPowerOff = TRUE;
    return;
  }
  else if(ucPowerOff == TRUE)
  {
    Power_3V3_On();
  }
  
  //
  switch(w_FlexAdjStep)
  {
   case 0:
    if(Input_GetFlexFootSwitch() == FOOT_SWITCH_ON)
    {  
      w_FlexAdjStep = 1;
    }
    else
    {  
      w_FlexAdjStep = 2;
    } 
    break;
   case 1://先向外走
    if(FlexMotor_Control(STATE_RUN_FLEX_OUT,FLEX_SPEED_FAST,FLEX_CURRENT_3A))
    {
      w_FlexAdjStep++;
    }
    break;
   case 2://再向里走
    if(FlexMotor_Control(STATE_RUN_FLEX_IN,FLEX_SPEED_FAST,FLEX_CURRENT_3A))
    {
      FlexMotorEnable = false;
    }
    break;
  }
}
***********************fww*************************/
/***********************fww************************
//1 执行自动跟脚程序
int FlexMotorGetEnable(void)
{
  return(FlexMotorEnable);
}
void FlexMotorSetEnable(void)
{
  FlexMotorEnable = true;
  w_FlexAdjStep = 0; 
}
void FlexMotorSetDisable(void)
{
  FlexMotorEnable = false;
  w_FlexAdjStep = 0; 
}
************************fww************************/
/*
*@brief   :
*@param   :
*@retval  :
******************************************/
/*void FlexMotor_100ms_Int(void)
{
  Flex_100ms_Flag = true;
}*/
/*
*@brief   :
*@param   :
*@retval  :
******************************************/
void Clear_Accident_flag(void)
{
  Accident_Happen_Flag = 0 ;
}
/*
*@brief   :
*@param   :
*@retval  :
******************************************/
void Clear_Accident1_flag(void)
{
  Accident_Happen_Flag1 = 0 ;
}
/*
*@brief   :
*@param   :
*@retval  :
******************************************/
unsigned char Get_CurAccident_flag(void)
{
  return Accident_Happen_Flag ;
}