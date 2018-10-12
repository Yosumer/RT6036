
/*
* Function: Input signal process
*/
/************************************************************************************/

#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "efm32_types.h"
#include "efm32_def.h"
#include "LegMotor.h"
#include "Valve.h"
#include "WalkMotor.h"
#include "MassageStatus.h"
#include "memory.h"
#include "input.h"
#include "BackPad.h"

BITS InputData1;
#define bLegStretchOutSwitch		InputData1.bD2   
#define bLegStretchInSwitch		InputData1.bD3   
#define bLegStretchGroundSwitch  	        InputData1.bD0// 8600此处是反的  
#define bLegStretchAngleSwitch		InputData1.bD1//  
#define bKneadMaxSwitch 	InputData1.bD4//电动小腿延伸到最前面位置
#define bKneadMidSwitch	        InputData1.bD5//电动小腿退到最后面位置   
#define bKneadMinSwitch		InputData1.bD6
#define bKneckCheck 		InputData1.bD7

BITS InputData2;
#define bZeroPadUpSwitch	InputData2.bD2   
#define bZeroPadDownSwitch	InputData2.bD3   
#define bLegDownSwitch 	        InputData2.bD0   
#define bLegUpSwitch		InputData2.bD1   
#define bWalkUpSwitch           InputData2.bD4   
#define bWalkDownSwitch 	InputData2.bD5   
#define bBackUpSwitch		InputData2.bD6
#define bBackDownSwitch		InputData2.bD7



#define Data1Offset         0
#define Data2Offset         1
#define Data3Offset         2

typedef  struct
{
        unsigned char timer_H;
        unsigned char timer_L;
        unsigned char flag;
}INPUT_ST;
bool bWalkChange;
INPUT_ST st_FlexGround,st_StretchOut,st_StretchIn,st_Foot,st_Angle,st_Vout;
INPUT_ST st_ZeroUp,st_ZeroDown,st_BackUp,st_BackDown,st_LegUp,st_LegDown,st_WalkUp,st_WalkDown;
bool b5msFlag;
BYTE nPulseHigh;
UINT16 WalkCount = 0xffff;  // 0xffff indicate no initial, at top =0 at bottom = maximal, up:dec down:add
BYTE by_KneadPosition = KNEAD_WIDTH_UNKNOWN;
volatile unsigned short nCurWalkLocate;
unsigned short nCounterCurWalkLocate;
volatile unsigned int nCurBackLocate;

/**************************************************************************//**
 * @brief GPIO_ODD_IRQHandler
 * Interrupt Service Routine Odd GPIO Interrupt Line
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{ 
  /*
  if(GPIO_IntGet()&(1<<INPUT_KNEAD_MID_BIT))
  {
    by_KneadPosition = KNEAD_WIDTH_MED;    
    GPIO_IntClear(1<<INPUT_KNEAD_MID_BIT);
  }
  */
}
//
//get backmotor's pulse
#define BACK_PULSE_CHECK_TIME 3
unsigned char BackPulseCountHigh = 0;
unsigned char BackPulseCountLow = 0;
unsigned char BackPulseStep = 0;
void Input_Back_Pulse1MS(void)
{
  bool flag = GPIO_PinInGet(INPUT_BACK_PULSE_PORT, INPUT_BACK_PULSE_BIT);
  if(flag == 0){
    //低电平为开关有效
    BackPulseCountLow++;
  }else{
    BackPulseCountHigh++;
  }
  //从高到底再到高为1个有效数
  switch(BackPulseStep){
  case 0:
    if((flag == 1) && (BackPulseCountHigh > BACK_PULSE_CHECK_TIME)){
      BackPulseCountLow = 0;
      BackPulseStep++;
    }
    break;
  case 1:
    //从高到底再到高为1个有效数
    if((flag == 0) && (BackPulseCountLow > BACK_PULSE_CHECK_TIME)){
      BackPulseCountHigh = 0;
      BackPulseStep = 0;
      //计数
      if(GPIO_PinOutGet(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT) != 0)
      {
        if(nCurBackLocate > 0 )nCurBackLocate-- ;
      }else{
        nCurBackLocate++ ;
      }
    }
    break;
  }
}
//
void GPIO_EVEN_IRQHandler(void)
{
  //if(GPIO_IntGet()&(1<<INPUT_WALK_PULSE_BIT))
  {
    bWalkChange = TRUE;
    GPIO_IntClear(1<<INPUT_WALK_PULSE_BIT);
    if(WalkRelay_Get() != 0)
    {
      if(nCurWalkLocate > 0 )
      {
          nCurWalkLocate-- ;
         // nWalkOverTime = 0;
      }
      if(nCounterCurWalkLocate < 1000 )
      {
        nCounterCurWalkLocate++ ;
      }
    }
    else
    {
      if(nCurWalkLocate < 1000)
      {
          nCurWalkLocate++ ;
         // nWalkOverTime = 0;
      }
      if(nCounterCurWalkLocate > 0 )
      {
        nCounterCurWalkLocate--;
      }
    }
  }
}

void Input_SetWalkMotorPosition(unsigned short locate)
{
  nCurWalkLocate = (unsigned short)locate;
  nCurWalkLocate *= 2;
}
unsigned short Input_GetWalkMotorPosition(void)
{
  return(nCurWalkLocate/2);
}

//相反方向
void Input_SetCounterWalkMotorPosition(unsigned short locate) //8600
{
  nCounterCurWalkLocate = (unsigned short)locate;
  nCounterCurWalkLocate *= 2;
}
unsigned short Input_GetCounterWalkMotorPosition(void) //8600
{
  return(nCounterCurWalkLocate/2);
}

void Input_Initial_IO(void)
{
 // GPIO_PinModeSet(INPUT_KNEAD_MAX_PORT, INPUT_KNEAD_MAX_BIT, INPUT_KNEAD_MAX_MODE, 1);
 // GPIO_PinModeSet(INPUT_KNEAD_MID_PORT, INPUT_KNEAD_MID_BIT, INPUT_KNEAD_MID_MODE, 1);
 // GPIO_PinModeSet(INPUT_KNEAD_MIN_PORT, INPUT_KNEAD_MIN_BIT, INPUT_KNEAD_MIN_MODE, 1);
  
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
    
  //GPIO_IntConfig(INPUT_KNEAD_MAX_PORT, INPUT_KNEAD_MAX_BIT, false, true, true);
  //GPIO_IntConfig(INPUT_KNEAD_MID_PORT, INPUT_KNEAD_MID_BIT, false, true, true);
  //GPIO_IntConfig(INPUT_KNEAD_MIN_PORT, INPUT_KNEAD_MIN_BIT, false, true, true);
  
  GPIO_PinModeSet(INPUT_BACK_PULSE_PORT, INPUT_BACK_PULSE_BIT, INPUT_BACK_PULSE_MODE, 1);
  GPIO_PinModeSet(INPUT_LEGSIDE_SENSOR_PORT, INPUT_LEGSIDE_SENSOR_BIT, INPUT_LEGSIDE_SENSOR_MODE, 1);
  GPIO_PinModeSet(INPUT_LEG_FOOT_PORT, INPUT_LEG_FOOT_BIT, INPUT_LEG_FOOT_MODE, 1);
  GPIO_PinModeSet(INPUT_WALK_PULSE_PORT, INPUT_WALK_PULSE_BIT, INPUT_WALK_PULSE_MODE, 1);
  GPIO_IntConfig(INPUT_WALK_PULSE_PORT, INPUT_WALK_PULSE_BIT, false, true, true);
//  GPIO_PinModeSet(INPUT_VOUT_PORT, INPUT_VOUT_BIT, INPUT_VOUT_MODE, 1);
  
//  GPIO_PinModeSet(INPUT_BACK_UP_PORT, INPUT_BACK_UP_BIT, INPUT_BACK_UP_MODE, 1); 
//  GPIO_PinModeSet(INPUT_BACK_DOWN_PORT, INPUT_BACK_DOWN_BIT, INPUT_BACK_DOWN_MODE, 1); 
  
//  GPIO_PinModeSet(INPUT_LEG_UP_PORT, INPUT_LEG_UP_BIT, INPUT_LEG_UP_MODE, 1); 
//  GPIO_PinModeSet(INPUT_LEG_DOWN_PORT, INPUT_LEG_DOWN_BIT, INPUT_LEG_DOWN_MODE, 1); 
  
//  GPIO_PinModeSet(INPUT_WALK_UP_PORT, INPUT_WALK_UP_BIT, INPUT_WALK_UP_MODE, 1); 
//  GPIO_PinModeSet(INPUT_WALK_DOWN_PORT, INPUT_WALK_DOWN_BIT, INPUT_WALK_DOWN_MODE, 1); 
  
//  GPIO_PinModeSet(INPUT_ZERO_UP_PORT, INPUT_ZERO_UP_BIT, INPUT_ZERO_UP_MODE, 1); 
//  GPIO_PinModeSet(INPUT_ZERO_DOWN_PORT, INPUT_ZERO_DOWN_BIT, INPUT_ZERO_DOWN_MODE, 1); 
  //  GPIO_PinModeSet(INPUT_WAVE_PORT, INPUT_WAVE_BIT, INPUT_WAVE_MODE, 1); 
  
   GPIO_PinModeSet(gpioPortE, 4, gpioModeInput, 1);
   GPIO_PinModeSet(gpioPortE, 5, gpioModeInput, 1);
   GPIO_PinModeSet(gpioPortE, 6, gpioModeInput, 1);
   GPIO_PinModeSet(gpioPortE, 7, gpioModeInput, 1);
  
}
void Input_5ms_Int(void)
{
 	b5msFlag = 1;
}

void Input_High(INPUT_ST* p)
{
  p->timer_L = 0;
  p->timer_H ++; 
  if((p->timer_H) >= 2)
  {
    p->timer_H = 2;
    p->flag = 1;
  }
}
void Input_Low(INPUT_ST* p)
{
  p->timer_H = 0;
  p->timer_L ++; 
  if((p->timer_L) >= 2)
  {
    p->timer_L = 2;
    p->flag = 0;
  }
}

void Input_Proce(void)
{
  if(!b5msFlag) return;		
  b5msFlag = 0;
  InputData1.nByte = *(pInputData + Data1Offset);
  InputData2.nByte = *(pInputData + Data2Offset);
  //InputData3.nByte = *(pInputData + Data3Offset);
   
  if(!bKneadMaxSwitch)
  {
    by_KneadPosition = KNEAD_WIDTH_MAX;    
  }
  if(!bKneadMidSwitch)
  {
    by_KneadPosition = KNEAD_WIDTH_MED;    
  }
  if(!bKneadMinSwitch)
  {
    by_KneadPosition = KNEAD_WIDTH_MIN;    
  }
  bLegStretchGroundSwitch?Input_High(&st_FlexGround):Input_Low(&st_FlexGround);
  bLegUpSwitch ? Input_High(&st_LegUp):Input_Low(&st_LegUp);
  bLegDownSwitch ? Input_High(&st_LegDown):Input_Low(&st_LegDown);
  bBackUpSwitch ? Input_High(&st_BackUp):Input_Low(&st_BackUp);
  bBackDownSwitch ? Input_High(&st_BackDown):Input_Low(&st_BackDown);			
  bZeroPadUpSwitch ? Input_High(&st_ZeroUp):Input_Low(&st_ZeroUp);
  bZeroPadDownSwitch ? Input_High(&st_ZeroDown):Input_Low(&st_ZeroDown);
  bWalkUpSwitch ? Input_High(&st_WalkUp):Input_Low(&st_WalkUp);
  if(!st_WalkUp.flag)
  {
  //  nCurWalkLocate = 424*2;
  }
  bWalkDownSwitch ? Input_High(&st_WalkDown):Input_Low(&st_WalkDown);
 
  bLegStretchOutSwitch ? Input_High(&st_StretchOut):Input_Low(&st_StretchOut);
  bLegStretchInSwitch ? Input_High(&st_StretchIn):Input_Low(&st_StretchIn);
  
  GPIO_PinInGet(INPUT_LEG_FOOT_PORT, INPUT_LEG_FOOT_BIT)?Input_High(&st_Foot):Input_Low(&st_Foot);
  bLegStretchAngleSwitch? Input_High(&st_Angle):Input_Low(&st_Angle);
  //GPIO_PinInGet(INPUT_KNEAD_MAX_PORT, INPUT_KNEAD_MAX_BIT) ? Input_High(&st_KneadMax):Input_Low(&st_KneadMax);	
  //GPIO_PinInGet(INPUT_KNEAD_MAX_PORT, INPUT_KNEAD_MID_BIT) ? Input_High(&st_KneadMid):Input_Low(&st_KneadMid);
  //GPIO_PinInGet(INPUT_KNEAD_MAX_PORT, INPUT_KNEAD_MIN_BIT) ? Input_High(&st_KneadMin):Input_Low(&st_KneadMin);			
  bKneckCheck ? Input_High(&st_Vout):Input_Low(&st_Vout);			
}

//return 0 or 1
unsigned int Input_GetKneadMax(void)
{
  return(bKneadMaxSwitch);
}
//return 0 or 1
unsigned int Input_GetKneadMid(void)
{
	return(bKneadMidSwitch);
}
//return 0 or 1
unsigned int Input_GetKneadMin(void)
{
    return(bKneadMinSwitch);
}

unsigned int Input_GetBackUpSwitch(void)
{
  return(st_BackUp.flag);
}                     
unsigned int Input_GetBackDownSwitch(void)
{
  return(st_BackDown.flag);
}                     
unsigned int Input_GetLegUpSwitch(void)
{
  return(st_LegUp.flag);
}                     
unsigned int Input_GetLegDownSwitch(void)
{
  return(st_LegDown.flag);
}

unsigned int Input_GetSlideForwardSwitch(void)
{
  bool enable = ReadEEByte(USER_DATA_BASE + SLIDE_MOTOR_ENABLE_ADDRESS);
  if(enable)
     return(st_ZeroDown.flag);
  else
    return(1);
}
unsigned int Input_GetSlideBackwardSwitch(void)
{
  bool enable = ReadEEByte(USER_DATA_BASE + SLIDE_MOTOR_ENABLE_ADDRESS);
  if(enable)
   return(st_ZeroUp.flag);
  else return 1;
}

unsigned int Input_GetKneadPosition(void)
{
  return (unsigned int)by_KneadPosition;
}
    
unsigned int Input_GetVout(void)
{
  return(st_Vout.flag);
}   

unsigned int Input_GetWalkPulse(void)
{
  return(GPIO_PinInGet(INPUT_WALK_PULSE_PORT, INPUT_WALK_PULSE_BIT));
}                           

unsigned int Input_GetFlexGroundSwitch(void)
{
  return(st_FlexGround.flag);
}

unsigned int Input_GetMp3Status(void)
{
  return 1;
}
unsigned int Input_PowerCheck(void)
{
  return 1;
}

bool Input_GetWalkChange(void)
{
  return(bWalkChange);
}

void Input_ClearWalkChange(void)
{
  bWalkChange = 0;
}

unsigned int Input_GetWalkPosition(void)
{
  if(Input_GetWalkDownSwitch() == REACH_WALK_LIMIT)
    return WALK_MOTOR_AT_BOTTOM;
  if(Input_GetWalkUpSwitch() == REACH_WALK_LIMIT)
    return WALK_MOTOR_AT_TOP;
  else
    return WALK_MOTOR_AT_MID;
}
//获取靠背电动缸的脉冲计数位置
unsigned int Input_GetBackPosition(void)
{
  return nCurBackLocate;
}
void Input_SetBackMotorPosition(unsigned int Position)
{
  nCurBackLocate = Position;
}
//最前面位置
unsigned int Input_GetFlexOutSwitch(void)
{
  return(st_StretchOut.flag);
} 
//最后面位置
unsigned int Input_GetFlexInSwitch(void)
{
  return(st_StretchIn.flag);
}                     

unsigned int Input_GetFlexFootSwitch(void)
{
  return(st_Foot.flag);
} 

unsigned int Input_GetFlexAngleSwitch(void)
{
  return(st_Angle.flag);
} 

unsigned int Input_GetWalkUpSwitch(void)
{
  return(st_WalkUp.flag);
}
unsigned int Input_GetWalkDownSwitch(void)
{
  return(st_WalkDown.flag);
}

/*
*@brief :Get The signal of the legside's sensor
*@param :
*@retval:
***************************/
unsigned char Sensor_Check(void)
{
  static unsigned char nReachTimes;
  unsigned char nSenseReach;
  //Value = GPIO_PinInGet(INPUT_LEGSIDE_SENSOR_PORT,INPUT_LEGSIDE_SENSOR_BIT);
  //
  if(GPIO_PinInGet(INPUT_LEGSIDE_SENSOR_PORT,INPUT_LEGSIDE_SENSOR_BIT))//get the signal
  {
    nReachTimes = 0;
    nSenseReach = FALSE;
  }
  else
  {
    nReachTimes++;
    if(nReachTimes >= 3)
    {
      nReachTimes = 0;
      nSenseReach = TRUE;
    }
  }
  //nSenseReach = 
  return nSenseReach;
}