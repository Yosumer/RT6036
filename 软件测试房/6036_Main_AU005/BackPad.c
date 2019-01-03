#include "EFM32_def.h"
#include "EFM32_types.h"
#include "efm32.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "input.h"
#include "MassageStatus.h"
#include "valve.h"
#include "ADC_Single.h"
#include "SlideMotor.h"
#include "LegMotor.h"

#include "BackPad.h"
//靠背电动缸位置 最高：0 最低：最大位置
static UINT32 w_Position;
static bool bBackMotorFlag;

static unsigned int motorStatus = MOTOR_STOP_BREAK;
static volatile unsigned int motorBreakTime;
bool slow_break_10MS_Int = false;
bool speedUp_10MS_Int = false;
bool breakDown_10MS_Int = false; 
bool set_Pwm_10MS_Int = false;
bool isRocking = false;
int speed = 0;
int currentSpeed = BACK_MOTOR_DEFAULT_TOP;
unsigned char currentBackPadMotorState = STATE_BACK_IDLE;

void BackMotor_Initial_IO(void)
{
  GPIO_PinModeSet(BACK_MOTOR_RESET_PORT, BACK_MOTOR_RESET_BIT, BACK_MOTOR_RESET_MODE, 0);
  GPIO_PinModeSet(BACK_MOTOR_ENBL_PORT, BACK_MOTOR_ENBL_BIT, BACK_MOTOR_ENBL_MODE, 0);
  GPIO_PinModeSet(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT, BACK_MOTOR_PHASE_MODE, 0);
  GPIO_PinModeSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT, BACK_MOTOR_DECAY_MODE, 0);
  GPIO_PinModeSet(BACK_MOTOR_FAULT_PORT, BACK_MOTOR_FAULT_BIT, BACK_MOTOR_FAULT_MODE, 1);
  //Configure TIMER1 CC0 LOC1 PE10
  TIMER_InitCC_TypeDef timerCCInit = BACK_MOTOR_Timer_CCInit;

  TIMER_InitCC(BACK_MOTOR_TIMER, BACK_MOTOR_TIMER_CHANNEL, &timerCCInit);
  
  TIMER_InitCC(SLIDE_MOTOR_TIMER, SLIDE_MOTOR_CHANNEL, &timerCCInit);
  
  TIMER_InitCC(LEG_MOTOR_TIMER, LEG_MOTOR_CHANNEL, &timerCCInit);
  
  TIMER_TopSet(BACK_MOTOR_TIMER, BACK_MOTOR_DEFAULT_TOP);
  
  TIMER_CompareBufSet(BACK_MOTOR_TIMER, BACK_MOTOR_TIMER_CHANNEL, 0);
  
  //init TIMER1
  TIMER_Init_TypeDef timerInit = BACK_MOTOR_Timer_Init;

  TIMER_Init(BACK_MOTOR_TIMER, &timerInit);
  
  BACK_MOTOR_TIMER->ROUTE |= (BACK_MOTOR_ROUTE_EN |LEG_MOTOR_ROUTE_EN |SLIDE_MOTOR_ROUTE_EN | TIMER_ROUTE_LOCATION_LOC1); 
}

int BackMotor_GetPower(void)
{
	if(currentSpeed == 0)
	{
		return BACK_MOTOR_POWER_OFF;
	}
	else
	{
		return BACK_MOTOR_POWER_ON;
	}
	/*
	if(GPIO_PinOutGet(BACK_MOTOR_ENBL_PORT,BACK_MOTOR_ENBL_BIT))
	{
		return BACK_MOTOR_POWER_ON; 
	}
	return BACK_MOTOR_POWER_OFF; 
	*/
}
int BackMotor_GetDirection(void)
{
  if(GPIO_PinOutGet(BACK_MOTOR_PHASE_PORT,BACK_MOTOR_PHASE_BIT))
  {
   return BACK_MOTOR_GO_UP; 
  }
  return BACK_MOTOR_GO_DOWN; 
}

int BackPower_On(void)
{
  int val = 0;
  //Power_On();
  if(GPIO_PinOutGet(BACK_MOTOR_ENBL_PORT,BACK_MOTOR_ENBL_BIT))
  { //端口已经high
    if(BackMotor_Get_Fault() == BACK_MOTOR_FAIL)
    {
      BackMotor_Reset();
      __no_operation();
      __no_operation();
      BackMotor_Reset_Cancel();
      __no_operation();
      if(BackMotor_Get_Fault() != BACK_MOTOR_FAIL) val = 0;
    }
  }
  else
  {
   GPIO_PinOutSet(BACK_MOTOR_ENBL_PORT, BACK_MOTOR_ENBL_BIT);
   motorStatus = MOTOR_RUN;
   if(motorStatus == MOTOR_RUN)
   {
     motorStatus = MOTOR_RUN;
   }
   val =  1;
  }
  return val;
}
void BackPower_Off(void)
{
  GPIO_PinOutClear(BACK_MOTOR_ENBL_PORT, BACK_MOTOR_ENBL_BIT);
}

void BackMotor_Down(void)
{
  //Power_On();
  GPIO_PinOutSet(BACK_MOTOR_RESET_PORT, BACK_MOTOR_RESET_BIT);
  GPIO_PinOutSet(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT);
  GPIO_PinOutSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);
}
void BackMotor_Up(void)
{
  //Power_On();
  GPIO_PinOutSet(BACK_MOTOR_RESET_PORT, BACK_MOTOR_RESET_BIT);
  GPIO_PinOutClear(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT);
  GPIO_PinOutSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);
}
void BackMotor_Break(void)
{
  /*
  BackPower_Off();    //关闭马达电源
  if(motorStatus == MOTOR_RUN)
  {
   GPIO_PinOutSet(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT); //使马达端口处于高阻，此时马达属于惯性滑行
   motorStatus = MOTOR_STOP_HZ;
   motorBreakTime = 0;
  }
  if(motorStatus == MOTOR_STOP_HZ)
  {
    if(motorBreakTime < MOTOR_STOP_HZ_TIME) return;
  }
  GPIO_PinOutClear(BACK_MOTOR_DECAY_PORT, BACK_MOTOR_DECAY_BIT);  //短路马达，保持马达在刹车状态
  motorStatus = MOTOR_STOP_BREAK;
  */
  if(!breakDown_10MS_Int)return;
  breakDown_10MS_Int = false;
  //端点刹车距离137.5
  if(currentSpeed > BACK_MIN_SPEED){
    //端点急刹车
    if(Input_GetBackUpSwitch() == REACH_BACK_LIMIT ||
       Input_GetBackDownSwitch() == REACH_BACK_LIMIT)
    {
      currentSpeed -= 3+currentSpeed/40+currentSpeed/20;
    }
    //中间一般刹车
    else{
      currentSpeed -= 1+currentSpeed/100+currentSpeed/70;
    }
  }
  if(currentSpeed <= BACK_MIN_SPEED)currentSpeed = 0;  
}

void BackMotor_Reset(void)
{
  GPIO_PinOutClear(BACK_MOTOR_RESET_PORT, BACK_MOTOR_RESET_BIT);
}
void BackMotor_Reset_Cancel(void)
{
  GPIO_PinOutSet(BACK_MOTOR_RESET_PORT, BACK_MOTOR_RESET_BIT);
}
int BackMotor_Get_Fault(void)
{
  if(GPIO_PinInGet(BACK_MOTOR_FAULT_PORT, BACK_MOTOR_FAULT_BIT))
    return BACK_MOTOR_NORMAL;
  return BACK_MOTOR_FAIL;
}
////131223
unsigned int BackMotor_VoltageAdj(unsigned int setDuty)//fww
{
  unsigned short adc24;      //此处的电压值已经扩大了100倍
  //unsigned int setDuty = BACK_MOTOR_DEFAULT_TOP;//fww
  ADC_Get_Voltage(ADC_V24,&adc24);
  if(adc24 <= BACK_SET_VOLTAGE/100)
  {
    return setDuty;        //电压值偏低，返回预设值
  }
  unsigned int scale = BACK_SET_VOLTAGE / adc24; //计算与设定电压的比例值
  setDuty *= scale;
  unsigned int yushu = setDuty % 100;
  setDuty /= 100;
  if(yushu > 50) setDuty++;
  return setDuty;
}
void Back_Motor_PwmSet_Fault_Process(unsigned long ulDuty)
{
  //unsigned long  ulDuty;
  ulDuty = BackMotor_VoltageAdj(ulDuty); //ZERO_MOTOR_DEFAULT_TOP //fww
  unsigned int duty ;
  if(ulDuty == 0)
  {
    TIMER_CompareBufSet(BACK_MOTOR_TIMER, BACK_MOTOR_TIMER_CHANNEL, ulDuty);
    return;
  }
  duty = TIMER_CompareBufGet(BACK_MOTOR_TIMER, BACK_MOTOR_TIMER_CHANNEL);
  if(ulDuty == duty)
  {
    if(BackMotor_Get_Fault() == BACK_MOTOR_FAIL)
    {
      BackMotor_Reset();
      __no_operation();
      __no_operation();
      BackMotor_Reset_Cancel();
      return;
    }
    return;
  }
  if(!set_Pwm_10MS_Int) return;
  set_Pwm_10MS_Int = false;
  duty = ulDuty;
  TIMER_CompareBufSet(BACK_MOTOR_TIMER, BACK_MOTOR_TIMER_CHANNEL, duty);
  
  
  //SlideMotor_Set_PWM(ulDuty);
  motorStatus = MOTOR_RUN;
  //return 1;
}

//返回靠背电动缸的大位置 最高，最低或中间
unsigned int BackMotor_Get_Location(void)
{
  if(Input_GetBackUpSwitch() == REACH_BACK_LIMIT)   return BACK_MOTOR_AT_TOP;
  if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT)   return BACK_MOTOR_AT_BOTTOM;
  return BACK_MOTOR_AT_MID;
}
//返回靠背电动缸的绝对位置，靠时间来记录，靠背电动缸位于最高处时间为0，单位10ms
unsigned int BackMotor_Get_Position(void)
{
  return w_Position;
}

void BackMotor_10ms_int(void)
{
  bBackMotorFlag = TRUE;
  speedUp_10MS_Int = TRUE;
  breakDown_10MS_Int = TRUE;
  set_Pwm_10MS_Int = TRUE;
  slow_break_10MS_Int = TRUE;
}
void speedUp(void)
{
  if(!speedUp_10MS_Int) return;
  speedUp_10MS_Int = false;
  //启动需要的最小力度（电压占空比）
  if(currentSpeed < BACK_MIN_SPEED)currentSpeed = BACK_MIN_SPEED;
  if(isRocking){
    if(currentSpeed < BACK_MOTOR_DEFAULT_TOP - 10/* - 20*/){
      currentSpeed++;
    }
  }else{
    if(currentSpeed < BACK_MOTOR_DEFAULT_TOP){
      currentSpeed+=2;
    }
  }
}
void ZeroMotor_SlowBreak(void)
{
  if(!slow_break_10MS_Int)return;
  slow_break_10MS_Int = false;
  if(currentSpeed > 0)currentSpeed--;
  if(currentSpeed < BACK_MIN_SPEED)currentSpeed = 0;
}
void BackMotor_Proce(void)
{
  if(!bBackMotorFlag) return;
  bBackMotorFlag = FALSE;
  
  //if(!GPIO_PinOutGet(BACK_MOTOR_ENBL_PORT, BACK_MOTOR_ENBL_BIT)) return;
  
  if(TIMER_CompareBufGet(BACK_MOTOR_TIMER,BACK_MOTOR_TIMER_CHANNEL) <= BACK_MIN_SPEED) return;
  
  if(GPIO_PinOutGet(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT))
  {
    //if(w_Position > 0) w_Position--;//Fungares 
    if(w_Position < BACK_MOTOR_MAX_POSITION) w_Position++; //Fungares 
  }
  else
  {
    //if(w_Position < BACK_MOTOR_MAX_POSITION) w_Position++; //Fungares 
    if(w_Position > 0) w_Position--;//Fungares 
  }
}

//BackPad motor control function
/*
unsigned char BackMotor_Control(unsigned char nFinalBackPadMotorState)
{
  static unsigned int position = 0;
  unsigned char nRetVal ;
  bool bPowerFlag;
  nRetVal = FALSE ;
  switch(nFinalBackPadMotorState)
  {
  case STATE_RUN_BACK_DOWN:  //back motor go down
    if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT || position == 1)
    {
      position = 1;
      bPowerFlag = FALSE;
      nRetVal = TRUE ;
      w_Position = BACK_MOTOR_MAX_POSITION;
      BackMotor_Break();
      break;
    }
    position = 0;
    BackMotor_Down();
    bPowerFlag = TRUE;
    break ;
  case STATE_RUN_BACK_UP:  //back motor go up
    if(Input_GetBackUpSwitch() == REACH_BACK_LIMIT || position == 2)
    {
      position = 2;
      bPowerFlag = FALSE;
      nRetVal = TRUE ;
      w_Position = 0;
      BackMotor_Break();
      break;
    }
    position = 0;
    BackMotor_Up();
    bPowerFlag = TRUE;
    break ;
  case STATE_BACK_IDLE:
    nRetVal = TRUE ;
    BackMotor_Break();
    bPowerFlag = FALSE;
    break ;
  default://异常处理
    break ;
  }
  //电源部分的处理
  if(bPowerFlag == TRUE)
  {
    BackPower_On();
  }
  else
  {
    BackPower_Off();
  }
  return nRetVal ;
}
*/
//BackMotor
unsigned char BackMotor_Control(unsigned char nFinalBackPadMotorState)
{
  static unsigned int position = 0;
  unsigned char nRetVal ;
  bool bPowerFlag;
  nRetVal = FALSE ;
  unsigned char speedState = BACK_SPEED_STATE_STOP;//停止0，刹车1，加速2，缓刹车3，缓加速4
  //换向
  if(currentBackPadMotorState != nFinalBackPadMotorState
     && currentBackPadMotorState != STATE_BACK_IDLE)
  {
    //刹车
    //到顶处理
    if((currentBackPadMotorState == STATE_RUN_BACK_UP && Input_GetBackUpSwitch() == REACH_BACK_LIMIT)
       ||(currentBackPadMotorState == STATE_RUN_BACK_DOWN && Input_GetBackDownSwitch() == REACH_BACK_LIMIT))
    {
      //到顶时，急刹车
      speedState = BACK_SPEED_STATE_BREAK;
    }else if(isRocking){
      //摇椅时,缓刹车
      speedState = BACK_SPEED_STATE_SLOW_BREAK;
    }else{
      //手动时，急刹车
      speedState = BACK_SPEED_STATE_BREAK;
    }
    if(currentSpeed <= 0){
      currentBackPadMotorState = nFinalBackPadMotorState;
    }
  }
  else
  {
    currentBackPadMotorState = nFinalBackPadMotorState;
    switch(nFinalBackPadMotorState)
    {
    case STATE_RUN_BACK_DOWN:  //back motor go down
      if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT || position == 1)
      {
        position = 1;
        bPowerFlag = FALSE;
        nRetVal = TRUE ;
        w_Position = BACK_MOTOR_MAX_POSITION;
        BackMotor_Break();
        break;
      }
      position = 0;
      //BackMotor_Set_Route();
      BackMotor_Down();
      speedState = BACK_SPEED_STATE_UP;
      bPowerFlag = TRUE;
      break ;
    case STATE_RUN_BACK_UP:  //back motor go up
      if(Input_GetBackUpSwitch() == REACH_BACK_LIMIT || position == 2)
      {
        position = 2;
        bPowerFlag = FALSE;
        nRetVal = TRUE ;
        w_Position = 0;
        BackMotor_Break();
        break;
      }
      position = 0;
      //BackMotor_Set_Route();
      BackMotor_Up();
      speedState = BACK_SPEED_STATE_UP;
      bPowerFlag = TRUE;
      break ;
    case STATE_BACK_IDLE:
      nRetVal = TRUE ;
      //BackMotor_Set_Route();
      BackMotor_Break();
      speedState = BACK_SPEED_STATE_BREAK;
      bPowerFlag = FALSE;
      break ;
    default://异常处理
      break ;
    }
  }
  switch(speedState)
  {
  case BACK_SPEED_STATE_STOP:
    currentSpeed = 0;
    break;
  case BACK_SPEED_STATE_BREAK:
    BackMotor_Break();
    break;
  case BACK_SPEED_STATE_UP:
    speedUp();
    break;
  case BACK_SPEED_STATE_SLOW_BREAK:
    ZeroMotor_SlowBreak();
    break;
  case BACK_SPEED_STATE_SLOW_UP:
    //摇椅加速判断在函数中
    speedUp();
    break;
  }
  if(Input_GetBackUpSwitch() == REACH_BACK_LIMIT)
  {
    //Input_SetBackMotorPosition(BACK_MOTOR_MAX_POSITION_PULSE);
    Input_SetBackMotorPosition(0);
  }
  else if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT)
  {
    //Input_SetBackMotorPosition(0);
    Input_SetBackMotorPosition(BACK_MOTOR_MAX_POSITION_PULSE);
  }
  //越界处理
  if(currentSpeed > BACK_MOTOR_DEFAULT_TOP){
    currentSpeed = BACK_MOTOR_DEFAULT_TOP;
  }
  if(currentSpeed < 0){
    currentSpeed = 0;
  }
  GPIO_PinOutClear(BACK_MOTOR_DECAY_PORT,BACK_MOTOR_DECAY_BIT);
  Back_Motor_PwmSet_Fault_Process(currentSpeed);
  if(bPowerFlag == FALSE)
    bPowerFlag = FALSE;
  return nRetVal ;
}
void setBackPadRockingEnable(bool flag)
{
  isRocking = flag;
}
/////
void BackMotor_Set_Route(void)
{
  if(BACK_MOTOR_TIMER->ROUTE != (BACK_MOTOR_ROUTE_EN | BACK_MOTOR_ROUTE_LOCATION))
  {
    TIMER_Init_TypeDef timerInit = BACK_MOTOR_Timer_Init;
    TIMER_Init(BACK_MOTOR_TIMER, &timerInit);
    BACK_MOTOR_TIMER->ROUTE = (BACK_MOTOR_ROUTE_EN | BACK_MOTOR_ROUTE_LOCATION); 
    //BackMotor_Set_PWM(0);
  }
}
void BackMotor_Set_PWM(unsigned long ulDuty)
{
  TIMER_CompareBufSet(BACK_MOTOR_TIMER, BACK_MOTOR_TIMER_CHANNEL, ulDuty);
}