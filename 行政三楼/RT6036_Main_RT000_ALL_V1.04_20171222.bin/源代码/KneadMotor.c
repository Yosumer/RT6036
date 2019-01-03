#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "efm32_def.h"
#include "MassageStatus.h"
#include "em_dac.h"
#include "DAC.h"
#include "ADC_Single.h"
#include "KneadMotor.h"
unsigned int w_KneadCurrent = 8;
static unsigned int motorStatus = MOTOR_STOP_BREAK;
static volatile unsigned int motorBreakTime;
//static volatile unsigned int faultCounter = 1000;
static void KneadMotor_Set_Pwm_Data(unsigned long ulDuty);
void KneadMotor_Initial_IO(void)
{
  GPIO_PinModeSet(KNEAD_MOTOR_RESET_PORT, KNEAD_MOTOR_RESET_BIT, KNEAD_MOTOR_RESET_MODE, 0);
  GPIO_PinModeSet(KNEAD_MOTOR_ENBL_PORT, KNEAD_MOTOR_ENBL_BIT, KNEAD_MOTOR_ENBL_MODE, 0);
  GPIO_PinModeSet(KNEAD_MOTOR_PHASE_PORT, KNEAD_MOTOR_PHASE_BIT, KNEAD_MOTOR_PHASE_MODE, 0);
  GPIO_PinModeSet(KNEAD_MOTOR_DECAY_PORT, KNEAD_MOTOR_DECAY_BIT, KNEAD_MOTOR_DECAY_MODE, 0);
  GPIO_PinModeSet(KNEAD_MOTOR_FAULT_PORT, KNEAD_MOTOR_FAULT_BIT, KNEAD_MOTOR_FAULT_MODE, 1);
  GPIO_PinModeSet(KNEAD_MOTOR_CURRENT_PORT, KNEAD_MOTOR_CURRENT_BIT, KNEAD_MOTOR_CURRENT_MODE, 1);
    
    //TIMER_InitCC_TypeDef timerCCInit = KNEAD_MOTOR_Timer_CCInit;
    /* Configure CC channel 0 */
    //TIMER_InitCC(KNEAD_MOTOR_TIMER, KNEAD_MOTOR_TIMER_CHANNEL, &timerCCInit);

    /* Set Top Value */
    //TIMER_TopSet(KNEAD_MOTOR_TIMER, KNEAD_MOTOR_DEFAULT_TOP);
    
    //TIMER_CompareBufSet(KNEAD_MOTOR_TIMER, KNEAD_MOTOR_TIMER_CHANNEL, 0);

    //TIMER_Init_TypeDef timerInit = KNEAD_MOTOR_Timer_Init;
 
    /* Configure timer */
    //TIMER_Init(KNEAD_MOTOR_TIMER, &timerInit);
    
    /* Route CC0 to location 3 (PD1) and enable pin */  
    //KNEAD_MOTOR_TIMER->ROUTE |= (KNEAD_MOTOR_ROUTE_EN | KNEAD_MOTOR_ROUTE_LOCATION); 
}

void KneadMotor_Reset(void)
{
  KneadMotor_Set_Pwm_Data(0);
  GPIO_PinOutClear(KNEAD_MOTOR_RESET_PORT, KNEAD_MOTOR_RESET_BIT);
}

void KneadMotor_Reset_Cancel(void)
{
  GPIO_PinOutSet(KNEAD_MOTOR_RESET_PORT, KNEAD_MOTOR_RESET_BIT);
}
void KneadMotorBrake_On(void)
{
  KneadMotor_Set_Pwm_Data(0);
  GPIO_PinOutClear(KNEAD_MOTOR_DECAY_PORT, KNEAD_MOTOR_DECAY_BIT);
}
void KneadMotorBrake_Off(void)
{
  KneadMotor_Set_Pwm_Data(0);
  GPIO_PinOutSet(KNEAD_MOTOR_DECAY_PORT, KNEAD_MOTOR_DECAY_BIT);
}
void KneadMotor_10ms_Int(void)
{
//  faultCounter++;
//  if(faultCounter > 1000) faultCounter = 1000;
}

unsigned int KneadMotor_VoltageAdj(unsigned int setDuty)
{
  unsigned short adc24;      //此处的电压值已经扩大了100倍
  ADC_Get_Voltage(ADC_V24,&adc24);     
  if(adc24 <= KNEAD_SET_VOLTAGE/100) 
  {
    return setDuty;        //电压值偏低，返回预设值
  }
  unsigned int scale = KNEAD_SET_VOLTAGE / adc24; //计算与设定电压的比例值
  setDuty *= scale;
  unsigned int yushu = setDuty  % 100;
  setDuty /= 100;
  if(yushu > 50) setDuty++;
  return setDuty; 
}

void KneadMotor_Set_Pwm_Data(unsigned long ulDuty)
{
  if(ulDuty == 0)
  {
    TIMER_CompareBufSet(KNEAD_MOTOR_TIMER, KNEAD_MOTOR_TIMER_CHANNEL, ulDuty);
    return;
  }
  if(KneadMotor_Get_Fault() == KNEAD_MOTOR_FAIL) 
  {
    KneadMotor_Reset();
    __no_operation();
    __no_operation();
    KneadMotor_Reset_Cancel();
  } 
  ulDuty = KneadMotor_VoltageAdj(ulDuty);
  TIMER_CompareBufSet(KNEAD_MOTOR_TIMER, KNEAD_MOTOR_TIMER_CHANNEL, ulDuty);
  motorStatus = MOTOR_RUN;
}
bool KneadMotor_IsRun(void)
{
  unsigned long  ulDuty;
  ulDuty = TIMER_CompareBufGet(KNEAD_MOTOR_TIMER, KNEAD_MOTOR_TIMER_CHANNEL);
  if(ulDuty > 0) return 1;
  else return 0;
}

void KneadMotor_ClockRun(void)
{
  GPIO_PinOutSet(KNEAD_MOTOR_RESET_PORT, KNEAD_MOTOR_RESET_BIT);
  GPIO_PinOutSet(KNEAD_MOTOR_PHASE_PORT, KNEAD_MOTOR_PHASE_BIT);
  //GPIO_PinOutSet(KNEAD_MOTOR_DECAY_PORT, KNEAD_MOTOR_DECAY_BIT);
  GPIO_PinOutClear(KNEAD_MOTOR_DECAY_PORT, KNEAD_MOTOR_DECAY_BIT);
}
void KneadMotor_UnClockRun(void)
{
  GPIO_PinOutSet(KNEAD_MOTOR_RESET_PORT, KNEAD_MOTOR_RESET_BIT);
  GPIO_PinOutClear(KNEAD_MOTOR_PHASE_PORT, KNEAD_MOTOR_PHASE_BIT);
  //GPIO_PinOutSet(KNEAD_MOTOR_DECAY_PORT, KNEAD_MOTOR_DECAY_BIT);
  GPIO_PinOutClear(KNEAD_MOTOR_DECAY_PORT, KNEAD_MOTOR_DECAY_BIT);
}
void KneadMotor_Break(void)
{
  KneadMotor_Set_Pwm_Data(0);    //关闭马达电源
  if(motorStatus == MOTOR_RUN)
  {
   GPIO_PinOutSet(KNEAD_MOTOR_DECAY_PORT, KNEAD_MOTOR_DECAY_BIT); //使马达端口处于高阻，此时马达属于惯性滑行
   motorStatus = MOTOR_STOP_HZ;
   motorBreakTime = 0;
  }
  if(motorStatus == MOTOR_STOP_HZ)
  {
    if(motorBreakTime < MOTOR_STOP_HZ_TIME) return;
  }
  GPIO_PinOutClear(KNEAD_MOTOR_DECAY_PORT, KNEAD_MOTOR_DECAY_BIT);  //短路马达，保持马达在刹车状态
  motorStatus = MOTOR_STOP_BREAK;
}

int KneadMotor_Get_Fault(void)
{
  if(GPIO_PinInGet(KNEAD_MOTOR_FAULT_PORT, KNEAD_MOTOR_FAULT_BIT))
    return KNEAD_MOTOR_NORMAL;
  return KNEAD_MOTOR_FAIL;
}

void KneadMotorUpdateSpeed(unsigned int speed)
{
  switch(speed)
  {
  default:  
  case 0: KneadMotor_Set_Pwm_Data(KNEAD_SPEED0_PWM); break;
  case 1: KneadMotor_Set_Pwm_Data(KNEAD_SPEED1_PWM); break;
  case 2: KneadMotor_Set_Pwm_Data(KNEAD_SPEED2_PWM); break;
  case 3: KneadMotor_Set_Pwm_Data(KNEAD_SPEED3_PWM); break;
  case 4: KneadMotor_Set_Pwm_Data(KNEAD_SPEED4_PWM); break;
  case 5: KneadMotor_Set_Pwm_Data(KNEAD_SPEED5_PWM); break;
  case 6: KneadMotor_Set_Pwm_Data(KNEAD_SPEED6_PWM); break;
  }
}

//Knead motor control function
unsigned int KneadMotor_Control(unsigned int nMotorState,unsigned int PWM)
{
  unsigned int nRetVal ;
  bool bPowerFlag;
  nRetVal = FALSE ;
  switch(nMotorState)
  {
  case STATE_KNEAD_CLOCK_RUN:  //顺时针旋转
    nRetVal = FALSE ;
    KneadMotor_ClockRun();
    bPowerFlag = TRUE;
    break;
  case STATE_KNEAD_UNCLOCK_RUN:  //逆时针旋转
    nRetVal = FALSE ;
    KneadMotor_UnClockRun();
    bPowerFlag = TRUE;
    break;
  default:
  case STATE_KNEAD_IDLE:
    nRetVal = TRUE ;
    KneadMotor_Break();
    bPowerFlag = FALSE;
    break ;  
  } 
  //电源部分的处理
  if(bPowerFlag == TRUE)
  {
    KneadMotor_Set_Pwm_Data(PWM);
  }
  else
  {
    KneadMotor_Set_Pwm_Data(0);
  }
  return nRetVal ;
}


void KneadMotor_SetCurrent(int current)
{
  if(current > DAC_CURRENT_MAX_LEVEL)  current = DAC_CURRENT_MAX_LEVEL;
  switch(current)
  {
  case 0: DAC_Set_Data(DAC0, 0,DAC_CURRENT_1A); break;
  case 1: DAC_Set_Data(DAC0, 0,DAC_CURRENT_1_5A); break;
  case 2: DAC_Set_Data(DAC0, 0,DAC_CURRENT_2A); break;
  case 3: DAC_Set_Data(DAC0, 0,DAC_CURRENT_2_5A); break;
  case 4: DAC_Set_Data(DAC0, 0,DAC_CURRENT_3A); break;
  case 5: DAC_Set_Data(DAC0, 0,DAC_CURRENT_3_5A); break;
  case 6: DAC_Set_Data(DAC0, 0,DAC_CURRENT_4A); break;
  case 7: DAC_Set_Data(DAC0, 0,DAC_CURRENT_4_5A); break;
  case 8: DAC_Set_Data(DAC0, 0,DAC_CURRENT_5A); break;
  case 9: DAC_Set_Data(DAC0, 0,DAC_CURRENT_5_5A); break;
  }
  w_KneadCurrent = current;
}

unsigned int KneadMotor_GetCurrent(void)
{
  return(w_KneadCurrent);
}
void KneadMotor_AdcCurrent(void)
{
  if(w_KneadCurrent < DAC_CURRENT_MAX_LEVEL)
  {
    w_KneadCurrent++;
    KneadMotor_SetCurrent(w_KneadCurrent);
  }
}

void KneadMotor_DecCurrent(void)
{
  if(w_KneadCurrent > 0)
  {
    w_KneadCurrent--;
    KneadMotor_SetCurrent(w_KneadCurrent);
  }
}