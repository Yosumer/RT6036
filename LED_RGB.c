#include "EFM32_def.h"
#include "EFM32_types.h"
#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "MassageStatus.h"
#include "SlideMotor.h"
#include "LED_RGB.h"        

bool bFlag10ms;
unsigned int w_LEDSteps;  //用於記錄當前的顯示步驟
unsigned int w_LEDTimer;  //用於記錄當前步骤的运行时间 单位0.1秒
unsigned int w_LEDCtrlTimer;  //控制LED的渐亮或渐暗的定时器
unsigned char by_Red_PWM,by_Green_PWM,by_Blue_PWM;
bool b_LED_Init;

  const st_LEDCtrlStruct LED_Settle[] = 
   {
     {0,0,0,LED_DARK,LED_DARK,LED_DARK,10},
     {80,0,0,LED_SLOWLY_LIGHT,LED_DARK,LED_DARK,2},     //红色
     {100,0,0,LED_SLOWLY_DARK,LED_SLOWLY_LIGHT,LED_DARK,50}, //红色渐暗绿色渐亮
     {0,100,0,LED_DARK,LED_SLOWLY_DARK,LED_SLOWLY_LIGHT,50}, //绿色渐暗蓝色渐亮
     {0,0,100,LED_SLOWLY_LIGHT,LED_DARK,LED_SLOWLY_DARK,50}, ////红色渐亮蓝色渐暗
   };

  const st_LEDCtrlStruct LED_Enginrering[] = 
   {
     {0,0,0,LED_DARK,LED_DARK,LED_DARK,10},
   };
  
  const st_LEDCtrlStruct LED_WaitCommand[] = 
   {
     {0,0,0,LED_SLOWLY_LIGHT,LED_DARK,LED_DARK,50},  //红色渐亮
     {100,0,0,LED_SLOWLY_DARK,LED_DARK,LED_DARK,50},  //红色渐暗
     
     {0,0,0,LED_DARK,LED_DARK,LED_DARK,10},  //关1秒
     
     {0,0,0,LED_DARK,LED_SLOWLY_LIGHT,LED_DARK,50},  //绿色渐亮
     {0,100,0,LED_DARK,LED_SLOWLY_DARK,LED_DARK,50},  //绿色渐暗
     {0,0,0,LED_DARK,LED_DARK,LED_DARK,10},  //关1秒
     
     {0,0,0,LED_DARK,LED_DARK,LED_SLOWLY_LIGHT,50},  //绿色渐亮
     {0,0,100,LED_DARK,LED_DARK,LED_SLOWLY_DARK,50},  //绿色渐暗
     {0,0,0,LED_DARK,LED_DARK,LED_DARK,10},  //关1秒
     
     {0,0,0,LED_SLOWLY_LIGHT,LED_SLOWLY_LIGHT,LED_SLOWLY_LIGHT,50},     //白色渐亮
     {100,100,100,LED_SLOWLY_DARK,LED_SLOWLY_DARK,LED_SLOWLY_DARK,50},  //白色渐暗
     {0,0,0,LED_DARK,LED_DARK,LED_DARK,10},  //关1秒
     
     {100,0,0,LED_LIGHT,LED_DARK,LED_DARK,5},     //红色
     {100,30,0,LED_LIGHT,LED_LIGHT,LED_DARK,5},   //橙色
     {100,100,0,LED_LIGHT,LED_LIGHT,LED_DARK,5},  //黄色
     {0,100,0,LED_DARK,LED_LIGHT,LED_DARK,5},     //绿色
     {0,0,100,LED_DARK,LED_DARK,LED_LIGHT,5},     //蓝色
     {100,0,100,LED_LIGHT,LED_DARK,LED_LIGHT,5},  //紫色
     
     {0,0,0,LED_DARK,LED_DARK,LED_DARK,10},
     
     {50,0,0,LED_SLOWLY_LIGHT,LED_DARK,LED_DARK,10},     //红色
     {100,0,0,LED_SLOWLY_DARK,LED_SLOWLY_LIGHT,LED_DARK,50}, //红色渐暗绿色渐亮
     {0,100,0,LED_DARK,LED_SLOWLY_DARK,LED_SLOWLY_LIGHT,50}, //绿色渐暗蓝色渐亮
     {0,0,100,LED_SLOWLY_LIGHT,LED_DARK,LED_SLOWLY_DARK,50}, ////红色渐亮蓝色渐暗

   };
  const st_LEDCtrlStruct LED_Running[] = 
   {
     {0,0,0,LED_SLOWLY_LIGHT,LED_DARK,LED_DARK,50},  //红色渐亮
     {100,0,0,LED_SLOWLY_DARK,LED_DARK,LED_DARK,50},  //红色渐暗
     {0,0,0,LED_DARK,LED_DARK,LED_DARK,10},  //关1秒
     
     {0,0,0,LED_DARK,LED_SLOWLY_LIGHT,LED_DARK,50},  //绿色渐亮
     {0,100,0,LED_DARK,LED_SLOWLY_DARK,LED_DARK,50},  //绿色渐暗
     {0,0,0,LED_DARK,LED_DARK,LED_DARK,10},  //关1秒
     
     {0,0,0,LED_DARK,LED_DARK,LED_SLOWLY_LIGHT,50},  //绿色渐亮
     {0,0,100,LED_DARK,LED_DARK,LED_SLOWLY_DARK,50},  //绿色渐暗
     {0,0,0,LED_DARK,LED_DARK,LED_DARK,10},  //关1秒
     
     {0,0,0,LED_SLOWLY_LIGHT,LED_SLOWLY_LIGHT,LED_SLOWLY_LIGHT,50},     //白色渐亮
     {100,100,100,LED_SLOWLY_DARK,LED_SLOWLY_DARK,LED_SLOWLY_DARK,50},  //白色渐暗
     {0,0,0,LED_DARK,LED_DARK,LED_DARK,10},  //关1秒
     
     {100,0,0,LED_LIGHT,LED_DARK,LED_DARK,5},     //红色
     {100,30,0,LED_LIGHT,LED_LIGHT,LED_DARK,5},   //橙色
     {100,100,0,LED_LIGHT,LED_LIGHT,LED_DARK,5},  //黄色
     {0,100,0,LED_DARK,LED_LIGHT,LED_DARK,5},     //绿色
     {0,0,100,LED_DARK,LED_DARK,LED_LIGHT,5},     //蓝色
     {100,0,100,LED_LIGHT,LED_DARK,LED_LIGHT,5},  //紫色
     
     {0,0,0,LED_DARK,LED_DARK,LED_DARK,10},
     
     {50,0,0,LED_SLOWLY_LIGHT,LED_DARK,LED_DARK,10},     //红色
     {100,0,0,LED_SLOWLY_DARK,LED_SLOWLY_LIGHT,LED_DARK,50}, //红色渐暗绿色渐亮
     {0,100,0,LED_DARK,LED_SLOWLY_DARK,LED_SLOWLY_LIGHT,50}, //绿色渐暗蓝色渐亮
     {0,0,100,LED_SLOWLY_LIGHT,LED_DARK,LED_SLOWLY_DARK,50}, ////红色渐亮蓝色渐暗
   };
  const st_LEDCtrlStruct LED_Problem[] = 
   {
     {0,0,0,LED_DARK,LED_DARK,LED_DARK,2},
     {100,0,0,LED_LIGHT,LED_DARK,LED_DARK,2},
   };

  const st_LEDCtrlStruct LED_Idle[] = 
   {
     {0,0,0,LED_DARK,LED_DARK,LED_DARK,10},
   };
#define LED_SETTLE_LENGTH         sizeof(LED_Settle)/sizeof(st_LEDCtrlStruct)
#define LED_WAIT_COMMAND_LENGTH   sizeof(LED_WaitCommand)/sizeof(st_LEDCtrlStruct)
#define LED_RUNNING_LENGTH        sizeof(LED_Running)/sizeof(st_LEDCtrlStruct)
#define LED_PROBLEM_LENGTH        sizeof(LED_Problem)/sizeof(st_LEDCtrlStruct)
#define LED_IDLE_LENGTH           sizeof(LED_Idle)/sizeof(st_LEDCtrlStruct)

void LED_RGB_Initial_IO(void)
{
  GPIO_PinModeSet(LED_RED_PORT, LED_RED_BIT, LED_RED_MODE, 0);
  GPIO_PinModeSet(LED_GREEN_PORT, LED_GREEN_BIT, LED_GREEN_MODE, 0);
  GPIO_PinModeSet(LED_BLUE_PORT, LED_BLUE_BIT, LED_BLUE_MODE, 0);
  
  TIMER_InitCC_TypeDef timerCCInit = LED_RGB_Timer_CCInit;
  /* Configure CC channel 0 */
  TIMER_InitCC(LED_RGB_TIMER, LED_RGB_RED_CHANNEL, &timerCCInit);
  TIMER_InitCC(LED_RGB_TIMER, LED_RGB_GREEN_CHANNEL, &timerCCInit);
  TIMER_InitCC(LED_RGB_TIMER, LED_RGB_BLUE_CHANNEL, &timerCCInit);
  
  /* Set Top Value */
  TIMER_TopSet(LED_RGB_TIMER, LED_RGB_DEFAULT_TOP);
  
  TIMER_CompareBufSet(LED_RGB_TIMER, LED_RGB_RED_CHANNEL, 0);
  TIMER_CompareBufSet(LED_RGB_TIMER, LED_RGB_GREEN_CHANNEL, 0);
  TIMER_CompareBufSet(LED_RGB_TIMER, LED_RGB_BLUE_CHANNEL, 0);
  
  
  LED_RGB_Set_Route();
  /* Route CC0 to location 3 (PD1) and enable pin */  
  //    LED_RGB_TIMER->ROUTE |= (LED_RGB_RED_ROUTE_EN | LED_RGB_ROUTE_LOCATION); 
  //    LED_RGB_TIMER->ROUTE |= (LED_RGB_GREEN_ROUTE_EN | LED_RGB_ROUTE_LOCATION); 
  //    LED_RGB_TIMER->ROUTE |= (LED_RGB_BLUE_ROUTE_EN | LED_RGB_ROUTE_LOCATION); 
  b_LED_Init = 1;
}
void LED_RGB_Set_Route(void)
{
    TIMER_Init_TypeDef timerInit = LED_RGB_Timer_Init;
    TIMER_Init(LED_RGB_TIMER, &timerInit);
    LED_RGB_TIMER->ROUTE = (LED_RGB_RED_ROUTE_EN | LED_RGB_ROUTE_LOCATION); 
    LED_RGB_TIMER->ROUTE |= (LED_RGB_GREEN_ROUTE_EN | LED_RGB_ROUTE_LOCATION); 
    LED_RGB_TIMER->ROUTE |= (LED_RGB_BLUE_ROUTE_EN | LED_RGB_ROUTE_LOCATION); 
}

void LED_RGB_Red_Set(void)
{
 GPIO_PinOutSet(LED_RED_PORT, LED_RED_BIT); 
}
void LED_RGB_Red_Clear(void)
{
 GPIO_PinOutClear(LED_RED_PORT, LED_RED_BIT); 
}
void LED_RGB_Red_Toggle(void)
{
  GPIO_PinOutToggle(LED_RED_PORT, LED_RED_BIT); 
}
void LED_RGB_Green_Set(void)
{
 GPIO_PinOutSet(LED_GREEN_PORT, LED_GREEN_BIT); 
}
void LED_RGB_Green_Clear(void)
{
 GPIO_PinOutClear(LED_GREEN_PORT, LED_GREEN_BIT); 
}
void LED_RGB_Green_Toggle(void)
{
 GPIO_PinOutToggle(LED_GREEN_PORT, LED_GREEN_BIT); 
}
void LED_RGB_Blue_Set(void)
{
 GPIO_PinOutSet(LED_BLUE_PORT, LED_BLUE_BIT); 
}
void LED_RGB_Blue_Clear(void)
{
  GPIO_PinOutClear(LED_BLUE_PORT, LED_BLUE_BIT); 
}
void LED_RGB_Blue_Toggle(void)
{
  GPIO_PinOutToggle(LED_BLUE_PORT, LED_BLUE_BIT); 
}

void LED_RGB_Set_Red_Data(unsigned long ulDuty)
{
  LED_RGB_Set_Route();
  TIMER_CompareBufSet(LED_RGB_TIMER, LED_RGB_RED_CHANNEL, ulDuty);
}
void LED_RGB_Set_Green_Data(unsigned long ulDuty)
{
 LED_RGB_Set_Route();
 TIMER_CompareBufSet(LED_RGB_TIMER, LED_RGB_GREEN_CHANNEL, ulDuty);
}

void LED_RGB_Set_Blue_Data(unsigned long ulDuty)
{
  LED_RGB_Set_Route();
  TIMER_CompareBufSet(LED_RGB_TIMER, LED_RGB_BLUE_CHANNEL, ulDuty);
}

void LED_RGB_10ms_Int(void)
{
  bFlag10ms = TRUE;
  w_LEDCtrlTimer++;
}
void LED_RGB_100ms_Int(void)
{
  w_LEDTimer++;
}

BYTE LED_RGB_Set_Pwm(BYTE CtrlMode,BYTE PWM_init)  
{
  BYTE pwm;
  unsigned int w_PWM;
 switch(CtrlMode)
  {
   case LED_LIGHT: pwm = PWM_init; break; 
   case LED_DARK:  pwm = PWM_init; break; 
   case LED_SLOWLY_LIGHT: 
        w_PWM = w_LEDCtrlTimer >> 2; 
        if(w_PWM > 100) w_PWM = 100;
        pwm = (unsigned char)w_PWM;   
        break; 
   case LED_SLOWLY_DARK: 
        w_PWM = w_LEDCtrlTimer >> 2; 
        if(w_PWM > 100) w_PWM = 100;
        w_PWM = 100 - w_PWM;
        pwm = (unsigned char)w_PWM;   
     break; 
  default:  break;  
  } 
 return pwm;
} 
  
void LED_RGB_Proce(unsigned char nChairRunState)
{
  if(!bFlag10ms) return;
  bFlag10ms = FALSE;
  const st_LEDCtrlStruct* pst_LED; 
  unsigned int length;
  static BYTE OldState = CHAIR_STATE_IDLE;
  static bool bRoute = false;
  if(LED_RGB_TIMER->ROUTE == (SLIDE_MOTOR_ROUTE_EN | SLIDE_MOTOR_ROUTE_LOCATION)) 
  //if(SlideMotor_GetPower() == SLIDE_MOTOR_POWER_ON)
  {
   LED_RGB_Red_Set();
   LED_RGB_Green_Set();
   LED_RGB_Blue_Set();
   bRoute = false;
   return; 
  }
  if(!bRoute)
  {
    bRoute = true;
    LED_RGB_Set_Route();
  }

  if(OldState != nChairRunState)
     {
       b_LED_Init = TRUE;  //按摩椅状态切换时，需要初始化按摩椅控制程序
       OldState = nChairRunState;
     }
  switch(nChairRunState)
  {
   case CHAIR_STATE_IDLE: 
     pst_LED = LED_Idle;
     length = LED_IDLE_LENGTH;
     break;
   case CHAIR_STATE_SETTLE: 
     pst_LED = LED_Settle;
     length = LED_SETTLE_LENGTH;
     break;
   case CHAIR_STATE_WAIT_COMMAND: 
     pst_LED = LED_WaitCommand;
     length = LED_WAIT_COMMAND_LENGTH;
     break;
   case CHAIR_STATE_RUN:
     pst_LED = LED_Running;
     length = LED_RUNNING_LENGTH;
     break;
   case CHAIR_STATE_WAIT_MEMORY: 
     break;
   case CHAIR_STATE_PROBLEM: 
     pst_LED = LED_Problem;
     length = LED_PROBLEM_LENGTH;
     break;
  case CHAIR_STATE_ENGINEERING:
     pst_LED = LED_Enginrering;
     length = LED_PROBLEM_LENGTH;
    break;
    //140531
  case CHAIR_STATE_SETTLE_1ST:
     pst_LED = LED_Running;
     length = LED_RUNNING_LENGTH;
    break;
  }
  if(b_LED_Init)  //LED显示数据被初始化，显示从表头开始
  {
    w_LEDTimer = 0;
    w_LEDCtrlTimer = 0;
    w_LEDSteps = 0;
    by_Red_PWM = (pst_LED + w_LEDSteps)->nRedInit;
    by_Blue_PWM = (pst_LED + w_LEDSteps)->nBlueInit;
    by_Green_PWM = (pst_LED + w_LEDSteps)->nGreenInit;
    b_LED_Init = FALSE;
    LED_RGB_Set_Red_Data(by_Red_PWM);
    LED_RGB_Set_Green_Data(by_Green_PWM);
    LED_RGB_Set_Blue_Data(by_Blue_PWM);
    return;
  }
  
  by_Red_PWM = LED_RGB_Set_Pwm((pst_LED + w_LEDSteps)->nRedCtrl,(pst_LED + w_LEDSteps)->nRedInit);
  by_Green_PWM = LED_RGB_Set_Pwm((pst_LED + w_LEDSteps)->nGreenCtrl,(pst_LED + w_LEDSteps)->nGreenInit);
  by_Blue_PWM = LED_RGB_Set_Pwm((pst_LED + w_LEDSteps)->nBlueCtrl,(pst_LED + w_LEDSteps)->nBlueInit);
  
  if(w_LEDTimer >= (pst_LED + w_LEDSteps)->nTime)
    {
      w_LEDSteps++;
      if(w_LEDSteps >= length) 
      {
        b_LED_Init = TRUE;
      }
      else
      {
        w_LEDTimer = 0;
        w_LEDCtrlTimer = 0;
        by_Red_PWM = (pst_LED + w_LEDSteps)->nRedInit;
        by_Blue_PWM = (pst_LED + w_LEDSteps)->nBlueInit;
        by_Green_PWM = (pst_LED + w_LEDSteps)->nGreenInit;
      }
    }
    LED_RGB_Set_Red_Data(by_Red_PWM);
    LED_RGB_Set_Green_Data(by_Green_PWM);
    LED_RGB_Set_Blue_Data(by_Blue_PWM);
}

void LED_RGB_Set_All(unsigned int w_Data)
{
  unsigned int DispVal;
 if(w_Data)  DispVal = 100;
   else DispVal = 0;
   LED_RGB_Set_Red_Data(DispVal);
   LED_RGB_Set_Green_Data(DispVal);
   LED_RGB_Set_Blue_Data(DispVal);
}
