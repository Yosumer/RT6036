//#include "efm32.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "EFM32_def.h"
#include "EFM32_types.h"
#include "em_letimer.h"
#include "em_vcmp.h"
#include "system.h"
#include "Timer.h"
#include "LegMotor.h"
#include "BackPad.h"
#include "WalkMotor.h"
#include "SlideMotor.h"
#include "em_gpio.h"
#include "input.h"
#include "Valve.h"
#include "Data_Cul.h"
#include "IndicateLED.h"
#include "ADC_Single.h"
#include "LED_RGB.h"        
#include "KneadMotor.h"        
#include "Walkmotor.h"        
#include "em_wdog.h"
#include "DMAUart.h"
#include "FlexPad.h"

extern void main_50ms_int(void);
extern void main_10ms_int(void);
extern void main_200ms_int(void);
bool  bTimer2MS;
unsigned int sysCounter;
void LETIMER_setup(void)
{
  /* Enable necessary clocks */
 // CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
 // CMU_ClockEnable(cmuClock_CORELE, true);
 // CMU_ClockEnable(cmuClock_LETIMER0, true);  
 // CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Configure PD6 and PD7 as push pull so the
     LETIMER can override them */
  //GPIO_PinModeSet(gpioPortD, 6, gpioModePushPull, 0);
  //GPIO_PinModeSet(gpioPortD, 7, gpioModePushPull, 0);
  
  /* Set initial compare values for COMP0 and COMP1 
     COMP1 keeps it's value and is used as TOP value
     for the LETIMER.
     COMP1 gets decremented through the program execution
     to generate a different PWM duty cycle */
  LETIMER_CompareSet(LETIMER0, 0, 32768);
//  LETIMER_CompareSet(LETIMER0, 1, 5000);
  
  /* Repetition values must be nonzero so that the outputs
     return switch between idle and active state */
//  LETIMER_RepeatSet(LETIMER0, 0, 0x01);
//  LETIMER_RepeatSet(LETIMER0, 1, 0x01);
  
  /* Route LETIMER to location 0 (PD6 and PD7) and enable outputs */
  //LETIMER0->ROUTE = LETIMER_ROUTE_OUT0PEN | LETIMER_ROUTE_OUT1PEN | LETIMER_ROUTE_LOCATION_LOC0;
  
  /* Set configurations for LETIMER 0 */
  const LETIMER_Init_TypeDef letimerInit = 
  {
  .enable         = true,                   /* Start counting when init completed. */
  .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
  .rtcComp0Enable = false,                  /* Don't start counting on RTC COMP0 match. */
  .rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
  .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
  .bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
  .out0Pol        = 0,                      /* Idle value for output 0. */
  .out1Pol        = 0,                      /* Idle value for output 1. */
  .ufoa0          = letimerUFOAPwm,         /* PWM output on output 0 */
  .ufoa1          = letimerUFOAPulse,       /* Pulse output on output 1*/
  .repMode        = letimerRepeatFree       /* Count until stopped */
  };
  
  /* Initialize LETIMER */
  LETIMER_Init(LETIMER0, &letimerInit); 
}

void System_Initial_IO(void)
{
    CHIP_Init();
    CMU_HFRCOBandSet(cmuHFRCOBand_28MHz);  /** main system clock - internal RC 28MHz*/
    SystemCoreClockUpdate();
    CMU_ClockEnable(cmuClock_HFPER,true);  /** High frequency peripheral clock */
    CMU_ClockEnable(cmuClock_CORELE, true);/* Enable CORELE clock */
    CMU_ClockEnable(cmuClock_GPIO,true);   /** General purpose input/output clock. */
    CMU_ClockEnable(cmuClock_USART0,true);
    CMU_ClockEnable(cmuClock_USART1,true);
    CMU_ClockEnable(cmuClock_UART1,true);
    CMU_ClockEnable(cmuClock_UART0,true);
    CMU_ClockEnable(cmuClock_ADC0, true);
    CMU_ClockEnable(cmuClock_AES, true);  //fww 加密
    CMU_ClockEnable(cmuClock_TIMER0,true);
    CMU_ClockEnable(cmuClock_TIMER1,true);
    CMU_ClockEnable(cmuClock_TIMER2,true);
    CMU_ClockEnable(cmuClock_TIMER3,true);
    CMU_ClockEnable(cmuClock_PRS, true);
    CMU_ClockEnable(cmuClock_DMA, true);        /* Enable DMA clock */
    //CMU_ClockEnable(cmuClock_LEUART0, true);    /* Enable LEUART1 clock */
    SysTick_Config(SystemCoreClock / 1000); //set 1ms interupt using systick
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO); //LFA选择内部32768时钟
    //CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
    //GPIO_PinModeSet(TEST_PORT, TEST_BIT, TEST_MODE, 1);
    CMU_ClockEnable(cmuClock_LETIMER0, true);  
    LETIMER_setup();
   /* Enable underflow interrupt */  
   LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF);  
  /* Enable LETIMER0 interrupt vector in NVIC*/
  NVIC_EnableIRQ(LETIMER0_IRQn);
  CMU_IntClear(CMU_IFC_LFXORDY);
  CMU_IntEnable(CMU_IEN_LFXORDY);
  NVIC_EnableIRQ(CMU_IRQn);
  CMU_OscillatorEnable(cmuOsc_LFXO,1,0);
  
  VCMP_Init_TypeDef vcmp =
  {
    true,                               /* Half bias current */
    0,                                  /* Bias current configuration */
    true,                               /* Enable interrupt for falling edge */
    false,                              /* Enable interrupt for rising edge */
    vcmpWarmTime256Cycles,              /* Warm-up time in clock cycles */
    vcmpHyst20mV,                       /* Hysteresis configuration */
    1,                                  /* Inactive comparator output value */
    false,                              /* Enable low power mode */
    VCMP_VoltageToLevel(2.5), /* Trigger level */
    false                               /* Enable VCMP after configuration */
  };
  /* Initialize VCMP */
  CMU_ClockEnable(cmuClock_VCMP, true);
  VCMP_Init(&vcmp);
  
  /* Enable VCMP interrupt lines */
  NVIC_EnableIRQ(VCMP_IRQn);
  VCMP_IntEnable(VCMP_IEN_EDGE | VCMP_IEN_WARMUP);
  
  /* Enable VCMP and wait for warm-up complete */
  VCMP_Enable();
  
}
//1ms interupt

bool bFlag1ms;
bool bVibrateEnable;
unsigned int Vibrate_Pwm_Speed;

void SysTick_Handler(void)
{
  static BYTE by_Time5ms = 0;
  static BYTE by_Time10ms = 0;
  static BYTE by_Time50ms = 0;
  static BYTE by_Time100ms = 0;
  static BYTE by_Time200ms = 0;
  static unsigned long by_Time3s = 0;

  if(ValveFungares6||ValveFungares4||ValveFungares5||ValveFungares3)
  {
  if(by_Time3s >= 1000)
  {
    bVibrateEnable = 1;
    by_Time3s = 0;
  }
  else ++by_Time3s;
  }
  else 
  {
    bVibrateEnable = 0;
    Vibrate_Pwm_Speed = 0;
  }
  // only test DC airbag  
  /*
  static BYTE by_Time20ms = 0;
  
  by_Time20ms++;
  by_Time20ms %= 20;
  if(by_Time20ms < 9)
  {
  GPIO_PinOutSet(SLIDE_MOTOR_ENBL_PORT, SLIDE_MOTOR_ENBL_BIT); //enable airbag 
  GPIO_PinOutSet(SLIDE_MOTOR_PHASE_PORT, SLIDE_MOTOR_PHASE_BIT); 
}
   else if(by_Time20ms > 10)
  {
  GPIO_PinOutSet(SLIDE_MOTOR_ENBL_PORT, SLIDE_MOTOR_ENBL_BIT); //enable airbag
  GPIO_PinOutClear(SLIDE_MOTOR_PHASE_PORT, SLIDE_MOTOR_PHASE_BIT); 
}
   else
  {
  //GPIO_PinOutClear(SLIDE_MOTOR_ENBL_PORT, SLIDE_MOTOR_ENBL_BIT); //Disable airbag 
  SlideMotor_Break();
}
  */
  bFlag1ms = 1;
  sysCounter++;
  Valve_1ms_Int();
  ADC_1ms_Int();
  if(by_Time5ms >= 4)
  { 
    Valve_10ms_Int();
    Input_5ms_Int();
  }                          
  else ++by_Time5ms;
  if(by_Time10ms >= 9)
  { 
    WDOG_Feed();
    //bluetoothCountIncrease();//fww
    FlexMotor_10ms_Int();
    WalkMotor_10ms_Int();
    SlideMotor_10ms_Int();
    BackMotor_10ms_int();
    LegMotor_10ms_int();
    // SlideMotor_10ms_int();
    KneadMotor_10ms_Int();
    by_Time10ms = 0;         
    //Input_10ms_Int();
    main_10ms_int();
    LED_RGB_10ms_Int();  
    if(bVibrateEnable)
    {
      static char degree = 1;
      if(degree)
      {
        Vibrate_Pwm_Speed++;
        if(Vibrate_Pwm_Speed >= 130)
        degree = 0;
      }
      else{
        Vibrate_Pwm_Speed--;
        if(Vibrate_Pwm_Speed <= 0)
        degree = 1;
      }
    }
  }                          
  else ++by_Time10ms;
  
  if(by_Time50ms >= 50)
  {                          
    by_Time50ms = 0;         
    main_50ms_int();
  }                          
  else ++by_Time50ms;
  
  if(by_Time100ms >= 100)
  {                          
    by_Time100ms = 0;  
    LED_RGB_100ms_Int();
    //  LegMotor_100ms_Int();
    // BackMotor_100ms_Int();
    // WalkMotor_100ms_Int();
    // ZeroMotor_100ms_Int();
    Timer_Flag_100ms_Int();
    //FlexMotor_100ms_Int();
  }                          
  else ++by_Time100ms;
  //
  if(by_Time200ms >= 200)
  {
    by_Time200ms = 0;
    main_200ms_int();
  }
  else ++by_Time200ms;
  //140906
  //get backmotor's pulse
  Input_Back_Pulse1MS();
}

void System_DelayXms(unsigned int ulData)
{
  while(ulData > 0)
  {
    while(!bFlag1ms); 
    bFlag1ms = 0;
    ulData--;
  }
}

void CMU_IRQHandler(void)
{
  NVIC_DisableIRQ(CMU_IRQn);
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);  //选择外部32768时钟
  CMU_OscillatorEnable(cmuOsc_LFRCO,0,0);  //禁止内部32768时钟，省电
  
}
/**************************************************************************//**
 * @brief LETIMER0_IRQHandler
 * Interrupt Service Routine for LETIMER
 * 中断时间1秒钟
 *****************************************************************************/
void LETIMER0_IRQHandler(void)
{ 
  /* Clear LETIMER0 underflow interrupt flag */
  LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);
  Data_Flag_Int();
 // IndicateLED_Toggle();
}


unsigned int  System_GetCounter(void)
{ 
  return(sysCounter);
}


/***************************************************************************//**
 * @brief
 *   VCMP interrupt handler, triggers on EDGE and WARMUP events
 ******************************************************************************/
extern unsigned int password;
void VCMP_IRQHandler()
{
  /* Execute on WARMUP interrupt */
  if (VCMP->IF & VCMP_IF_WARMUP)
  {
    /* Enable Low Power Reference */
   // VCMP_LowPowerRefSet(true);

    /* Clear interrupt flag */
    VCMP_IntClear(VCMP_IFC_WARMUP);
  }

  /* Execute on EDGE interrupt */
  if (VCMP->IF & VCMP_IF_EDGE)
  {
    /* Low voltage warning */
   password = 0;  //低电压清除password

    /* Clear interrupt flag */
    VCMP_IntClear(VCMP_IFC_EDGE);
  }
}

