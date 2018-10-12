#include "efm32.h"
#include "efm32_chip.h"
#include "efm32_gpio.h"
#include "efm32_cmu.h"
#include "efm32_timer.h"
#include "efm32_def.h"
#include "Waveringly.h"
bool bWaveMotorAction;
void Waveringly_Initial_IO(void)
{
    GPIO_PinModeSet(WAVE_MOTOR_PORT, WAVE_MOTOR_BIT, WAVE_MOTOR_PORT_MODE, 1);
    
    TIMER_InitCC_TypeDef timerCCInit = WAVE_MOTOR_Timer_CCInit;
    /* Configure CC channel 0 */
    TIMER_InitCC(WAVE_MOTOR_TIMER, WAVE_MOTOR_TIMER_CHANNEL, &timerCCInit);
    
    /* Set Top Value */
    TIMER_TopSet(WAVE_MOTOR_TIMER, WAVE_MOTOR_DEFAULT_TOP);
    
    TIMER_CompareBufSet(WAVE_MOTOR_TIMER, WAVE_MOTOR_TIMER_CHANNEL, WAVE_MOTOR_DEFAULT_TOP);

    TIMER_Init_TypeDef timerInit = WAVE_MOTOR_Timer_Init;
   /* 
      TIMER_Init_TypeDef timerInit =
  {
    .enable     = true, 
    .debugRun   = true, 
    .prescale   = WAVE_MOTOR_PRESCALE, 
    .clkSel     = timerClkSelHFPerClk, 
    .count2x    = false,
    .ati        = false,
    .fallAction = timerInputActionNone, 
    .riseAction = timerInputActionNone, 
    .mode       = timerModeUp, 
    .dmaClrAct  = false,
    .quadModeX4 = false, 
    .oneShot    = false, 
    .sync       = false, 
  };
    
    */
    /* Configure timer */
    TIMER_Init(WAVE_MOTOR_TIMER, &timerInit);
    
        /* Route CC0 to location 3 (PD1) and enable pin */  
    WAVE_MOTOR_TIMER->ROUTE |= (WAVE_MOTOR_ROUTE_EN | WAVE_MOTOR_ROUTE_LOCATION); 
}

void Waveringly_Set_Pwm_Data(unsigned long ulDuty)
{
  
  if(ulDuty >(WAVE_MOTOR_DEFAULT_TOP-5)) bWaveMotorAction = FALSE;
  else bWaveMotorAction = TRUE;
  
        TIMER_CompareBufSet(WAVE_MOTOR_TIMER, WAVE_MOTOR_TIMER_CHANNEL, ulDuty);
}

bool WaveMotor_IsRun(void)
{
  return(bWaveMotorAction);
}