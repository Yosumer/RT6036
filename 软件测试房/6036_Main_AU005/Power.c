#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "power.h"

void Power_Initial_IO(void)
{
  GPIO_PinModeSet(POWER_GENERAL_PORT, POWER_GENERAL_BIT, POWER_GENERAL_MODE, GENERAL_POWER_OFF); 
  GPIO_PinModeSet(POWER_3V3_PORT, POWER_3V3_BIT, POWER_3V3_MODE, GENERAL_3V3_ON); 
  GPIO_PinModeSet(POWER_AMP_PORT, POWER_AMP_BIT, POWER_AMP_MODE, GENERAL_AMP_ON); 
}

unsigned int Power_Get(void)
{
  return(GPIO_PinOutGet(POWER_GENERAL_PORT,POWER_GENERAL_BIT));
}

void Power_3V3_On(void)
{
  GPIO_PinOutSet(POWER_3V3_PORT,POWER_3V3_BIT);
}
void Power_3V3_Off(void)
{
  GPIO_PinOutClear(POWER_3V3_PORT,POWER_3V3_BIT);
}    
void Power_On(void)
{
  GPIO_PinOutSet(POWER_GENERAL_PORT,POWER_GENERAL_BIT);
}
void Power_Off(void)
{
  GPIO_PinOutClear(POWER_GENERAL_PORT,POWER_GENERAL_BIT);
}    
//AMP power on
void Power_AMP_On(void)
{
  GPIO_PinOutClear(POWER_AMP_PORT,POWER_AMP_BIT);
}
//AMP power off
void Power_AMP_Off(void)
{
  GPIO_PinOutSet(POWER_AMP_PORT,POWER_AMP_BIT);
}

void Power_All_Off(void)
{
  //Power_AMP_Off();
  Power_Off();
  Power_3V3_Off();
}

void Power_All_On(void)
{
  //Power_AMP_On();
  Power_On();
  Power_3V3_On();
}
