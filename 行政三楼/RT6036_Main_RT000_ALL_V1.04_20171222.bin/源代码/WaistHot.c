#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "WaistHot.h"

void WaistHeat_Initial_IO(void)
{
  GPIO_PinModeSet(WAIST_HEAT_PORT, WAIST_HEAT_BIT, WAIST_HEAT_MODE, 1);   
}

void WaistHeat_On(void)
{
  GPIO_PinOutClear(WAIST_HEAT_PORT, WAIST_HEAT_BIT);   
}

void WaistHeat_Off(void)
{
  GPIO_PinOutSet(WAIST_HEAT_PORT, WAIST_HEAT_BIT);   
}