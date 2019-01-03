#include "efm32.h"
#include "efm32_chip.h"
#include "efm32_gpio.h"
#include "efm32_timer.h"
#include "VfdPower.h"

void VFDPower_Initial_IO(void)
{
  //GPIO_PinModeSet(VFD_POWER_PORT,VFD_POWER_BIT,VFD_POWER_MODE,1);
  //GPIO_PinModeSet(POWER_PORT,POWER_BIT,POWER_MODE,1);
}
void VFDPower_On(void)
{
  //GPIO_PinOutSet(VFD_POWER_PORT,VFD_POWER_BIT);
}
void VFDPower_Off(void)
{
  //GPIO_PinOutClear(VFD_POWER_PORT,VFD_POWER_BIT);
}
