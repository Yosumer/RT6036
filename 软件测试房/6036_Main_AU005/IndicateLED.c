#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "IndicateLED.h"

void IndicateLED_Initial_IO(void)
{
    GPIO_PinModeSet(INDICATE_LED_PORT, INDICATE_LED_BIT, INDICATE_LED_MODE, 0);   /** Set GPIO PortC as push-pull output */ 
}

void IndicateLED_On(void)
{
    GPIO_PinOutClear(INDICATE_LED_PORT, INDICATE_LED_BIT);
}

void IndicateLED_Off(void)
{
    GPIO_PinOutSet(INDICATE_LED_PORT, INDICATE_LED_BIT);
}
void IndicateLED_Toggle(void)
{
    GPIO_PinOutToggle(INDICATE_LED_PORT, INDICATE_LED_BIT);
}