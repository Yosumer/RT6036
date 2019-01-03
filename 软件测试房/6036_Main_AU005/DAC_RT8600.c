#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "em_dac.h"
#include "DAC.h"

/** Default config for DAC init structure. */
#define DAC_INIT_DEFAULT_KNOCK                                                 \
  { dacRefresh8,              /* Refresh every 8 prescaled cycles. */    \
    dacRefVDD,               /* VDD reference. */            \
    dacOutputPin,          /* Output to pin only. */                  \
    dacConvModeContinuous,    /* Continuous mode. */                     \
    0,                        /* No prescaling. */                       \
    false,                    /* Do not enable low pass filter. */       \
    false,                    /* Do not reset prescaler on ch0 start. */ \
    false,                    /* DAC output enable always on. */         \
    false,                    /* Disable sine mode. */                   \
    false                     /* Single ended mode. */                   \
  }

void DAC_setup(void)
{
  DAC_Init_TypeDef        init        = DAC_INIT_DEFAULT_KNOCK;
  DAC_InitChannel_TypeDef initChannel = DAC_INITCHANNEL_DEFAULT;

  CMU_ClockEnable(cmuClock_DAC0, true);
  /* Calculate the DAC clock prescaler value that will result in a DAC clock
   * close to 1 MHz. Second parameter is zero, if the HFPERCLK value is 0, the
   * function will check what the HFPERCLK actually is. */
  init.prescale = DAC_PrescaleCalc(1000000, 0);

  /* Initialize the DAC. */
  DAC_Init(DAC0, &init);

  /* Enable prs to trigger samples at the right time with the timer */
 // initChannel.prsEnable = true;
  initChannel.prsSel    = dacPRSSELCh0;

  /* Both channels can be configured the same
   * and be triggered by the same prs-signal. */
  DAC_InitChannel(DAC0, &initChannel, 0);
  DAC_InitChannel(DAC0, &initChannel, 1);
}

void DAC_Initial_IO(void)
{
  DAC_setup();
  
  DAC_Enable(DAC0, 0, true);
  DAC_Enable(DAC0, 1, true);
  
  DAC_Set_Data(DAC0, 0,DAC_CURRENT_5A); //ÈàÄó5A
  DAC_Set_Data(DAC0, 1,DAC_CURRENT_3A); //ÇÃ»÷3A
}



