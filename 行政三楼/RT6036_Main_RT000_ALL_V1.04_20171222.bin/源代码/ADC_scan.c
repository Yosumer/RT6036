/******************************************************************************
 * @file
 * @brief ADC scan conversion example
 * @author Energy Micro AS
* @version 1.01
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2010 Energy Micro AS, http://www.energymicro.com</b>
 ******************************************************************************
 *
 * This source code is the property of Energy Micro AS. The source and compiled
 * code may only be used on Energy Micro "EFM32" microcontrollers.
 *
 * This copyright notice may not be removed from the source code nor changed.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 *****************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include "efm32.h"
#include "efm32_chip.h"
#include "efm32_emu.h"
#include "efm32_cmu.h"
#include "efm32_dma.h"
#include "efm32_adc.h"
#include "efm32_lcd.h"
//#include "segmentlcd.h"
//#include "rtc.h"
//#include "dmactrl.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

/** DMA channel used for scan sequence sampling adc channel 2, 3 and 4. */
#define DMA_CHANNEL    0
#define NUM_SAMPLES    3

/*******************************************************************************
 ***************************   LOCAL FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
* @brief
*   Configure ADC for scan mode.
*******************************************************************************/
static void ADCConfig(void)
{
  ADC_Init_TypeDef     init     = ADC_INIT_DEFAULT;
  ADC_InitScan_TypeDef scanInit = ADC_INITSCAN_DEFAULT;

  /* Init common issues for both single conversion and scan mode */
  init.timebase = ADC_TimebaseCalc(0);
  init.prescale = ADC_PrescaleCalc(7000000, 0);
  ADC_Init(ADC0, &init);

  /* Init for scan sequence use ( for dvk: accelerometer X, Y and Z axis). */
  scanInit.reference = adcRefVDD;
  scanInit.input     = ADC_SCANCTRL_INPUTMASK_CH2 |
                       ADC_SCANCTRL_INPUTMASK_CH3 |
                       ADC_SCANCTRL_INPUTMASK_CH4;
  ADC_InitScan(ADC0, &scanInit);
}


/***************************************************************************//**
* @brief
*   Configure DMA usage for this application.
*******************************************************************************/
static void DMAConfig(void)
{
  DMA_Init_TypeDef       dmaInit;
  DMA_CfgDescr_TypeDef   descrCfg;
  DMA_CfgChannel_TypeDef chnlCfg;

  /* Configure general DMA issues */
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);

  /* Configure DMA channel used */
  chnlCfg.highPri   = false;
  chnlCfg.enableInt = false;
  chnlCfg.select    = DMAREQ_ADC0_SCAN;
  chnlCfg.cb        = NULL;
  DMA_CfgChannel(DMA_CHANNEL, &chnlCfg);

  descrCfg.dstInc  = dmaDataInc4;
  descrCfg.srcInc  = dmaDataIncNone;
  descrCfg.size    = dmaDataSize4;
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CHANNEL, true, &descrCfg);
}


  /* Configure ADC and DMA used for scanning channel 2, 3 and 4 */
  ADCConfig();
  DMAConfig();

  uint32_t samples[NUM_SAMPLES];
  int      i;

  /* Stay in this loop forever */
  while (1)
  {
    DMA_ActivateBasic(DMA_CHANNEL,
                      true,
                      false,
                      samples,
                      (void *)((uint32_t) &(ADC0->SCANDATA)),
                      NUM_SAMPLES - 1);

    /* Start Scan */
    ADC_Start(ADC0, adcStartScan);

    /* Poll for completion, entering EM1 when waiting for next poll */
    while (ADC0->STATUS & ADC_STATUS_SCANACT) ;
    {
      RTC_Trigger(5, NULL);
      EMU_EnterEM1();
    }

    if (errataShift)
    {
      for (i = 0; i < NUM_SAMPLES; i++)
      {
        samples[i] <<= errataShift;
      }
    }


    /* Format numbers and write to LCD */
    char buffer[10];
    sprintf(buffer, "%02d%02d%02d", samples[0] >> 6, samples[1] >> 6, samples[2] >> 6);
   // SegmentLCD_Write(buffer);

    


