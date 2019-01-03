/******************************************************************************
* @file
* @brief ADC single conversion example
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
#include <stdio.h>
#include "efm32.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_adc.h"
#include "em_prs.h"
#include <stdio.h>
#include "ADC_Single.h"

#define prsChannel 0

//ADC顺序
bool bADCDone = false;
#define ADC_CH_V24         adcSingleInpCh3
#define ADC_CH_VCC         adcSingleInpCh4
#define ADC_CH_V24_1       adcSingleInpCh5
#define ADC_CH_AUDIO_L     adcSingleInpCh6
#define ADC_CH_AUDIO_R     adcSingleInpCh7
#define ADC_CH_VDDCPU      adcSingleInpVDD
#define ADC_CH_TEMP        adcSingleInpTemp

typedef struct
{
  ADC_SingleInput_TypeDef channel; 
  bool  active; 
} st_ADC;                                               

st_ADC st_adc[7] = {
  {ADC_CH_V24,true},
  {ADC_CH_VCC,true},
  {ADC_CH_V24_1,true},
  {ADC_CH_AUDIO_L,true},
  {ADC_CH_AUDIO_R,true},
  {ADC_CH_VDDCPU,true},
  {ADC_CH_TEMP,true}
};

#define ADC_CHANNEL_LENGTH (sizeof(st_adc)/sizeof(st_ADC))

unsigned short w_ADC_Result[ADC_CHANNEL_LENGTH];

unsigned short adcResult;  //12位
unsigned char adc_Step;
ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_RT8600;
/**************************************************************************//**
 * @brief ADC0_IRQHandler
 * Interrupt Service Routine for ADC
 *****************************************************************************/
void ADC0_IRQHandler(void)
{
  /* Clear ADC0 interrupt flag */
  ADC0->IFC = 1;
  /* Read conversion result to clear Single Data Valid flag */
  adcResult = ADC_DataSingleGet(ADC0);
  w_ADC_Result[adc_Step] = adcResult;
  bADCDone = true;
}

uint32_t ADC_Calibration(ADC_TypeDef *adc, ADC_Ref_TypeDef ref)
{
  int32_t  sample;
  uint32_t cal;

  /* Binary search variables */
  uint8_t high;
  uint8_t mid;
  uint8_t low;

  /* Reset ADC to be sure we have default settings and wait for ongoing */
  /* conversions to be complete. */
  ADC_Reset(adc);

  ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

  /* Init common settings for both single conversion and scan mode */
  init.timebase = ADC_TimebaseCalc(0);
  /* Might as well finish conversion as quickly as possibly since polling */
  /* for completion. */
  /* Set ADC clock to 7 MHz, use default HFPERCLK */
  init.prescale = ADC_PrescaleCalc(1000000, 0);

  /* Set an oversampling rate for more accuracy */
  init.ovsRateSel = adcOvsRateSel8;
  /* Leave other settings at default values */
  ADC_Init(adc, &init);

  /* Init for single conversion use, measure diff 0 with selected reference. */
  singleInit.reference = ref;
  singleInit.input     = adcSingleInpDiff0;
  singleInit.acqTime   = adcAcqTime16;
  singleInit.diff      = true;
  /* Enable oversampling rate */
  singleInit.resolution = adcResOVS;

  ADC_InitSingle(adc, &singleInit);

  /* ADC is now set up for offset calibration */
  /* Offset calibration register is a 7 bit signed 2's complement value. */
  /* Use unsigned indexes for binary search, and convert when calibration */
  /* register is written to. */
  high = 128;
  low  = 0;

  /* Do binary search for offset calibration*/
  while (low < high)
  {
    /* Calculate midpoint */
    mid = low + (high - low) / 2;

    /* Midpoint is converted to 2's complement and written to both scan and */
    /* single calibration registers */
    cal      = adc->CAL & ~(_ADC_CAL_SINGLEOFFSET_MASK | _ADC_CAL_SCANOFFSET_MASK);
    cal     |= (mid - 63) << _ADC_CAL_SINGLEOFFSET_SHIFT;
    cal     |= (mid - 63) << _ADC_CAL_SCANOFFSET_SHIFT;
    adc->CAL = cal;

    /* Do a conversion */
    ADC_Start(adc, adcStartSingle);

    /* Wait while conversion is active */
    while (adc->STATUS & ADC_STATUS_SINGLEACT) ;

    /* Get ADC result */
    sample = ADC_DataSingleGet(adc);

    /* Check result and decide in which part of to repeat search */
    /* Calibration register has negative effect on result */
    if (sample < 0)
    {
      /* Repeat search in bottom half. */
      high = mid;
    }
    else if (sample > 0)
    {
      /* Repeat search in top half. */
      low = mid + 1;
    }
    else
    {
      /* Found it, exit while loop */
      break;
    }
  }

  /* Now do gain calibration, only input and diff settings needs to be changed */
  adc->SINGLECTRL &= ~(_ADC_SINGLECTRL_INPUTSEL_MASK | _ADC_SINGLECTRL_DIFF_MASK);
  adc->SINGLECTRL |= (adcSingleInpCh4 << _ADC_SINGLECTRL_INPUTSEL_SHIFT);
  adc->SINGLECTRL |= (false << _ADC_SINGLECTRL_DIFF_SHIFT);

  /* ADC is now set up for gain calibration */
  /* Gain calibration register is a 7 bit unsigned value. */

  high = 128;
  low  = 0;

  /* Do binary search for gain calibration */
  while (low < high)
  {
    /* Calculate midpoint and write to calibration register */
    mid = low + (high - low) / 2;

    /* Midpoint is converted to 2's complement */
    cal      = adc->CAL & ~(_ADC_CAL_SINGLEGAIN_MASK | _ADC_CAL_SCANGAIN_MASK);
    cal     |= mid << _ADC_CAL_SINGLEGAIN_SHIFT;
    cal     |= mid << _ADC_CAL_SCANGAIN_SHIFT;
    adc->CAL = cal;

    /* Do a conversion */
    ADC_Start(adc, adcStartSingle);

    /* Wait while conversion is active */
    while (adc->STATUS & ADC_STATUS_SINGLEACT) ;

    /* Get ADC result */
    sample = ADC_DataSingleGet(adc);

    /* Check result and decide in which part to repeat search */
    /* Compare with a value atleast one LSB's less than top to avoid overshooting */
    /* Since oversampling is used, the result is 16 bits, but a couple of lsb's */
    /* applies to the 12 bit result value, if 0xffe is the top value in 12 bit, this */
    /* is in turn 0xffe0 in the 16 bit result. */
    /* Calibration register has positive effect on result */
    if (sample > 0xffd0)
    {
      /* Repeat search in bottom half. */
      high = mid;
    }
    else if (sample < 0xffd0)
    {
      /* Repeat search in top half. */
      low = mid + 1;
    }
    else
    {
      /* Found it, exit while loop */
      break;
    }
  }

  return adc->CAL;
}


/***************************************************************************//**
* @brief
*   Configure ADC usage for this application.
*******************************************************************************/
static void ADCConfig(void)
{
  /*
  PRS_LevelSet(0, 1 << (prsChannel + _PRS_SWLEVEL_CH0LEVEL_SHIFT));
  PRS_SourceSignalSet(prsChannel,
                      PRS_CH_CTRL_SOURCESEL_TIMER0,
                      PRS_CH_CTRL_SIGSEL_TIMER0OF,
                      prsEdgePos);
  */
  ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
  
  /* Init common settings for both single conversion and scan mode */
  init.timebase = ADC_TimebaseCalc(0);
  /* Might as well finish conversion as quickly as possibly since polling */
  /* for completion. */
  /* Set ADC clock to 7 MHz, use default HFPERCLK */
  init.prescale = ADC_PrescaleCalc(1000000, 0);
  
  /* WARMUPMODE must be set to Normal according to ref manual before */
  /* entering EM2. In this example, the warmup time is not a big problem */
  /* due to relatively infrequent polling. Leave at default NORMAL, */
  
  ADC_Init(ADC0, &init);
  
  /* Init for single conversion use, measure VDD/3 with 1.25 reference. */
  singleInit.reference  = adcRefVDD;
  adc_Step = 0;
  singleInit.input = st_adc[0].channel;
  singleInit.resolution = adcRes12Bit;
  /* The datasheet specifies a minimum aquisition time when sampling vdd/3 */
  /* 32 cycles should be safe for all ADC clock frequencies */
  singleInit.acqTime = adcAcqTime32;
  ADC_InitSingle(ADC0, &singleInit);
  
  
  /*
  uint8_t  offset_calibration_value;
  uint8_t  gain_calibration_value;
  uint32_t calibration_value;

  uint32_t old_gain_calibration_value =
    (DEVINFO->ADC0CAL0 & _DEVINFO_ADC0CAL0_1V25_GAIN_MASK)
    >> _DEVINFO_ADC0CAL0_1V25_GAIN_SHIFT;

  uint32_t old_offset_calibration_value =
    (DEVINFO->ADC0CAL0 & _DEVINFO_ADC0CAL0_1V25_OFFSET_MASK)
    >> _DEVINFO_ADC0CAL0_1V25_OFFSET_SHIFT;

 // RTC_Trigger(100, NULL);

  calibration_value = ADC_Calibration(ADC0, adcRefVDD);

  offset_calibration_value = (calibration_value & _ADC_CAL_SINGLEOFFSET_MASK) >> _ADC_CAL_SINGLEOFFSET_SHIFT;
  gain_calibration_value   = (calibration_value & _ADC_CAL_SINGLEGAIN_MASK) >> _ADC_CAL_SINGLEGAIN_SHIFT;
*/
  
  
  bADCDone = true;
}

void ADC_1ms_Int(void)
{
 // if(!bADCDone) return;
  
  int length = ADC_CHANNEL_LENGTH;
  while(length--)
  {
    adc_Step++;  
    adc_Step %= ADC_CHANNEL_LENGTH;
    if(st_adc[adc_Step].active)
    {
      singleInit.input = st_adc[adc_Step].channel;
      ADC_InitSingle(ADC0, &singleInit);
      bADCDone = false;
      ADC_Start(ADC0, adcStartSingle);
      break;
    }
  }
}

void ADC_Data_Init(void)
{
  ADCConfig();
  /* Enable ADC Interrupt when Single Conversion Complete */
  ADC0->IEN = ADC_IEN_SINGLE;

  /* Enable ADC interrupt vector in NVIC*/
  NVIC_EnableIRQ(ADC0_IRQn);
}

int ADC_Get_ADC(unsigned char channel,unsigned short* ADC)
{
  int retval  = -1;
  
  switch(channel)
  {
  case ADC_V24:*ADC = w_ADC_Result[ADC_V24]; break;
  case ADC_VCC:*ADC = w_ADC_Result[ADC_VCC]; break;
  case ADC_V24_1:*ADC = w_ADC_Result[ADC_V24_1]; break;
  case ADC_AUDIO_L:*ADC = w_ADC_Result[ADC_AUDIO_L]; break;
  case ADC_AUDIO_R:*ADC = w_ADC_Result[ADC_AUDIO_R]; break;
  case ADC_VDDCPU:*ADC = w_ADC_Result[ADC_VDDCPU]; break;
  case ADC_TEMP:*ADC = w_ADC_Result[ADC_TEMP]; break;
  default: return retval;
  }
  retval = 1;
  return retval;
}
unsigned short ADC_Convert_Voltage(unsigned short adc,unsigned short vref)
{
  unsigned int reault;
        reault = adc;
        reault *= 3630;  //4.7K电阻和47K电阻 扩大11倍，并且还要扩大100倍
        reault /= vref;  //此处的电压值扩大了100倍
        return((unsigned short)reault);
}


/**************************************************************************//**
 * @brief Convert ADC sample values to celsius.
 * @note See section 2.3.4 in the reference manual for details on this
 *       calculatoin
 * @param adcSample Raw value from ADC to be converted to celsius
 * @return The temperature in degrees Celsius.
 *****************************************************************************/
float convertToCelsius(uint32_t adcSample)
{
  float temp;
  /* Factory calibration temperature from device information page. */
  float cal_temp_0 = (float)((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK) 
                            >> _DEVINFO_CAL_TEMP_SHIFT);
  
  float cal_value_0 = (float)((DEVINFO->ADC0CAL2 
                            & _DEVINFO_ADC0CAL2_TEMP1V25_MASK) 
                            >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);

  /* Temperature gradient (from datasheet) */
  float t_grad = -6.27;
  
  temp = (cal_temp_0 - ((cal_value_0 - adcSample)  / t_grad));
  
  return temp;
}

unsigned int ADC_Get_Inttemp(void)
{
  unsigned int intTemp = (unsigned int)(convertToCelsius(w_ADC_Result[ADC_TEMP]) * 10);
  return(intTemp);
}


unsigned short ADC_Get_Voltage(unsigned char channel,unsigned short* voltage)
{
  int retval  = -1;
  
  switch(channel)
  {
   case ADC_V24:
    *voltage = ADC_Convert_Voltage(w_ADC_Result[ADC_V24],w_ADC_Result[ADC_VDDCPU]);
    break;
   case ADC_VCC:
    *voltage = ADC_Convert_Voltage(w_ADC_Result[ADC_VCC],w_ADC_Result[ADC_VDDCPU]);
    break;
   case ADC_V24_1:
    *voltage = ADC_Convert_Voltage(w_ADC_Result[ADC_V24_1],w_ADC_Result[ADC_VDDCPU]);
    break;
    
  case ADC_AUDIO_L:
  case ADC_AUDIO_R:
  case ADC_VDDCPU:
  case ADC_TEMP:
  default: return retval;
  }
  retval = 1;
  return retval;
}



