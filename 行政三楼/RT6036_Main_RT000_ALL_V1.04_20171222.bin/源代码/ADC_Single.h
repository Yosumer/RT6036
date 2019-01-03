#ifndef __ADC_H__
#define __ADC_H__


#define ADC_V24         0
#define ADC_VCC         1
#define ADC_V24_1       2
#define ADC_AUDIO_L     3
#define ADC_AUDIO_R     4
#define ADC_VDDCPU      5
#define ADC_TEMP        6

#define ADC_INITSINGLE_RT8600                                                      \
  { adcPRSSELCh0,              /* PRS ch0 (if enabled). */                          \
    adcAcqTime1,               /* 1 ADC_CLK cycle acquisition time. */              \
    adcRef1V25,                /* 1.25V internal reference. */                      \
    adcRes8Bit,               /* 12 bit resolution. */                             \
    adcSingleInpCh4,           /* CH4 input selected. */                            \
    false,                     /* Single ended input. */                            \
    false,                     /* PRS disabled. */                                  \
    false,                     /* Right adjust. */                                  \
    false                      /* Deactivate conversion after one scan sequence. */ \
  }

void ADC_1ms_Int(void);
void ADC_Data_Init(void);
//unsigned char ADC_Get_ADC(void);
int ADC_Get_ADC(unsigned char channel,unsigned short* ADC);
unsigned short ADC_Get_Voltage(unsigned char channel,unsigned short* voltage);
unsigned short ADC_Get_Voltage(unsigned char channel,unsigned short* voltage);
unsigned int ADC_Get_Inttemp(void);
#endif /*__ADC_H__*/