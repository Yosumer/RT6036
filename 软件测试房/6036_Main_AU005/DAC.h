#ifndef __DAC_H__
#define __DAC_H__

#define DAC_CURRENT_1A      620
#define DAC_CURRENT_1_5A    930
#define DAC_CURRENT_2A      1241
#define DAC_CURRENT_2_5A    1552
#define DAC_CURRENT_3A      1862
#define DAC_CURRENT_3_5A    2172
#define DAC_CURRENT_4A      2482
#define DAC_CURRENT_4_5A    2793
#define DAC_CURRENT_5A      3103
#define DAC_CURRENT_5_5A    3413
#define DAC_CURRENT_MAX_LEVEL    9

void DAC_Initial_IO(void);

void DAC_SetKneadCurrent(unsigned int current);
unsigned int DAC_GetKneadCurrent(void);
void DAC_SetKnockCurrent(unsigned int current);
unsigned int DAC_GetKnockCurrent(void);

#endif 