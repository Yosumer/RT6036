#include "efm32.h"
#include "efm32_chip.h"
#include "efm32_gpio.h"
#include "Beep.h"
unsigned char by_BeepStatus = C_BEEP_STOP;    //指示Beep的状态
unsigned char by_BeepCurStatus = C_BEEP_STOP; //指示当前Beep的状态，读走后
bool bBeepStatus = 0;
void Beep_Set(unsigned char action)
{
  switch(action)
  {
   default: break; 
   case C_BEEP_STOP:     
          //if(by_BeepStatus == C_BEEP_STOP) break;
          bBeepStatus = 1;
          by_BeepStatus = C_BEEP_STOP;
          break;         
   case C_BEEP_ONETIME:  
          //if(by_BeepStatus == C_BEEP_ONETIME) break;
          bBeepStatus = 1;
          by_BeepStatus = C_BEEP_ONETIME;
          break;
   case C_BEEP_FAST:  
          //if(by_BeepStatus == C_BEEP_FAST) break;
          bBeepStatus = 1;
          by_BeepStatus = C_BEEP_FAST;
          break;       
   case C_BEEP_SLOW:  
          //if(by_BeepStatus == C_BEEP_SLOW) break;
          bBeepStatus = 1;
          by_BeepStatus = C_BEEP_SLOW;
          break;             
   case C_BEEP_TWOTIME:  
          //if(by_BeepStatus == C_BEEP_TWOTIME) break;
          by_BeepStatus = C_BEEP_TWOTIME;
          bBeepStatus = 1;
          break;
  }
}

unsigned char Beep_Get_Status(void)
{
  return(bBeepStatus);
}

void  Beep_Clear_Status(void)
{
  bBeepStatus = 0;
}

