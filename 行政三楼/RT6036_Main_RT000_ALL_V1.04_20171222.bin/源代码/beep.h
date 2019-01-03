
#define C_BEEP_STOP     0
#define C_BEEP_SLOW     1
#define C_BEEP_FAST     2
#define C_BEEP_ONETIME  3
#define C_BEEP_TWOTIME  4

/*
#define BUZZER_MODE_OFF			0
#define BUZZER_MODE_SLOW		1   //电动缸正常运转提示音
#define BUZZER_MODE_FAST		2   //电动缸到位置提示音
#define BUZZER_MODE_ONETIME		3   //按键有效提示音
#define BUZZER_MODE_TWOTIME		4   //按键无效提示音 
*/

extern unsigned char by_BeepStatus;
void Beep_Set(unsigned char action);
unsigned char Beep_Get_Status(void);