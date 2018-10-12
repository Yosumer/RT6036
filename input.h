
#ifndef __INPUT_H__
#define __INPUT_H__

#define INPUT_WALK_PULSE_PORT       gpioPortC
#define INPUT_WALK_PULSE_BIT        14
#define INPUT_WALK_PULSE_MODE       gpioModeInputPullFilter
/*
#define INPUT_LEG_FOOT_PORT         gpioPortB
#define INPUT_LEG_FOOT_BIT          3
#define INPUT_LEG_FOOT_MODE         gpioModeInput
*/
//140905
#define INPUT_LEG_FOOT_PORT         gpioPortC
#define INPUT_LEG_FOOT_BIT          7
#define INPUT_LEG_FOOT_MODE         gpioModeInput
//140906
#define INPUT_BACK_PULSE_PORT       gpioPortB
#define INPUT_BACK_PULSE_BIT        3
#define INPUT_BACK_PULSE_MODE       gpioModeInputPullFilter

//legside check
#define INPUT_LEGSIDE_SENSOR_PORT   gpioPortE
#define INPUT_LEGSIDE_SENSOR_BIT    2
#define INPUT_LEGSIDE_SENSOR_MODE   gpioModeInputPullFilter


#define KNEAD_WIDTH_UNKNOWN		0
#define KNEAD_WIDTH_MIN			1
#define KNEAD_WIDTH_MED			2
#define KNEAD_WIDTH_MAX			3

//�ﵽ����λ�õ��źŵ�ƽ
#define REACH_WALK_LIMIT    0   //hull
#define REACH_BACK_LIMIT    1   //limit switch
#define REACH_LEG_LIMIT     1   //limit switch
#define REACH_ZERO_LIMIT    1   //limit switch
#define REACH_SLIDE_LIMIT   1   //limit switch
#define REACH_FLEX_LIMIT    0   //hull

#define WALK_MOTOR_AT_MID     0
#define WALK_MOTOR_AT_BOTTOM  1
#define WALK_MOTOR_AT_TOP     2

#define LEGSTRETCH_SWITCH_ON	0			
#define LEGSTRETCH_SWITCH_OFF	1

#define LEGGROUND_SWITCH_ON	0 //����������		
#define LEGGROUND_SWITCH_OFF	1 //δ�������棬��������״̬		

#define LEGANGLE_SWITCH_ON	1 //С�������봹ֱ�ߵĽǶȳ�С��15��ʱ���Ѿ�����Σ�սǶȣ���������ǰ����
#define LEGANGLE_SWITCH_OFF	0 //С�������봹ֱ�ߵĽǶȳ�����15��ʱ��������ǰ����

#define FOOT_SWITCH_ON		0 //��������
#define FOOT_SWITCH_OFF		1

void Input_Initial_IO(void);
void Input_5ms_Int(void);
void Input_Proce(void);
unsigned int Input_GetBackUpSwitch(void);
unsigned int Input_GetBackDownSwitch(void);
unsigned int Input_GetLegUpSwitch(void);
unsigned int Input_GetLegDownSwitch(void);
unsigned int Input_GetWalkUpSwitch(void);
unsigned int Input_GetWalkDownSwitch(void);
unsigned int Input_GetVout(void);
unsigned int Input_GetKneadPosition(void);
unsigned int Input_GetWalkPulse(void);
unsigned int Input_GetFlexGroundSwitch(void);
unsigned int Input_GetKneadMax(void);
unsigned int Input_GetKneadMid(void);
unsigned int Input_GetKneadMin(void);
unsigned int Input_GetMp3Status(void);
unsigned int Input_PowerCheck(void);
bool Input_GetWalkChange(void);
void Input_ClearWalkChange(void);
void Input_SetWalkMotorPosition(unsigned short locate);
unsigned short Input_GetWalkMotorPosition(void);
unsigned int Input_GetWalkPosition(void);
unsigned int Input_GetValveFault(void);
unsigned int Input_GetLedFault(void);
unsigned int Input_GetSlideForwardSwitch(void);
unsigned int Input_GetSlideBackwardSwitch(void);
unsigned int Input_GetFlexOutSwitch(void);
unsigned int Input_GetFlexInSwitch(void);
unsigned int Input_GetFlexFootSwitch(void);
unsigned int Input_GetFlexAngleSwitch(void);
void Input_SetCounterWalkMotorPosition(unsigned short locate); 
unsigned short Input_GetCounterWalkMotorPosition(void);
unsigned char Sensor_Check(void);
unsigned int Input_GetBackPosition(void);
void Input_SetBackMotorPosition(unsigned int Position);
void Input_Back_Pulse1MS(void);

#endif /*__INPUT_H__*/
