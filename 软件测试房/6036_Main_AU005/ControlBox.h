#include "MassageStatus.h"

#define SOI                                             0XF0
#define EOI                                             0XF1
#define CONTROL_CODE                                    0X82

#define NORMAL_CTRL                                     CONTROL_CODE
#define ENGGER_CTRL                                     0xBD

#define H10_KEY_NONE			                0x7f
//Power Switch Key
#define H10_KEY_POWER_SWITCH			        0x01
#define H10_KEY_MENU                                    0x02
//Massage Chair Reset to the original Position
#define H10_KEY_RESET_CONFIRM                           0x04
#define H10_KEY_RESET_CANCLE                            0x05
//ZONE1:自动程序键值
#define H10_KEY_CHAIR_AUTO_0			        0x10 //疲劳恢复 recovery
#define H10_KEY_CHAIR_AUTO_1			        0x11 //舒展按摩 extend
#define H10_KEY_CHAIR_AUTO_2			        0x12 //轻松按摩 relax
#define H10_KEY_CHAIR_AUTO_3			        0x13 //酸痛改善 refresh
#define H10_KEY_CHAIR_AUTO_4                            0x14
#define H10_KEY_CHAIR_AUTO_5                            0x15
#define H10_KEY_CHAIR_UP_BACK                           H10_KEY_CHAIR_AUTO_4 
#define H10_KEY_CHAIR_DOWN_BACK	                        H10_KEY_CHAIR_AUTO_5
#define H10_KEY_NECK_SHOULDER_AUTO                      H10_KEY_CHAIR_UP_BACK
#define H10_KEY_BACK_WAIST_AUTO                         H10_KEY_CHAIR_DOWN_BACK


//ZONE2:手动程序键值
/**********************************************************/
#define H10_KEY_KNEAD					0x20 
#define H10_KEY_KNOCK					0x21
#define H10_KEY_PRESS					0x22
#define H10_KEY_SOFT_KNOCK				0x23
#define H10_KEY_WAVELET					0x24 
#define H10_KEY_RUBBING					0x77 
#define H10_KEY_MANUAL					0x25 
#define H10_KEY_MUSIC                                   0x26
//#define H10_KEY_HEAT_OFF  				0x26 //背部加热关
#define H10_KEY_HEAT_ON  				0x27 //背部加热开
#define H10_KEY_HEAT  				        0x27 //背部加热开

//#define H10_KEY_LOCATE_AREA       0x28
//case H10_KEY_LOCATE_POINT :
//case KEY_LOCATE_PARTIAL://

#define H10_KEY_LOCATE_PART			        0x28 //按摩部位局部
#define H10_KEY_LOCATE_POINT				0x29 //按摩部位定点
#define H10_KEY_LOCATE_FULL				0x2A //按摩部位全程

#define H10_KEY_WIDTH_MIN				0x2B
#define H10_KEY_WIDTH_MED				0x2C
#define H10_KEY_WIDTH_MAX				0x2D

#define H10_KEY_SPEED_1                                 0x2E //按摩速度1
#define H10_KEY_SPEED_2                                 0x2F //按摩速度2
#define H10_KEY_SPEED_3                                 0x30 //按摩速度3
#define H10_KEY_SPEED_4                                 0x31 //按摩速度4
#define H10_KEY_SPEED_5                                 0x32 //按摩速度5
#define H10_KEY_SPEED_6                                 0x33 //按摩速度6

#define H10_KEY_WHEEL_SPEED_SLOW                        0x34 //滚轮速度慢
#define H10_KEY_WHEEL_SPEED_MED                         0x35 //滚轮速度中
#define H10_KEY_WHEEL_SPEED_FAST                        0x36 //滚轮速度快
#define H10_KEY_WHEEL_SPEED_OFF                         0x37 //滚轮关

#define H10_KEY_SPEED_INCREASE                          0x38
#define H10_KEY_SPEED_DECREASE                          0x39


//ZONE3:气囊设置程序键值
#define H10_KEY_AIRBAG_LEG			        0x40  //小腿
#define H10_KEY_AIRBAG_ARM			        0x41  //臂肩
#define H10_KEY_AIRBAG_WAIST			        0x42  //背腰
#define H10_KEY_AIRBAG_BUTTOCKS			        0x43  //臀部
#define H10_KEY_AIRBAG_AUTO			        0x44  //全身自动

#define H10_KEY_AIRBAG_STRENGTH_1                       0x45  //气囊力度1
#define H10_KEY_AIRBAG_STRENGTH_WEAK                    0x45  //气囊力度弱
#define H10_KEY_AIRBAG_STRENGTH_2                       0x46  //气囊力度2
#define H10_KEY_AIRBAG_STRENGTH_3                       0x47  //气囊力度3
#define H10_KEY_AIRBAG_STRENGTH_MIDDLE                  0x47  //气囊力度中
#define H10_KEY_AIRBAG_STRENGTH_4                       0x48  //气囊力度4
#define H10_KEY_AIRBAG_STRENGTH_5                       0x49  //气囊力度5
#define H10_KEY_AIRBAG_STRENGTH_STRONG                  0x49  //气囊力度强
#define H10_KEY_AIRBAG_STRENGTH_OFF                     0x4A  //气囊关

//ZONE4:设置程序键值
#define H10_KEY_WORK_TIME_10MIN                         0x50  //按摩10分钟
#define H10_KEY_WORK_TIME_20MIN                         0x51  //按摩20分钟
#define H10_KEY_WORK_TIME_30MIN                         0x52  //按摩30分钟

#define H10_KEY_BACK_LIGHT_SLOW                         0x55  //背光强度弱
#define H10_KEY_BACK_LIGHT_MED                          0x56  //背光强度中
#define H10_KEY_BACK_LIGHT_HIGH                         0x57  //背光强度强

//ZONE5:机芯位置控制键值
#define H10_KEY_WALK_UP_START				0x60
#define H10_KEY_WALK_UP_STOP				0x61
#define H10_KEY_WALK_DOWN_START				0x62
#define H10_KEY_WALK_DOWN_STOP				0x63

//ZONE6:靠背电动缸控制键值
#define H10_KEY_BACKPAD_UP_START			0x64
#define H10_KEY_BACKPAD_UP_STOP				0x65
#define H10_KEY_BACKPAD_DOWN_START			0x66
#define H10_KEY_BACKPAD_DOWN_STOP			0x67

//ZONE7:小腿电动缸控制键值
#define H10_KEY_LEGPAD_UP_START				0x68
#define H10_KEY_LEGPAD_UP_STOP				0x69
#define H10_KEY_LEGPAD_DOWN_START			0x6A
#define H10_KEY_LEGPAD_DOWN_STOP			0x6B
/*********************fww***************************
#define H10_KEY_LEGPAD_EXTEND_START                     0x6C
#define H10_KEY_LEGPAD_EXTEND_STOP                      0x6D
#define H10_KEY_LEGPAD_CONTRACT_START                   0x6E
#define H10_KEY_LEGPAD_CONTRACT_STOP                    0x6F
********************fww******************************/
#define  H10_KEY_RESET                                  0x7E

//ZONE8:零重力键值
#define H10_KEY_ZERO_START                              0x70
//
#define H10_KEY_WIDTH_INCREASE                          0x90
#define H10_KEY_WIDTH_DECREASE                          0x91
#define H10_KEY_BLUETOOTH_POWER_SWITCH  		0x53

/************************************************/
#define nMaunalSubMode_NO_ACTION	0
#define nMaunalSubMode_PRESS		1
#define nMaunalSubMode_RUBBING		2
#define nMaunalSubMode_SOFT_KNOCK	3
#define nMaunalSubMode_KNOCK		4
#define nMaunalSubMode_KNEAD		5
#define nMaunalSubMode_WAVELET		6
#define nMaunalSubMode_MUSIC		7

/************************************************/

#define LED_OFF			        0 
#define LED_ON			        1

#define BUZZER_MODE_OFF			0
#define BUZZER_MODE_SLOW		1
#define BUZZER_MODE_FAST		2
#define BUZZER_MODE_ONETIME		3
#define BUZZER_MODE_TWOTIME		4

#define ENG_CMD_RESET                   1
#define ENG_CMD_DEC_STRENGTH            2
#define ENG_CMD_ADD_STRENGTH            3
#define ENG_CMD_SLIDE                   4
#define ENG_CMD_STRETCH_MODE            5
#define ENG_CMD_SLEEP_PROGRAM           6