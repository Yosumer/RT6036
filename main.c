#define SIGMA

#define   VFD_POWER_ON      1
#define   VFD_POWER_OFF     0
//#define USER_DATA_BASE         ((uint32_t) 0x0000FFFBUL)//0x0FE00000UL)  /**< user data flash base address  */
//#define TEST_VALVE
#define FOOT_ROLLER_ENABLE
//#define SN_CHECK_EN
//#define TEST_ON 1
#define BACK_MODE
//#define FORCE_CONTROLLER  1
#define POSITION_CTRL_OFFSET        50
#define POSITION_DISPLAY_OFFSET     80

#define RELEASE
//#define RUBBING_MANUAL_TEST 1
#define C_TIMER_TEMP        0
#define C_TIMER_FAST        1
#define C_TIMER_SLOW        2
#define C_TIMER_MP3         3
#define C_TIMER_ENG1        3   //用于工程模式
#define C_TIMER_WAVE_START  4
#define C_TIMER_POWER       5
#define C_TIME_RUBBING      6 //用于搓背程序
#define C_TIMER_INDICATE    7
#define C_TIMER_RESET_TIME  8
#define C_TIMER_ENG2        0   //用于工程模式
#ifdef TEST_VALVE
unsigned int ValveTestData = 0xffffff;
#endif
//#define OTG_MP3

//#define SINGLE_HEADER //单头丝杠
//#define DOUBLE_HEADER //双头丝杠

//ON/OFF键收藏功能配置
#define POWER_SETTLE_FUNCTION

//#define POWER_VIBRATION
////定义开机振动

//Time Over收藏功能配置
//#define TIMEOVER_SETTLE_FUNCTION

//TOP定位方式
//#define TOP_BY_LIMIT //最高点由行程开关决定
#define TOP_BY_NECK //最高点由颈部决定
#include "em_cmu.h"
#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "EFM32_def.h"
#include "EFM32_types.h"
#include "backaction.h"
#include "airbagaction.h"
#include "vibration.h"
#include "system.h"
#include "IndicateLED.h"
#include "Hot_Roller.h"
#include "KneadMotor.h"
#include "KnockMotor.h"
#include "Valve.h"
#include "Timer.h"
#include "FlexPad.h"
#include "WalkMotor.h"
#include "BackPad.h"
#include "LegMotor.h"
#include "SlideMotor.h"
#include "input.h"
#include "power.h"
#include "DMAUart.h"
#include "BlueTooth.h"
#include "ControlBox.h"
#include "WaistHot.h"
#include "Mp3Ctrl.h"
#include "em_msc.h"
#include <string.h>
#include <stdio.h>
#include "beep.h"
#include "Data_Cul.h"
#include "ADC_Single.h"
#include "DAC.h"
#include "em_wdog.h"
#include "LED_RGB.h"
#include "memory.h"
#include "MassageStatus.h"
#include "testCommand.h"
//#include "FlexPad.h"
#include "main.h"
//#include "MotorPosCheckRx.h"
#include "BlueTooth.h"
#include "em_aes.h"
//位结构定义

#define MAX_MP3_TIME	10 	//100ms for OTG MP3 command
#define MP3_POWER_ON 	1
#define MP3_POWER_OFF	0
#define MP3_STOP		0
#define MP3_PLAY		1
#define MP3_OK			1
#define MP3_ERROR		0

#define 	WalkUpIdlesse	20  //20*10ms 零重力马达，继电器吸合到MOSFET吸合的时间
#define 	WalkDownIdlesse	150

#define		ADSTRONG1  20
#define		ADSTRONG2  80
#define		ADSTRONG3  120
#define		ADSTRONG4  200
#define		ADSTRONG5  600
#define		ADSTRONG6  1000

/***************************************/
//Communication
#define MAX_INBUFFER_COUNT			10
#define MAX_OUTBUFFER_COUNT			20
#define MAX_RETRY_TIMES				2 //最大重试次数(不计算第1次的正常发送次数,实际发送的次数为MAX_RETRY_TIMES+1)
#define MIN_PACKET_INTERVAL 		2 //最小Packet间隔，单位：10ms
#define MAX_WAIT_MASTER_PACKET_TIME     200 //Test Value:200;Release Value:10-20
#define MAX_WAIT_COMMAND_TIME		120 //120*0.5s=60s=1min
#define INIT_WAIT_COMMAND_TIME		30 //120*0.5s=60s=1min

//Master->Slave Packet Type
#define PACKET_MASTER_GET_COMMAND 	0x00
#define PACKET_MASTER_ACK_COMMAND 	0x01
#define PACKET_MASTER_SET_STATE   	0x02
//Slave->Master Packet Type
#define PACKET_SLAVE_SEND_COMMAND 	0x03

//Timer
#define RATED_WORK_TIME				20	  //20  //Unit:Minutes
#define FUNCTION_SAVE_WAIT_TIME		100   //Unit:100ms
/****************************************************/
#define Zero_MOTOR_STATE_IDLE		0
#define Zero_MOTOR_STATE_ANTICLOCK	1
#define Zero_MOTOR_STATE_CLOCK		2
#define Zero_StartMotor				0
#define Zero_RunMotor				1
#define Zero_StopMotor				2
/****************************************************/
//手控板显示和蜂鸣器
//H10_VFD_LED_BUZZER_STRUCT VfdLedBuzzer ;
__no_init StretchStruct st_Stretch;
/********************************/
#define bLedFullBackAutoMode0 		VfdLedBuzzer.bAuto1
#define bLedFullBackAutoMode1		VfdLedBuzzer.bAuto2
#define bLedFullBackAutoMode3		VfdLedBuzzer.bAuto3
#define bLedFullBackAutoMode2		VfdLedBuzzer.bAuto4
/******************************************************/
const unsigned char LcdSeg[] =
{
    0x3f, 0x06, 0x5b, 0x4f, //0,1,2,3
    0x66, 0x6d, 0x7d, 0x07, //4,5,6,7
    0x7f, 0x67, 0x00, 0x00, //8,9,A,B
    0x79, 0x79, 0x79, 0x71, //c,d,e,f
} ;

#define AUTO_FUNCTION_0_STEPS	sizeof(AutoFunction0)/sizeof(struct Walk_Knead_Knock_Motor_Struct_Auto)
#define AUTO_FUNCTION_1_STEPS	sizeof(AutoFunction1)/sizeof(struct Walk_Knead_Knock_Motor_Struct_Auto)
#define AUTO_FUNCTION_2_STEPS	sizeof(AutoFunction2)/sizeof(struct Walk_Knead_Knock_Motor_Struct_Auto)
#define AUTO_FUNCTION_3_STEPS	sizeof(AutoFunction3)/sizeof(struct Walk_Knead_Knock_Motor_Struct_Auto)
#define AUTO_FUNCTION_4_STEPS	sizeof(AutoFunction4)/sizeof(struct Walk_Knead_Knock_Motor_Struct_Auto)
#define AUTO_FUNCTION_5_STEPS	sizeof(AutoFunction5)/sizeof(struct Walk_Knead_Knock_Motor_Struct_Auto)

//自动程序步骤总数
const unsigned int BACK_AUTO_STEPS[] =
{
    AUTO_FUNCTION_0_STEPS,
    AUTO_FUNCTION_1_STEPS,
    AUTO_FUNCTION_2_STEPS,
    AUTO_FUNCTION_3_STEPS,
    AUTO_FUNCTION_4_STEPS,
    AUTO_FUNCTION_5_STEPS,
} ;
//自动程序循环运行开始的步骤
const unsigned char BACK_AUTO_START_STEP[] =
{
    0,
    0,
    0,
    0,
    0,
    0,
} ;

#define AIRBAG_MODE_ARM_NECK_STEPS		    sizeof(AirBagModeArmNeck)/sizeof(struct AirBagStruct)
#define AIRBAG_MODE_BODY_UP_STEPS			sizeof(AirBagModeBodyUp)/sizeof(struct AirBagStruct)
#define AIRBAG_MODE_LEG_FOOT_STEPS		    sizeof(AirBagModeLegFoot)/sizeof(struct AirBagStruct)
//#define AIRBAG_MODE_BODY_UP_ARM_SHOULDER_STEPS	sizeof(AirBagModeBodyUpArmShoulder)/sizeof(struct AirBagStruct)
//#define AIRBAG_MODE_ARM_STEPS			        sizeof(AirBagModeArm)/sizeof(struct ArmAirBagStruct)

/* Defining the watchdog initialization data */
WDOG_Init_TypeDef WDTinit =
{
    .enable     = true,               /* Start watchdog when init done */
    .debugRun   = false,              /* WDOG not counting during debug halt */
    .em2Run     = true,               /* WDOG counting when in EM2 */
    .em3Run     = true,               /* WDOG counting when in EM3 */
    .em4Block   = false,              /* EM4 can be entered */
    .swoscBlock = false,              /* Do not block disabling LFRCO/LFXO in CMU */
    .lock       = false,              /* Do not lock WDOG configuration (if locked, reset needed to unlock) */
    .clkSel     = wdogClkSelULFRCO,   /* Select 1kHZ WDOG oscillator */
    .perSel     = wdogPeriod_2k,      /* Set the watchdog period to 2049 clock periods (ie ~2 seconds)*/
};

unsigned char engineeringTime_10msFlag = 0; //工程模式使用
unsigned char sendBluetoothCMD = 0;
unsigned char bluetoothCountDown = 20;

__no_init WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO AutoDirector ;
__no_init WALK_KNEAD_KNOCK_MOTOR_STRUCT_MANUAL ManualDirector[4] ;

__no_init BITS GlobalFlags0 ;
#define bKneadWidthMaxPulseLevel0 	GlobalFlags0.bD0
#define bKneadWidthMaxPulseLevel1 	GlobalFlags0.bD1
#define bKneadWidthMaxPulseLevel2 	GlobalFlags0.bD2
#define bKneadWidthMaxPulseLevel3 	GlobalFlags0.bD3
#define bHasKneadWidthMaxPulse	 	GlobalFlags0.bD4
#define bDisplayKneadWidthMax		GlobalFlags0.bD5
#define bDisplayKneadTrackMax		GlobalFlags0.bD6
#define bUpdateLocate 			GlobalFlags0.bD7

__no_init BITS GlobalFlags1 ;
#define bKneadWidthMedPulseLevel0 	GlobalFlags1.bD0
#define bKneadWidthMedPulseLevel1 	GlobalFlags1.bD1
#define bKneadWidthMedPulseLevel2	GlobalFlags1.bD2
#define bKneadWidthMedPulseLevel3 	GlobalFlags1.bD3
#define bHasKneadWidthMedPulse		GlobalFlags1.bD4
#define bDisplayKneadWidthMed		GlobalFlags1.bD5
#define bDisplayKneadTrackMed		GlobalFlags1.bD6
#define bLegPadLinkage 			GlobalFlags1.bD7

__no_init BITS GlobalFlags2 ;
#define bKneadWidthMinPulseLevel0 	GlobalFlags2.bD0
#define bKneadWidthMinPulseLevel1 	GlobalFlags2.bD1
#define bKneadWidthMinPulseLevel2 	GlobalFlags2.bD2
#define bKneadWidthMinPulseLevel3 	GlobalFlags2.bD3
#define bHasKneadWidthMinPulse	 	GlobalFlags2.bD4
#define bDisplayKneadWidthMin		GlobalFlags2.bD5
#define bDisplayKneadTrackMin		GlobalFlags2.bD6
#define bWaveMotorFail 			GlobalFlags2.bD7

__no_init BITS GlobalFlags3 ;
#define bodyDetectSuccess		        GlobalFlags3.bD0
#define bFail		                GlobalFlags3.bD1
#define bKeyPowerSwitch 		GlobalFlags3.bD2
#define bKeyWaistHeat 			GlobalFlags3.bD3
#define bSlowDisplayFlash		GlobalFlags3.bD4
#define bKeySeatVibrate 		GlobalFlags3.bD5
#define bKeySeatEnable 			GlobalFlags3.bD6
#define bMP3RunMode	 		GlobalFlags3.bD7

//位变量
__no_init BITS GlobalFlags4 ;
#define bFastDisplayFlash 		GlobalFlags4.bD0
#define bTimer10MS 			GlobalFlags4.bD1
#define bRunTimeChange 			GlobalFlags4.bD2
#define bWalkMotorPowerFlag 		GlobalFlags4.bD3
#define bKneadMotorPowerFlag 		GlobalFlags4.bD4
#define bKnockMotorPowerFlag 		GlobalFlags4.bD5
#define bBackLegPadSettle 		GlobalFlags4.bD6
#define bDisplayFlash 			GlobalFlags4.bD7

__no_init BITS GlobalFlags5 ;
#define bBackAutoModeInit 			GlobalFlags5.bD0
#define bBackManualModeInit 		        GlobalFlags5.bD1
#define bWalkMotorInProcess 		        GlobalFlags5.bD2 //行走电机程序执行标志
#define bKneadMotorInProcess 		        GlobalFlags5.bD3 //揉捏电机程序执行标志，例如顺时针揉捏3圈后停止
#define bKnockMotorInProcess 		        GlobalFlags5.bD4 //敲击电机程序执行标志
#define bGetNextActionStep 			GlobalFlags5.bD5
#define bKeyWalkUp 				GlobalFlags5.bD6
#define bKeyWalkDown 				GlobalFlags5.bD7

__no_init BITS GlobalFlags6 ;
#define bVibMotorEnable 			GlobalFlags6.bD0
#define bMassagePositionUpdate 			GlobalFlags6.bD1
#define bMarkSpace				GlobalFlags6.bD2
#define bSendBuzzerMode 			GlobalFlags6.bD3
#define bDisplayDetect  			GlobalFlags6.bD4//20181019
#define bMasterSendPacket 			GlobalFlags6.bD5
#define bReconfigFlag 				GlobalFlags6.bD6
#define bKneadWidthChange			GlobalFlags6.bD7


__no_init BITS GlobalFlags7 ;
#define bKeyBackPadUp 				GlobalFlags7.bD0
#define bKeyBackPadDown 			GlobalFlags7.bD1
//#define bPowerOffReach           	        GlobalFlags7.bD2
//#define bReachBackPadDownPosition 	        GlobalFlags7.bD3
//#define bBackPadMotorPowerFlag		        GlobalFlags7.bD4
#define bGetAirBagNextStep 			GlobalFlags7.bD5
#define bCurActionStepChange		        GlobalFlags7.bD6
#define bWalkLocateChange			GlobalFlags7.bD7

__no_init BITS GlobalFlags8 ;
#define bKeyLegPadUp 				GlobalFlags8.bD0
#define bKeyLegPadDown 				GlobalFlags8.bD1
//#define bKeyFlexOut 		                GlobalFlags8.bD2   //fww
//#define bKeyFlexIn 	                        GlobalFlags8.bD3   //fww
#define bKeyWaistHeatStore		        GlobalFlags8.bD4
//#define bWalkMotorLocateChange 		GlobalFlags8.bD5
//#define bReachWalkUpLimitFlag		        GlobalFlags8.bD6
//#define bReachWalkDownLimitFlag		        GlobalFlags8.bD7

__no_init BITS GlobalFlags9 ;
#define bProgramMemorySet			GlobalFlags9.bD0
#define bBodyDetectSuccess			GlobalFlags9.bD1
#define bKeyZeroUp			        GlobalFlags9.bD2
#define bGetArmAirBagNextStep 		        GlobalFlags9.bD3
#define bZeroTransition				GlobalFlags9.bD4
#define bZeroRestFlag				GlobalFlags9.bD5
#define bZeroRunFlag				GlobalFlags9.bD6
#define bGetBodyUpAirBagNextStep 	        GlobalFlags9.bD7

__no_init BITS GlobalFlags10 ;
#define bZeroRunUpFlag				GlobalFlags10.bD0
#define bZeroRunDownFlag			GlobalFlags10.bD1
#define bMP3_AD_Enable				GlobalFlags10.bD2
#define bKeyZeroDown    			GlobalFlags10.bD3
#define bBackMotorUpFlag			GlobalFlags10.bD4
#define bLegkMotorUpFlag			GlobalFlags10.bD5
#define bBlueToothMasterSendPacket		GlobalFlags10.bD6
#define bBlueToothSendBuzzerMode		GlobalFlags10.bD7

//MP3 变量

__no_init BITS GlobalFlags11 ;
#define KR_PROGRAM                              GlobalFlags11.bD0
#define bLegMotorReport				GlobalFlags11.bD1
#define bBackMotorReport			GlobalFlags11.bD2
#define bKneckCheckSwitchLast			GlobalFlags11.bD3
#define bErrorOverFlag				GlobalFlags11.bD4
#define bRunOverFlag				GlobalFlags11.bD5
#define bDemoRun                		GlobalFlags11.bD6
#define ZeroNewCount                		GlobalFlags11.bD7
//#define bDemoFlag		                GlobalFlags11.bD7

//工程模式存储数据定义
/********存储地址分配********************
0: 标识ID
1: 标识ID
2: 气囊强度
3: 拉退强度
4: 关机回位
*************************************/
#define MEMORY_DEFAULT_AIR     1 //0,1,2
#define MEMORY_DEFAULT_SETTLE  0 //0: 运行结束关机不复位,1:运行结束关机复位
#define SLIDE_DEFAULT_ENABLE   1 //拉退状态 5为行程开关
//按摩椅位置定义
__no_init static int stretchMode;
__no_init unsigned char nIndicateTimer;
//unsigned int nLegMotorRemoveTime,nBackMotorRemoveTime,nZeroMotorRemoveTime;
//当前按摩椅位置，目标按摩椅位置
__no_init unsigned char nTargetMassagePosition;
__no_init unsigned char nReworkShoulderPosition;
__no_init unsigned int password;
//摇摆相关变量
//__no_init bool bRockEnable; //fww
//__no_init unsigned char nRockModeEnterEnable;  //fww
//__no_init unsigned char WorkStep; //摇摆       //fww
//830气阀变量定义
//__no_init st_AirBag /*st_AirBagAuto,*/st_AirBagArmSholderBackWaist, st_AirBagModeLegFootSeat, st_AirBagLegFoot, st_AirBagArmSholder, st_AirBagBackWaist, st_AirBagSeat, st_AirBagArm;
__no_init st_AirBag st_AirBagAuto0,st_AirBagAuto1,st_AirBagAuto2,st_AirBagAuto3;//Fungares:1:ArmSholder,2:Seat,3:Foot.
//Walk Motor Variables
//unsigned char nCurWalkMotorState,nPrevWalkMotorState,nFinalWalkMotorState ;
unsigned char nCurWalkMotorStateCounter ;//State Counter
__no_init unsigned short nShoulderPosition, nShoulderPositionTop, nShoulderPositionBottom ; //Shoulder Position
//default value according to the value of DEFAULT_SHOULDER_POSITION,can be modified
//#ifdef SINGLE_HEADER
unsigned char WALK_MOTOR_ZONE[] = {28, 56, 84, 113, 142, 170} ;
//#else
//unsigned char WALK_MOTOR_ZONE[] = {14, 28, 42, 56, 71, 85} ;
//#endif
__no_init unsigned short nFinalWalkMotorLocate ;
//Knead Motor Variables
unsigned char nCurKneadMotorState, nPrevKneadMotorState, nFinalKneadMotorState ;
unsigned char nCurKneadMotorStateCounter ;//State Counter

unsigned char nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
//unsigned char nDisplayKneadWidth ;
__no_init unsigned char nCurKneadMotorCycles ;
__no_init unsigned char nLastKneadPosition ;

unsigned char nCurBackPadMotorState ;
unsigned char nPrevBackPadMotorState ;
unsigned char nCurLegPadMotorState ;
unsigned char nPrevLegPadMotorState ;
unsigned char nCurZeroPadMotorState ;
unsigned char nPrevZeroPadMotorState ;
unsigned char nCurZeroPadMotorStateCounter ;

__no_init unsigned char nRollerPWM, nRollerPWMStore ;
__no_init bool bRollerEnable, bRollerEnableStore ;

//__no_init unsigned char bMasterSendPacketWL;//140722//fww
//__no_init unsigned char bMasterSendPacketDecide;//140722  1:BlueTooth send ;0:Hand Controller
/*****************************************/
//unsigned char nZeroMotorState,nNewZeroMotorState;
//unsigned char nZero_MotorRunState,Zero_StartState,Zero_MotorRunState,Zero_MotorStopState;
//unsigned int nZeroTime = 0;
__no_init unsigned char nMaunalSubMode, nCurMaunalSubModeStore ;
//unsigned char nPartSubMode;
unsigned char nWalkMotorLocateState = 0;
__no_init unsigned char nDetectStep, nStretchStep;
/*****************************************/
__no_init unsigned char nKeyAirBagLocate ;//一个变量对应3个按键(开关/自动/定位)
__no_init unsigned char nKeyAirBagLocateStore ;

//与振动相关的变量
__no_init unsigned char nKeySeatVibrateStrength ;
//unsigned char nVibDescriptor ;
//unsigned char nVibTotalSteps,nCurVibStep ;//Auto数组单个元素内的步骤数
//unsigned char nVibAction ;
//unsigned char nVibTimeMode ;
//unsigned char nVibTime,nCurVibTime ;
//unsigned char nVibStrengthMode ;
//unsigned char nVibStrength ;
//unsigned char nCurAutoVibItem ;//Auto数组包含的元素

//Vibration PWM
//unsigned char nVibPWMCount ;
//unsigned char nVibPWMDutyCycle ;

//Motor control parameters
__no_init unsigned char nWalkMotorControlParam1;
__no_init unsigned short nWalkMotorControlParam2 ;
__no_init unsigned char nKneadMotorControlParam1, nKneadMotorControlParam2 ;
__no_init unsigned char nKnockMotorControlParam1, nKnockMotorControlParam2, nKnockMotorControlParam3 ;

//Variables relative to Auto&Manual&Standby mode
//Auto
__no_init unsigned char nBackMainRunMode, nBackSubRunMode, nCurBackMainRunModeStore, nCurBackSubRunModeStore ;
__no_init unsigned char nCurSubFunction ;//用于显示及判断Soft_Knock
__no_init unsigned char nCurKneadKnockSpeed ;//用于显示
unsigned int  nMusicKnockPWM ;//用于音乐互动
//unsigned int  nLastPWM1Value ;
//unsigned int  nLastPWM2Value ;

//Manual
__no_init unsigned char nKeyBackLocate, nKeyBackLocateStore ;
__no_init unsigned char nKeyKneadWidth ;
__no_init unsigned char nKeyKneadKnockSpeed ;
__no_init unsigned short nPartialTop, nPartialBottom ;
//Standby
__no_init unsigned char nBackSettleStep ;
__no_init unsigned char nBackSettleReason ;

//Auto&Manual&Standby
__no_init unsigned int nCurActionStep ;
__no_init unsigned int nMaxActionStep ;
__no_init unsigned char nStartActionStep ;

__no_init unsigned char nCurActionStepCounter ;//当前步骤时间计数器(适用于所有行走，揉捏，敲打电机)
__no_init unsigned char nCurShoulderAdjustCounter ;
__no_init unsigned char nCurKnockRunStopCounter ;
//与通信相关的变量
//两种类型的数据包，命令包和状态包，命令包需要应答，状态包无需应答
__no_init unsigned char nInChar ;
__no_init unsigned char OutBuffer[MAX_OUTBUFFER_COUNT] ;
__no_init unsigned char OutBufferBlueTooth[MAX_OUTBUFFER_COUNT] ;
__no_init unsigned char InBuffer[MAX_INBUFFER_COUNT] ;
__no_init unsigned char nInBufferCount ;
__no_init unsigned char nOutBufferCount ;
__no_init unsigned char nOutBufferBlueToothCount ;
__no_init unsigned char nSendCount ;
__no_init unsigned char nCommandID ;
__no_init unsigned char nSendPacketID ;

//141116
__no_init unsigned int nKneadBalanceCounter;
//
__no_init unsigned char nTimer10MS ;
//unsigned char nTimer50MS ;
__no_init unsigned char nTimer100MS ;
__no_init unsigned char nTimer500MS ;
//unsigned int  nTimer1Min ;
//unsigned char nRunTime;
__no_init unsigned char nBackLegPadFlashCount ;

__no_init unsigned char nChairStateCount ;//Wait_command->Idle
__no_init unsigned char nChairStateCount2 ;//复位二分钟未完成标志位
__no_init unsigned char nChairRunDemoCount ;
//蜂鸣器
__no_init unsigned char nBuzzerMode ;
__no_init unsigned int w_PresetTime;
__no_init unsigned char w_PresetTime_Min;                           //fww
//140530
__no_init unsigned int w_PresetTimeStore ;
//A/D转换
unsigned char nvcBluetoothDescoverable;
unsigned char nvcBluetoothPower = 1;
unsigned char nvcBluetoothPair;
unsigned char closeBluetooth;
//#define PAD_STOP;
//unsigned char padAction  //电动缸动作  0：stop 1：手动 2:自动位置  3：拉退程序控制
unsigned int nAvrADResult0 ;

unsigned int topPositionRefreshedFlag;
//unsigned int test_p1;unsigned int test_p2;unsigned int test_p3;unsigned int test_value;
//肩位检测用3个点
int shoulderPos[3];
unsigned int shoulderPositionScanStep;

/*****************************/
unsigned int nWidthOverTime;
unsigned int nPowerOverTime;
unsigned int nWalkOverTime;
unsigned int nBackOverTime;
unsigned int nLegOverTime;
unsigned int nZeroOverTime;
unsigned int nWaveOverTime;
unsigned int nFlexOverTime;
unsigned int nPowerMotorHighTime;
unsigned int nPowerMotorLowTime;
unsigned int nPowerValveHighTime;
unsigned int nPowerValveLowTime;
unsigned int nPowerVCCHighTime;
unsigned int nPowerVCCLowTime;

unsigned int w_KeyWalkHoldTimer;
//
//unsigned int nPowerOff_1st3s_Time;
//
//unsigned int FlexMotor_ResetTime;
//
//unsigned char bFlexMotorRestTimeStartFlag;  //1:start  0:stop and clear it as 0
//  bFlexMotor_OutFlag :1 make the flexmotor out;0 do NOT make the flexmotor out 
//__no_init unsigned char bFlexMotor_OutFlag;

//140527 
// 1:Press OK; 0:Press Cancle ; 2:no any key was pressed
//__no_init unsigned char bSelectKeyValue ;

//1:Enter Display Program; 0:Not Enter
//unsigned char bResetStateORchangeModeDis ;//fww

//1: reset the massage chair ; 0: Change the workmode(auto to auto;manual to auto)
//__no_init unsigned char bResetFlag ;//fww
//__no_init unsigned char nChairResetReason ; //1: Power Key was pressed; 0: time over //fww
//__no_init unsigned char nTimerOverResetAgainFlag ;//fww
//140530
__no_init unsigned char nSendBuzzerTimes ;
//__no_init unsigned char nBuzzer100msFlag ;
//
//__no_init unsigned char nWaitCustomActionTime ;//fww

/*****************************/
void refreshAutoDirector(void);
#ifdef SN_CHECK_EN

#define SN_DIS_INTERVAL 20



const unsigned char SN_CHECK_KEY_MASK[] =
{
    H10_KEY_CHAIR_AUTO_0,
    H10_KEY_CHAIR_AUTO_2,
    H10_KEY_CHAIR_AUTO_1,
    H10_KEY_CHAIR_AUTO_3,
    H10_KEY_CHAIR_AUTO_0
} ;
unsigned char SN_CHECK_KEY_INPUT[5] ;

unsigned char nSendSNFlag = FALSE ;
unsigned char nSendSNStep = 0 ;
unsigned char nSendSNStepCount = 0 ;
unsigned char SNCheckKeyCompare(void)
{
    unsigned char i ;
    for(i = 0; i < 5 ; i++)
    {
        if(SN_CHECK_KEY_INPUT[i] != SN_CHECK_KEY_MASK[i]) return FALSE ;
    }
    return TRUE ;
}
#endif


void Main_Shoulder_Detect(void);
void Main_Initial_Data(void);

void SNSend(unsigned char nSN)
{
    /*
      H10_RUNTIME_STRUCT RunTimeBuffer ;
    RunTimeBuffer.nRunTime[0] = LcdSeg[(nSN>>4) & 0x0f] ;
    RunTimeBuffer.nRunTime[1] = LcdSeg[nSN & 0x0f] ;
    VfdLedBuzzer.bSeg1A = RunTimeBuffer.bSeg1A ;
    VfdLedBuzzer.bSeg1B = RunTimeBuffer.bSeg1B ;
    VfdLedBuzzer.bSeg1C = RunTimeBuffer.bSeg1C ;
    VfdLedBuzzer.bSeg1D = RunTimeBuffer.bSeg1D ;
    VfdLedBuzzer.bSeg1E = RunTimeBuffer.bSeg1E ;
    VfdLedBuzzer.bSeg1F = RunTimeBuffer.bSeg1F ;
    VfdLedBuzzer.bSeg1G = RunTimeBuffer.bSeg1G ;
    VfdLedBuzzer.bSeg2A = RunTimeBuffer.bSeg2A ;
    VfdLedBuzzer.bSeg2B = RunTimeBuffer.bSeg2B ;
    VfdLedBuzzer.bSeg2C = RunTimeBuffer.bSeg2C ;
    VfdLedBuzzer.bSeg2D = RunTimeBuffer.bSeg2D ;
    VfdLedBuzzer.bSeg2E = RunTimeBuffer.bSeg2E ;
    VfdLedBuzzer.bSeg2F = RunTimeBuffer.bSeg2F ;
    VfdLedBuzzer.bSeg2G = RunTimeBuffer.bSeg2G ;
    */
}
//1 Instruction Cycle Macro
//Delay2T对应的汇编语句
//MOVLB    0x1  //选择BANK
//CLRF     0xd7,0x1 //清除
/*
void Main_Get_Massage_Position(void)
{
 unsigned int w_BackPadLocation,w_LegPadLocation,w_SildeLocation;
 w_BackPadLocation = BackMotor_Get_Location();
 w_LegPadLocation = LegMotor_Get_Location();
 w_SildeLocation = SildeMotor_Get_Location();
 if((w_BackPadLocation == BACK_MOTOR_AT_TOP) && (w_LegPadLocation == LEG_MOTOR_AT_BOTTOM) &&(w_SildeLocation == SLIDE_MOTOR_AT_BACKWARD))
 {
   nCurMassagePosition = MASSAGE_RESET_POSITION;
   return;
 }
 if((w_BackPadLocation == BACK_MOTOR_AT_TOP) && (w_LegPadLocation == LEG_MOTOR_AT_BOTTOM) &&(w_SildeLocation == SLIDE_MOTOR_AT_FORWARD))
 {
   nCurMassagePosition = MASSAGE_INIT_POSITION;
   return;
 }

 if((w_BackPadLocation == BACK_MOTOR_AT_BOTTOM) && (w_LegPadLocation == LEG_MOTOR_AT_TOP) &&(w_SildeLocation == SLIDE_MOTOR_AT_FORWARD))
 {
   nCurMassagePosition = MASSAGE_MAX_POSITION;
   return;
 }

  if((w_BackPadLocation == BACK_MOTOR_AT_MID) && (w_LegPadLocation == BACK_MOTOR_AT_MID) &&(w_SildeLocation == SLIDE_MOTOR_AT_FORWARD))
 {
   nCurMassagePosition = MASSAGE_MAX_POSITION;
   return;
 }


}
*/

void main_Problem(unsigned char ErrCode)
{
//  memset(VfdLedBuzzer.nVfdLedBuzzer,0,sizeof(VfdLedBuzzer.nVfdLedBuzzer));  //先将显示全部清0
// Display_ErrorCode(ErrCode);
    nBuzzerMode = BUZZER_MODE_FAST ;
    bSendBuzzerMode = TRUE ;
    //Close_Power();
    Valve_Test_Set_Data(0);
    Valve_LegFootAirPumpACPowerOff();
    nChairRunState = CHAIR_STATE_PROBLEM;
    nIndicateTimer = PROBLEM_INDICATE_TIME;
    while(1)
    {
        WDOG_Feed();
        if(Timer_Counter(C_TIMER_INDICATE + T_LOOP, nIndicateTimer))
        {
            IndicateLED_Toggle();
        }
        //CommProcess() ;
        Valve_Send_Data();
        LED_RGB_Proce(nChairRunState);
    }
}

void ExceptionHandles(void)
{
//  static unsigned int timeBack = 0;
//  SlideMotor_Proce();
    //if(bCancelErr) return;
    if(nWidthOverTime > 100)
    {
        main_Problem(0xE2);
    }
    if(nWalkOverTime > 100)
    {
        main_Problem(0xE3);
    }
    /*
    if(nBackOverTime > timeBack)
    {
      timeBack = nBackOverTime;
    }
    */
    if(nBackOverTime > 300)
    {
        main_Problem(0xE4);
    }

    if(nLegOverTime > 300)
    {
        main_Problem(0xE5);
    }


    if(nZeroOverTime > 300)
    {
        main_Problem(0xE6);
    }
    if(nPowerOverTime > 50)
    {
        main_Problem(0xE7);
    }
    if(nWaveOverTime > 100)  //摇摆马达回中位置检测处理
    {
        bWaveMotorFail = 1;
    }
    else
    {
        bWaveMotorFail = 0;
    }
}
////////
bool RockBackLegProcess(void)
{
  unsigned int /*w_BackPosition,*/w_LegPosition;
  bool /*bBackpositiondone,*/bLegpositiondone;
 // w_BackPosition = BackMotor_Get_Position();
  w_LegPosition = LegMotor_Get_Position();
  /*
  if(w_BackPosition <= MASSAGE_BACK_ROCK_POSITION)
  {
    bKeyBackPadUp = FALSE;
    bKeyBackPadDown = TRUE;
    bBackpositiondone = false;
  }
  else if(w_BackPosition >= (MASSAGE_BACK_ROCK_POSITION + MASSAGE_DIFFERENT_POSITION))
  {
    bKeyBackPadUp = TRUE;
    bKeyBackPadDown = FALSE;
    bBackpositiondone = false;
  }
  else 
  {
    bKeyBackPadUp = FALSE;
    bKeyBackPadDown = FALSE;
    bBackpositiondone = true;
  }
  */
  if(w_LegPosition <= MASSAGE_LEG_ROCK_POSITION)
  {
    bLegPadLinkage = FALSE;
    bKeyLegPadUp = TRUE;
    bKeyLegPadDown = FALSE;
    bLegpositiondone = false;
  }
  else if(w_LegPosition >= MASSAGE_LEG_ROCK_POSITION + MASSAGE_DIFFERENT_POSITION)
  {
    bLegPadLinkage = FALSE;
    bKeyLegPadUp = FALSE;
    bKeyLegPadDown = TRUE;
    bLegpositiondone = false;
  }
  else
  {
    bKeyLegPadUp = FALSE;
    bKeyLegPadDown = FALSE;
    bLegpositiondone = true;
  }
  //if((bLegpositiondone == true)&&(bBackpositiondone == true)) return true;  //到达了预定的位置
  if(bLegpositiondone == true) return true;  //到达了预定的位置
  else return false;  //未到达预定的位置
}
//
/*******************fww***********************
void RockFunctionEnable(bool Enable)
{
  bKeyBackPadUp = FALSE;
  bKeyBackPadDown = FALSE;
  bKeyLegPadUp = FALSE;
  bKeyLegPadDown = FALSE;
  switch(Enable)
  {
  case RockDisable:
    bRockEnable = false;
    break;
  case RockEnable:
    setBackPadRockingEnable(Enable);
    nRockModeEnterEnable = EnterRock;
    bRockEnable = true;
    WorkStep = 0;//一旦刚进入该模式就开始尝试下躺
    break;
  default :
    bRockEnable = false;
    break;
  }
}
***********************************************/
//
/************************fww********************************
void RockProcess(void)
{
#ifdef ROCK_TEST
  
  //没有在摇摆模式时就直接退出
  if(nRockModeEnterEnable == ExitRock) return;
  
  unsigned int CurrentBackMotorPosition;//,w_BackPosition;
  bool bBackupdone,bBackdowndone;
  if(bRockEnable)
  {
    switch(WorkStep)
    {
    case StartRock:
      //SlideMotorControl(STATE_RUN_SLIDE_FORWARD);
      //w_BackPosition = BackMotor_Get_Position();
      //if((Input_GetSlideForwardSwitch() == REACH_SLIDE_LIMIT)&&(bLegdowndone == true)&&(bBackdowndone == true))
      //if(Input_GetSlideForwardSwitch() == REACH_SLIDE_LIMIT)
      {
        //131224
        bKeyBackPadUp = FALSE;
        bKeyBackPadDown = TRUE;
        //bKeyLegPadUp = TRUE;
        //bKeyLegPadDown = FALSE;
        
        //if(RockBackLegProcess() == true)
        if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT)
        //if(Input_GetSlideForwardSwitch() == REACH_SLIDE_LIMIT)
        {
          //BackMotor_Control(STATE_RUN_BACK_DOWN);
          //if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT)
          {
            WorkStep++;
          }
        }
      }
      break;
    case LieDown:
      CurrentBackMotorPosition = Input_GetBackPosition();
      if(CurrentBackMotorPosition < MASSAGE_BACK_DOWN_ROCK_POSITION)
        //if(Input_GetSlideForwardSwitch() != REACH_SLIDE_LIMIT)
      {
        //SlideMotorControl(STATE_RUN_SLIDE_FORWARD);
        //BackMotor_Control(STATE_RUN_BACK_DOWN);
        bKeyBackPadUp = FALSE;
        bKeyBackPadDown = TRUE;
        bBackdowndone = false;
      }
      else
      {
        //SlideMotorControl(STATE_SLIDE_IDLE);
        //BackMotor_Control(STATE_BACK_IDLE);
        bKeyBackPadUp = FALSE;
        bKeyBackPadDown = FALSE;
        bBackdowndone = true;
      }
      if(//(RockBackLegProcess() == true)//&&(bBackdowndone == true))
      {
        WorkStep++;
      }
      break;
    case LieUP:
      CurrentBackMotorPosition = Input_GetBackPosition();
      if(CurrentBackMotorPosition > MASSAGE_BACK_UP_ROCK_POSITION)
        //if(Input_GetSlideBackwardSwitch() != REACH_SLIDE_LIMIT)
      {
        //SlideMotorControl(STATE_RUN_SLIDE_BACKWARD);
        //BackMotor_Control(STATE_RUN_BACK_UP);
        bKeyBackPadUp = TRUE;
        bKeyBackPadDown = FALSE;
        bBackupdone = false;
      }
      else
      {
        //SlideMotorControl(STATE_SLIDE_IDLE);
        //BackMotor_Control(STATE_BACK_IDLE);
        bKeyBackPadUp = FALSE;
        bKeyBackPadDown = FALSE;
        bBackupdone = true;
      }
      if(//(RockBackLegProcess() == true)//&&(bBackupdone == true))
      {
        WorkStep = LieDown;
      }
      break;
    default :
      break;
    }
  }
  else
  {
    BackMotor_Control(STATE_BACK_IDLE);
    //nRockModeEnterEnable = ExitRock;
  }
  
#else
  //轻松模式下
  /////////////////////////////////////////////////
  if(nBackMainRunMode == BACK_MAIN_MODE_AUTO &&
     (nBackSubRunMode == BACK_SUB_MODE_AUTO_2))
  {
    unsigned int CurTime = Data_Get_TimeSecond();
    if((CurTime == 13 * 60) ||
       (CurTime == 4 * 60))
    {
      RockFunctionEnable(true);
    }
    if((CurTime == 16 * 60) ||
       (CurTime == 7 * 60))
    {
      RockFunctionEnable(false);
      //回到类似第一零重力状态
      nTargetMassagePosition = MASSAGE_OPTIMAL_POSITION;
      bMassagePositionUpdate = TRUE;    
    }
  }
  //////////////////////////////////////////
  //没有在摇摆模式时就直接退出
  if(nRockModeEnterEnable == ExitRock) return;
  
  unsigned int CurrentBackMotorPosition;//,w_BackPosition;
  bool bBackupdone,bBackdowndone;
  if(bRockEnable)
  {
    switch(WorkStep)
    {
    case StartRock:     
        bKeyBackPadUp = FALSE;
        bKeyBackPadDown = TRUE;
        RockBackLegProcess();
        if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT)
        {
          WorkStep++;
        }
      break;
    case LieDown:
      CurrentBackMotorPosition = Input_GetBackPosition();
      if(CurrentBackMotorPosition < MASSAGE_BACK_DOWN_ROCK_POSITION)
      {
        bKeyBackPadUp = FALSE;
        bKeyBackPadDown = TRUE;
        bBackdowndone = false;
      }
      else
      {
        bKeyBackPadUp = FALSE;
        bKeyBackPadDown = FALSE;
        bBackdowndone = true;
      }
      if((RockBackLegProcess() == true)&&(bBackdowndone == true))
      {
        WorkStep++;
      }
      break;
    case LieUP:
      CurrentBackMotorPosition = Input_GetBackPosition();
      if(CurrentBackMotorPosition > MASSAGE_BACK_UP_ROCK_POSITION)
      {
        bKeyBackPadUp = TRUE;
        bKeyBackPadDown = FALSE;
        bBackupdone = false;
      }
      else
      {
        bKeyBackPadUp = FALSE;
        bKeyBackPadDown = FALSE;
        bBackupdone = true;
      }
      if((RockBackLegProcess() == true)&&(bBackupdone == true))
      {
        WorkStep = LieDown;
      }
      break;
    default :
      break;
    }
  }
  else
  {
    BackMotor_Control(STATE_BACK_IDLE);
    nRockModeEnterEnable = ExitRock;
  }
#endif
}
**********************************************/
/************************************************
*@brief :adjust the tapmotor stop at the 
*needed position
*@input :no
*@return :no
**************************fww**********************/
/***********************fww*************************
void TapMotor_Position_Proce(void)
{
  static unsigned char AdjustStep = 0 ;
  //AdjustStep = 0;
  if(bKnockMotorInProcess == FALSE)
  {
    
    //141116
    if(Input_GetVout() == 0)
    {
      KnockMotor_Set_Pwm_Data(KNOCK_SPEED0_PWM);
      return;
    }
    
    //141116
    if(nKneadBalanceCounter >= MAX_KNEADBALANCE_TIME_1MIN) return;
    
    switch(AdjustStep)
    {
     case 0:
      //if(TapMotorPosition() == FALSE) //Not the needed position
      if(Input_CurTapMotorPos() == FALSE)
      {
        KnockMotor_ClockRun();
        KnockMotor_Set_Pwm_Data(KNOCK_SPEED3_PWM);
      }
      else
      {
        nKneadBalanceCounter = 0;
        AdjustStep++ ;
        KnockMotor_Set_Pwm_Data(KNOCK_SPEED0_PWM);
      }
      break;
     case 1:
      //KNOCK_ADJ_POS0_PWM
      //if(TapMotorPosition() == FALSE) //Not the needed position
      if(Input_CurTapMotorPos() == FALSE)
      {
        KnockMotor_UnClockRun();
        KnockMotor_Set_Pwm_Data(KNOCK_SPEED3_PWM);
      }
      else
      {
        nKneadBalanceCounter = 0;
        AdjustStep++ ;
        KnockMotor_Set_Pwm_Data(KNOCK_SPEED0_PWM);
      }
      break;
     case 2:
      //if(TapMotorPosition() == FALSE)
      if(Input_CurTapMotorPos() == FALSE)
      {
        KnockMotor_ClockRun();
        KnockMotor_Set_Pwm_Data(KNOCK_SPEED3_PWM);
      }
      else
      {
        nKneadBalanceCounter = 0;
        AdjustStep = 0;
        KnockMotor_Set_Pwm_Data(KNOCK_SPEED0_PWM);
      }
      break;
     default :
      break;
    }
  }
  else
  {
    //141116
    nKneadBalanceCounter = 0;
  }
  
}
************************fww***********************/
//
void Main_Massage_Position_Proce(void)
{
  bool bBackPadFinish, bLegPadFinish/*, bFlexPadFinish*/;
  bool bAdjFlex = 0;
  unsigned int w_BackPosition, w_LegPosition;
  
  if((bKeyBackPadUp == TRUE) ||
     (bKeyBackPadDown == TRUE) ||
       (bKeyLegPadUp == TRUE) ||
         (bKeyLegPadDown == TRUE) ||
           /*(bKeyFlexOut == TRUE) ||
             (bKeyFlexIn == TRUE) ||*/
               (st_Stretch.active))
  {
    bMassagePositionUpdate = 0;    //手动优先
    return;
  }
 
  w_BackPosition = BackMotor_Get_Position();
  w_LegPosition = LegMotor_Get_Position();
  //printf("key=%d\n",w_BackPosition);
  
  
  if(KR_PROGRAM)
  {
    if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2))
    {
      if(Data_Get_TimeSecond() == 25 * 60)
      {
        nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
        bMassagePositionUpdate = TRUE;
      }
      if(Data_Get_TimeSecond() == 15 * 60)
      {
        nTargetMassagePosition = MASSAGE_MAX_POSITION;
        bMassagePositionUpdate = TRUE;
      }
    }
  }
  
  if(!bMassagePositionUpdate)
  {
    return;
  }
  //140529
  //if(bResetStateORchangeModeDis == EnterDisReset) return ;
  
  switch(nTargetMassagePosition)
  {
  case MASSAGE_RESET_POSITION:
    if(BackMotor_Control(STATE_RUN_BACK_UP) == TRUE)
    {
      bBackPadFinish = TRUE;
      /*if(SlideMotorControl(STATE_RUN_SLIDE_BACKWARD) == TRUE)
      {
        bSliderFinish = TRUE;
      }
      else
      {
        bSliderFinish = FALSE;
      }*/
    }
    else
    {
      bBackPadFinish = FALSE;
      //bSliderFinish = FALSE;
    }
    if(LegMotor_Control(STATE_RUN_LEG_DOWN) == LEG_STOP_AT_DOWN)
    {
      bLegPadFinish = TRUE;
    }
    else
    {
      bLegPadFinish = FALSE;
    }
    /*************************fww***********************************
    if(FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_RESET) == FLEX_STOP_AT_IN)
    {
      bFlexPadFinish = TRUE;
      //FlexMotor_ResetTime = 0;
    }
    else
    {
      bFlexPadFinish = FALSE;
    }
    ***************************fww*********************************/
    //FLEX_MOTOR_RESET_PROCESS();
    break;
  case MASSAGE_INIT_POSITION:
    //140704
    //Clear_Accident_flag() ;
    //if(SlideMotorControl(STATE_RUN_SLIDE_FORWARD) == TRUE)
    {
      //bSliderFinish = TRUE;
      
      if(BackMotor_Control(STATE_RUN_BACK_UP) == TRUE)
      {
        bBackPadFinish = TRUE;
      }
      else
      {
        bBackPadFinish = FALSE;
      }
      if(LegMotor_Control(STATE_RUN_LEG_DOWN) == TRUE)
      {
        bLegPadFinish = TRUE;
      }
      else
      {
        bLegPadFinish = FALSE;
      }
      /*******************************fww***************************************
      if(FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A) == FLEX_STOP_AT_IN)
      {
        bFlexPadFinish = TRUE;
      }
      else
      {
        bFlexPadFinish = FALSE;
      }
      *********************************fww****************************************/
    }
    /*
    else
    {
      bSliderFinish = FALSE;
      bBackPadFinish = FALSE;
      bLegPadFinish = FALSE;
      bFlexPadFinish = FALSE;
    }
    */
    break;
  case MASSAGE_OPTIMAL_POSITION:
    //140704
    //Clear_Accident_flag() ;
    bAdjFlex = true;
    //if(SlideMotorControl(STATE_RUN_SLIDE_FORWARD) == TRUE)
    {
      //bSliderFinish = TRUE;
      
      if(w_BackPosition > (MASSAGE_BACK_OPTIMAL_POSITION + 50))
      {
        BackMotor_Control(STATE_RUN_BACK_UP) ;
        bBackPadFinish = FALSE;
      }
      else if(w_BackPosition < (MASSAGE_BACK_OPTIMAL_POSITION - 50))
      {
        BackMotor_Control(STATE_RUN_BACK_DOWN) ;
        bBackPadFinish = FALSE;
      }
      else
      {
        BackMotor_Control(STATE_BACK_IDLE) ;
        bBackPadFinish = TRUE;
      }
      w_LegPosition = LegMotor_Get_Position();
      if(w_LegPosition > (MASSAGE_LEG_OPTIMAL_POSITION + 50))
      {
        LegMotor_Control(STATE_RUN_LEG_DOWN) ;
        bLegPadFinish = FALSE;
      }
      else if(w_LegPosition < (MASSAGE_LEG_OPTIMAL_POSITION - 50))
      {
        LegMotor_Control(STATE_RUN_BACK_UP) ;
        bLegPadFinish = FALSE;
      }
      else
      {
        LegMotor_Control(STATE_BACK_IDLE) ;
        bLegPadFinish = TRUE;
      }
      /**************************************************************************
      if(FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A) == FLEX_STOP_AT_IN)
      {
        bFlexPadFinish = TRUE;
      }
      else
      {
        bFlexPadFinish = FALSE;
      }
      ****************************************************************************/
    }
    /*
    else
    {
      bFlexPadFinish = FALSE;
      bSliderFinish = FALSE;
      bBackPadFinish = FALSE;
      bLegPadFinish = FALSE;
    }
    */
    //FlexMotorSetEnable();//fww
    break;
    
  case MASSAGE_OPTIMAL2_POSITION:
    //140704
    //Clear_Accident_flag() ;
    bAdjFlex = true;
    //if(SlideMotorControl(STATE_RUN_SLIDE_FORWARD) == TRUE)
    {
      //bSliderFinish = TRUE;
      w_BackPosition = BackMotor_Get_Position();
      if(w_BackPosition > (MASSAGE_BACK_OPTIMAL1_POSITION + POSITION_CTRL_OFFSET))
      {
        BackMotor_Control(STATE_RUN_BACK_UP) ;
        bBackPadFinish = FALSE;
      }
      else if(w_BackPosition < (MASSAGE_BACK_OPTIMAL1_POSITION - POSITION_CTRL_OFFSET))
      {
        BackMotor_Control(STATE_RUN_BACK_DOWN) ;
        bBackPadFinish = FALSE;
      }
      else
      {
        BackMotor_Control(STATE_BACK_IDLE) ;
        bBackPadFinish = TRUE;
      }
      w_LegPosition = LegMotor_Get_Position();
      if(w_LegPosition > (MASSAGE_LEG_OPTIMAL1_POSITION + POSITION_CTRL_OFFSET))
      {
        LegMotor_Control(STATE_RUN_LEG_DOWN) ;
        bLegPadFinish = FALSE;
      }
      else if(w_LegPosition < (MASSAGE_LEG_OPTIMAL1_POSITION - POSITION_CTRL_OFFSET))
      {
        LegMotor_Control(STATE_RUN_BACK_UP) ;
        bLegPadFinish = FALSE;
      }
      else
      {
        LegMotor_Control(STATE_BACK_IDLE) ;
        bLegPadFinish = TRUE;
      }
      /**************************************************************
      if(FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A) == FLEX_STOP_AT_IN)
      {
        bFlexPadFinish = TRUE;
      }
      else
      {
        bFlexPadFinish = FALSE;
      }
      ******************************************************************/
    }
    /*
    else
    {
      bFlexPadFinish = FALSE;
      bSliderFinish = FALSE;
      bBackPadFinish = FALSE;
      bLegPadFinish = FALSE;
    }
    */
    //FlexMotorSetEnable();//fww
    break;
  case MASSAGE_MAX_POSITION:
    //140704
    //Clear_Accident_flag() ;
    bAdjFlex = true;
    //if(SlideMotorControl(STATE_RUN_SLIDE_FORWARD) == TRUE)
    {
      //bSliderFinish = TRUE;
      
      if(BackMotor_Control(STATE_RUN_BACK_DOWN) == TRUE)
      {
        bBackPadFinish = TRUE;
      }
      else
      {
        bBackPadFinish = FALSE;
      }
      if(LegMotor_Control(STATE_RUN_LEG_UP) == LEG_STOP_AT_UP)
      {
        bLegPadFinish = TRUE;
      }
      else
      {
        bLegPadFinish = FALSE;
      }
      /*************************************************************
      if(FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A) == FLEX_STOP_AT_IN)
      {
        bFlexPadFinish = TRUE;
      }
      else
      {
        bFlexPadFinish = FALSE;
      }
      ***************************************************************/
    }
    /*
    else
    {
      bSliderFinish = FALSE;
      bBackPadFinish = FALSE;
      bLegPadFinish = FALSE;
      bFlexPadFinish = FALSE;
    }
    */
    //FlexMotorSetEnable();//fww
    break;
  case MASSAGE_ANY_POSITION:
  default:
    bMassagePositionUpdate = 0;
    break;
  }
  if(/*(bSliderFinish == TRUE) && */(bBackPadFinish == TRUE) && (bLegPadFinish == TRUE) /*&& (bFlexPadFinish == TRUE)*/)
  {
    bMassagePositionUpdate = 0;
    
    
    
    
    if(bAdjFlex)
    {
      //clear accident flag 140707
      //Clear_Accident_flag();
      //FlexMotorSetEnable();//fww
    }
    /*
    if(nTargetMassagePosition == MASSAGE_INIT_POSITION ||
    nTargetMassagePosition == MASSAGE_OPTIMAL_POSITION ||
    nTargetMassagePosition == MASSAGE_OPTIMAL1_POSITION ||
    nTargetMassagePosition == MASSAGE_OPTIMAL1_POSITION ||
    
    */
  }
}
//靠背电动缸控制程序
void Main_BackPad_Proce(void)
{
    BackMotor_Proce();
    if(st_Stretch.active) return;
    if(bMassagePositionUpdate) return;
    //140529
    //if(bResetStateORchangeModeDis == EnterDisReset) return ;
    //if(bPowerOffReach == TRUE) return;
    if(bKeyBackPadUp == TRUE)
    {
      if(BackMotor_Control(STATE_RUN_BACK_UP) == TRUE)
      {
        /*
        if(SlideMotorControl(STATE_RUN_SLIDE_BACKWARD) == TRUE)
        {
        nBuzzerMode = BUZZER_MODE_FAST ;
        bSendBuzzerMode = TRUE ;
        bBlueToothSendBuzzerMode = TRUE;
      }
            else
        {
        nBuzzerMode = BUZZER_MODE_SLOW ;
        bSendBuzzerMode = TRUE ;
        bBlueToothSendBuzzerMode = TRUE;
      }
        */
        nBuzzerMode = BUZZER_MODE_FAST ;
        bSendBuzzerMode = TRUE ;
        bBlueToothSendBuzzerMode = TRUE;
      }
      else
      {
        if(bBackLegPadSettle == FALSE)
        {
          //140906
          //if(bRockEnable == false)//fww
          {
            nBuzzerMode = BUZZER_MODE_SLOW ;
            bSendBuzzerMode = TRUE ;
            bBlueToothSendBuzzerMode = TRUE;
          }
        }
      }
    }
    if(bKeyBackPadDown == TRUE)
    {
      if(BackMotor_Control(STATE_RUN_BACK_DOWN) == TRUE)
      {
        {
          nBuzzerMode = BUZZER_MODE_FAST ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
        }
      }
      else
      {
        if(bBackLegPadSettle == FALSE)
        {
          //140906
          //if(bRockEnable == false)//fww
          {
            nBuzzerMode = BUZZER_MODE_SLOW ;
            bSendBuzzerMode = TRUE ;
            bBlueToothSendBuzzerMode = TRUE;
          }
        }
      }
      /*
      if(SlideMotor_Get_Location() == SLIDE_MOTOR_AT_FORWARD)
      {
        SlideMotorControl(STATE_LEG_IDLE);
        if(BackMotor_Control(STATE_RUN_BACK_DOWN) == TRUE)
        {
          {
            nBuzzerMode = BUZZER_MODE_FAST ;
            bSendBuzzerMode = TRUE ;
            bBlueToothSendBuzzerMode = TRUE;
          }
        }
        else
        {
          if(bBackLegPadSettle == FALSE)//if(bBackLegPadSettle == FALSE// && st_Stretch.bBackLegFlag == FALSE//)
          {
            nBuzzerMode = BUZZER_MODE_SLOW ;
            bSendBuzzerMode = TRUE ;
            bBlueToothSendBuzzerMode = TRUE;
          }
        }
      }
      else
      {
        SlideMotorControl(STATE_RUN_SLIDE_FORWARD);
        {
          LegMotor_Control(STATE_LEG_IDLE);
          BackMotor_Control(STATE_BACK_IDLE);
          nBuzzerMode = BUZZER_MODE_SLOW ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
        }
      }
      */
    }
    if((bKeyBackPadUp == FALSE) && (bKeyBackPadDown == FALSE))
    {
        BackMotor_Control(STATE_BACK_IDLE) ;
        //SlideMotorControl(STATE_SLIDE_IDLE);
    }
}
//小腿电动缸控制程序
void Main_LegPad_Proce(void)
{
  LegMotor_Proce();
  if(st_Stretch.active) return;
  if(bMassagePositionUpdate) return;
  //140529
  //if(bResetStateORchangeModeDis == EnterDisReset) return ;
  
  if(bLegPadLinkage == FALSE) //小腿单独动，此时不考虑前滑电动缸的位置
  {
    if(bKeyLegPadUp == TRUE)
    {
      //clear accident flag 140707
      //Clear_Accident_flag();
      //FlexMotorSetEnable();//fww
      if(LegMotor_Control(STATE_RUN_LEG_UP) != LEG_RUN)
      {
        nBuzzerMode = BUZZER_MODE_FAST ;
        bSendBuzzerMode = TRUE ;
        bBlueToothSendBuzzerMode = TRUE;
      }
      else
      {
        //140906
        //if(bRockEnable == false)//fww
        {
          nBuzzerMode = BUZZER_MODE_SLOW ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
        }
      }
    }
    
    if(bKeyLegPadDown == TRUE)
    {
      
      if(LegMotor_Control(STATE_RUN_LEG_DOWN) != LEG_RUN)
      {
        nBuzzerMode = BUZZER_MODE_FAST ;
        bSendBuzzerMode = TRUE ;
        bBlueToothSendBuzzerMode = TRUE;
      }
      else
      {
        
          nBuzzerMode = BUZZER_MODE_SLOW ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
        
      }
    }
    
  }
  else  //靠背和小腿联动，前滑电动缸必须在最前位置
  {
    //if(SlideMotor_Get_Location() == SLIDE_MOTOR_AT_FORWARD)
    {
      if(bKeyLegPadUp == TRUE)
      {
        LegMotor_Control(STATE_RUN_LEG_UP);
        //clear accident flag 140707
        //Clear_Accident_flag();
        //FlexMotorSetEnable();
        
      }
      if(bKeyLegPadDown == TRUE)
      {
        LegMotor_Control(STATE_RUN_LEG_DOWN);
        // if(Input_GetFlexInSwitch() != REACH_FLEX_LIMIT)
        // {
        //clear accident flag 140707
        //Clear_Accident_flag();
        //FlexMotorSetEnable();
        // }
        //else
        //{
        //FlexMotorSetDisable();
        //}
      }
    }
    //else
    {
      if(Input_GetLegDownSwitch() == REACH_LEG_LIMIT)
      {
        //LegMotor_Control(STATE_LEG_IDLE);//fww
      }
    }
  }
  
  if((bKeyLegPadUp == FALSE) && (bKeyLegPadDown == FALSE))
  {
    LegMotor_Control(STATE_LEG_IDLE) ;
  }
}



void BackManualModeNoAction(void)
{
    nBackMainRunMode = BACK_MAIN_MODE_MANUAL ;
    nBackSubRunMode = BACK_SUB_MODE_NO_ACTION ;
    //nKeyBackLocate = LOCATE_NONE ;
    nKeyKneadKnockSpeed = SPEED_0 ;
    nCurKneadKnockSpeed = SPEED_0 ;
    ManualDirector[0].nSubFunction = BACK_SUB_MODE_NO_ACTION ;
    //ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_PARK ;
    //ManualDirector[0].nWalkMotorLocateParam = 0 ;
    ManualDirector[0].nKneadMotorState = KNEAD_STOP ;//KNEAD_STOP_AT_MAX ;
    ManualDirector[0].nKneadMotorCycles = 0 ;
    ManualDirector[0].nKnockMotorState = KNOCK_STOP ;
    ManualDirector[0].nKnockMotorRunTime = 0 ;
    ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
    nMaxActionStep = 1 ;
    nStartActionStep = 0 ;
    bBackManualModeInit = TRUE ;
}

void BodyDataRefresh(void)
{
    unsigned char nZoneStep, nZoneStepRemain ;

    unsigned short by_TopPosition = TOP_POSITION;
    /*
    if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) &&( nBackSubRunMode == BACK_SUB_MODE_AUTO_2))
    {
      by_TopPosition = TOP_POSITION_AUTO2;
    }
    */
    if(nShoulderPosition >= (by_TopPosition - MAX_SHOULDER_ADJUST_DIFF))
    {
        nShoulderPositionTop = by_TopPosition ;
        nShoulderPositionBottom = nShoulderPosition - MAX_SHOULDER_ADJUST_DIFF ;
    }
    else if(nShoulderPosition < MAX_SHOULDER_ADJUST_DIFF)
    {
        nShoulderPositionTop = nShoulderPosition + MAX_SHOULDER_ADJUST_DIFF ;
        nShoulderPositionBottom = 0 ;
    }
    else
    {
        nShoulderPositionTop = nShoulderPosition + MAX_SHOULDER_ADJUST_DIFF ;
        nShoulderPositionBottom = nShoulderPosition - MAX_SHOULDER_ADJUST_DIFF ;
    }
    nZoneStep = nShoulderPosition / 6 ;
    nZoneStepRemain = nShoulderPosition % 6 ;
    WALK_MOTOR_ZONE[0] = nZoneStep + ((nZoneStepRemain > 0) ? 1 : 0) ;
    if(nZoneStepRemain > 0) nZoneStepRemain-- ;
    WALK_MOTOR_ZONE[1] = WALK_MOTOR_ZONE[0] + nZoneStep + ((nZoneStepRemain > 0) ? 1 : 0) ;
    if(nZoneStepRemain > 0) nZoneStepRemain-- ;
    WALK_MOTOR_ZONE[2] = WALK_MOTOR_ZONE[1] + nZoneStep + ((nZoneStepRemain > 0) ? 1 : 0) ;
    if(nZoneStepRemain > 0) nZoneStepRemain-- ;
    WALK_MOTOR_ZONE[3] = WALK_MOTOR_ZONE[2] + nZoneStep + ((nZoneStepRemain > 0) ? 1 : 0) ;
    if(nZoneStepRemain > 0) nZoneStepRemain-- ;
    WALK_MOTOR_ZONE[4] = WALK_MOTOR_ZONE[3] + nZoneStep + ((nZoneStepRemain > 0) ? 1 : 0) ;
    if(nZoneStepRemain > 0) nZoneStepRemain-- ;
    WALK_MOTOR_ZONE[5] = WALK_MOTOR_ZONE[4] + nZoneStep + ((nZoneStepRemain > 0) ? 1 : 0) ;
}

bool isZeroPosition(void)
{
	unsigned int w_BackPosition = BackMotor_Get_Position();
	unsigned int w_LegPosition = LegMotor_Get_Position();
	bool result;
	result = (w_BackPosition < (MASSAGE_BACK_OPTIMAL1_POSITION + POSITION_DISPLAY_OFFSET));
	if(result) result = (w_BackPosition > (MASSAGE_BACK_OPTIMAL1_POSITION - POSITION_DISPLAY_OFFSET));
	if(result) result = (w_LegPosition > (MASSAGE_LEG_OPTIMAL1_POSITION - POSITION_DISPLAY_OFFSET));
	if(result) result = (w_LegPosition < (MASSAGE_LEG_OPTIMAL1_POSITION + POSITION_DISPLAY_OFFSET));
	return(result);
}

bool isZeroPositionNew(void)
{
	if(isZeroPosition())
		return(1);
	else
	{
		if((nBackSubRunMode == BACK_SUB_MODE_AUTO_1)&&(st_Stretch.active == TRUE))
			return(0);
		//if(nTargetMassagePosition == MASSAGE_OPTIMAL2_POSITION)
			//return(ZeroNewCount);
	}
	return(0);
}

unsigned char  ValToAscii(unsigned char nVal)
{
    if(nVal <= 9)
        return(nVal + 0x30);
    else
        return(nVal + 0x37); /*UpperCase*/
}
void Main_Close_Power(void)
{
  bool bEngineeringSettle = ReadEEByte(USER_DATA_BASE + SETTLE_ADDRESS);
  bKeyPowerSwitch = FALSE ;
  nKeyBackLocate = LOCATE_NONE;
  //VfdLedBuzzer.bREST = bDisplayFlash ;
  //电动缸回位
  //#ifdef POWER_SETTLE_FUNCTION
  
  if(bEngineeringSettle)
  {
    if(KR_PROGRAM)
    {
      if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2))
      {
        __no_operation();
      }
      else
      {
        bBackLegPadSettle = TRUE ;
        nTargetMassagePosition = MASSAGE_RESET_POSITION;
        bMassagePositionUpdate = TRUE;
      }
    }
    else
    {
      bBackLegPadSettle = TRUE ;
      nTargetMassagePosition = MASSAGE_RESET_POSITION;
      bMassagePositionUpdate = TRUE;
    }
  }
  //#endif
  //气囊按摩停止
  nKeyAirBagLocate = AIRBAG_LOCATE_NONE ;
  //加热功能停止
  //振动功能停止
  bKeySeatVibrate = FALSE ;
  //定时变量复位
  //nRunTime = 0 ;
  Data_Set_Start(0, 0);
  bKeyWaistHeat = FALSE;
  //140703
  //Clear_Accident_flag();
  bRollerEnable = FALSE;
  nRollerPWM = 0;
  Valve_SetRollerPWM(nRollerPWM);
  bRunTimeChange = TRUE ;
}

void RunOverStop(void)
{
  bool bEngSettle = (bool)ReadEEByte(USER_DATA_BASE + SETTLE_ADDRESS);
  bKeyLegPadUp = FALSE ;
  bKeyLegPadDown = FALSE ;
  bKeyBackPadUp = FALSE ;
  bKeyBackPadDown = FALSE ;
  bLegPadLinkage = FALSE ;
  nChairRunState = CHAIR_STATE_SETTLE ;
  nChairStateCount2 = 0;
  if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
  {
    nBackSettleReason = PARK_AUTO_OVER ;
  }
  else
  {
    nBackSettleReason = PARK_MANUAL_OVER ;
  }
  nBackMainRunMode = BACK_MAIN_MODE_SETTLE ;
  nBackSettleStep = 0 ;
  //140528
  if(bEngSettle == true)
  {
    //bResetFlag = RESET_CHAIR ;//fww
    //140530
    if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2))
    {
      __no_operation();
    }
    else
    {
      bBackLegPadSettle = TRUE ;
      nTargetMassagePosition = MASSAGE_RESET_POSITION;
      bMassagePositionUpdate = TRUE;
    }
  }
  /**********************fww**************************
  else
  {
    bResetFlag = NORESET_CHAIR ;
  }
  ************************fww*************************/
  //140528
  //nTimerOverResetAgainFlag = FIRST_TIME_ENTER ;//fww
  //nChairResetReason = TIMEOVER ;//fww
  //140529
  //bSelectKeyValue = KEY_NOT_CONFIRMORCANCLE ;//fww
  //140530
  //bResetStateORchangeModeDis = EnterDisReset ;
  nCurBackMainRunModeStore = nBackMainRunMode ;
  nCurBackSubRunModeStore = BACK_SUB_MODE_NO_ACTION ;
  //时间到了之后摇摆功能也要关闭 140906
  //RockFunctionEnable(false);    
  Main_Close_Power();
  nBuzzerMode = BUZZER_MODE_TWOTIME ;
  bSendBuzzerMode = TRUE;
  bBlueToothSendBuzzerMode = TRUE;
}

BYTE Main_GetKey(void)
{
    BYTE by_Key = H10_KEY_NONE;
    if(DMAUart_GetRXStatus() == TRUE)
    {
        DMAUart_ClearRXStatus();
        by_Key = DMAUart_GetKey();
        DMAUart_SetKey(H10_KEY_NONE);
    }
    return by_Key;
}

BYTE Main_GetKeyNoClear(void)
{
    BYTE by_Key = H10_KEY_NONE;
    if(DMAUart_GetRXStatus() == TRUE)
    {
        //DMAUart_ClearRXStatus();
        by_Key = DMAUart_GetKey();
        //DMAUart_SetKey(H10_KEY_NONE);
    }
  //if The command is from Bluetooth ,then awake from sleep mode for there's keys arrive.
  if(BlueToothUart_GetRXStatus() == TRUE)
  {
    by_Key = BlueToothUart_GetKey();
  }
    return by_Key;
}

void Eng_Send(void)
{
    if(bMasterSendPacket)
    {
        OutBuffer[0] = SOI ;

        unsigned int snH = DEVINFO->UNIQUEH;
        unsigned int snL = DEVINFO->UNIQUEL;

        OutBuffer[1] = (unsigned char)(snH >> 24);
        OutBuffer[2] = (unsigned char)(snH >> 16);
        OutBuffer[3] = (unsigned char)(snH >> 8);
        OutBuffer[4] = (unsigned char)(snH);

        OutBuffer[5] = (unsigned char)(snL >> 24);
        OutBuffer[6] = (unsigned char)(snL >> 16);
        OutBuffer[7] = (unsigned char)(snL >> 8);
        OutBuffer[8] = (unsigned char)(snL);

        OutBuffer[9] = (unsigned char)ReadEEByte(USER_DATA_BASE + SOFT_MAIN_VER_ADDRESS);
        OutBuffer[10] = (unsigned char)ReadEEByte(USER_DATA_BASE + SOFT_SECONDARY_VER_ADDRESS);
        OutBuffer[11] = (unsigned char)ReadEEByte(USER_DATA_BASE + SETTLE_ADDRESS);
        OutBuffer[12] = (unsigned char)ReadEEByte(USER_DATA_BASE + AIRBAG_STRETCH_ADDRESS);
        OutBuffer[13] = (unsigned char)ReadEEByte(USER_DATA_BASE + SLIDE_MOTOR_ENABLE_ADDRESS);

        OutBuffer[14] = EOI ;
        nOutBufferCount = 15;
        DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
        bMasterSendPacket = FALSE ;
    }
}
//
void Main_BlueToothSend(void)
{
  if(bBlueToothMasterSendPacket)
  {
    OutBufferBlueTooth[0] = SOI ;
    OutBufferBlueTooth[1] = 0;
    OutBufferBlueTooth[1] = 0x02;  //小腿伸缩图标变灰
    //OutBufferBlueTooth[1] |= 0x04;  //3D 标识
    //标识 1	按摩椅运行状态 1	按摩手法 3	按摩程序 3
    if(nChairRunState == CHAIR_STATE_IDLE || nChairRunState == CHAIR_STATE_SLEEP)
    {
      OutBufferBlueTooth[1] |= 0 << 6;
    }
    else
    {
      OutBufferBlueTooth[1] |= 1 << 6;
    }
    
    /*****************************************************/
    //按摩手法显示
    switch(nCurSubFunction)
    {
      //00：停止
      //01：揉捏
      //02：敲击
      //03：揉敲同步
      //04：叩击
      //05：指压
      //06：韵律按摩
      //07：保留
    case BACK_SUB_MODE_KNEAD:
      OutBufferBlueTooth[1] |= 1 << 3;
      break;
    case BACK_SUB_MODE_KNOCK:
      OutBufferBlueTooth[1] |= 2 << 3;
      break;
    case BACK_SUB_MODE_WAVELET:
      OutBufferBlueTooth[1] |= 3 << 3;
      break;
    case BACK_SUB_MODE_SOFT_KNOCK:
      OutBufferBlueTooth[1] |= 4 << 3;
      break;
    case BACK_SUB_MODE_PRESS:
      OutBufferBlueTooth[1] |= 5 << 3;
      break;
    case BACK_SUB_MODE_MUSIC:
      OutBufferBlueTooth[1] |= 6 << 3;
      break;
    default:
      OutBufferBlueTooth[1] |= 0 << 3;
      break;
    case BACK_SUB_MODE_RUBBING:
      OutBufferBlueTooth[1] |= 7 << 3;
      break;
    }
    /*****************************************************/
    /*
    if(nBackMainRunMode == BACK_MAIN_MODE_MANUAL)
    {
      OutBufferBlueTooth[1] |=  7;
    }
    else if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
    {
      BYTE mode = (nBackSubRunMode + 1);
      OutBufferBlueTooth[1] |= mode & 0x7;
    }
    else  if(nChairRunState == CHAIR_STATE_RUN)
    {
      OutBufferBlueTooth[1] |=  7;
    }
    */
    //标识 1 加热 1	保留 1	按摩机芯速度 3 	揉捏头宽度位置 2
    //00-03 自定义
  
    unsigned char speed;
    if(nBackMainRunMode == BACK_MAIN_MODE_IDLE)
    {
      speed = 0;
    }
    else
    {
      speed = nCurKneadKnockSpeed;
    }
    OutBufferBlueTooth[2] = ((bKeyWaistHeat & 0x1) << 6) | ((speed & 0x7) << 2) | (Input_GetKneadPosition() & 0x3);
    
    if(bRollerEnable)
    {
      OutBufferBlueTooth[2] |= (1 << 5);
    }
    else
    {
      OutBufferBlueTooth[2] &= ~(1 << 5);
    }
    // 标识 1	负离子开关 1 	 振动（或扭腰）强度 3	气压强度 3
    OutBufferBlueTooth[3] = (nKeySeatVibrateStrength & 0x7) << 3;
    
    if(nKeyAirBagLocate != AIRBAG_LOCATE_NONE)
    {
      OutBufferBlueTooth[3] |= (Valve_GetAirBagStrength() & 0x7);
    }
    //标识 1	机芯按摩部位 2	运行时间高5位 5
    //显示位置
    OutBufferBlueTooth[4] = 0;
    if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
    {
      if((nBackSubRunMode == BACK_SUB_MODE_AUTO_0) ||
         (nBackSubRunMode == BACK_SUB_MODE_AUTO_1) ||
           (nBackSubRunMode == BACK_SUB_MODE_AUTO_2) ||
             (nBackSubRunMode == BACK_SUB_MODE_AUTO_3))
      {
        OutBufferBlueTooth[4] = 1 << 5;
      }
      else
      {
        OutBufferBlueTooth[4] = 2 << 5;
      }
    }
    else
    {
      switch(nKeyBackLocate)
      {
      case LOCATE_FULL_BACK:
        OutBufferBlueTooth[4] = 1 << 5;
        break ;
      case LOCATE_PARTIAL:
        OutBufferBlueTooth[4] = 2 << 5;
        break ;
      case LOCATE_POINT:
        OutBufferBlueTooth[4] = 3 << 5; ;
        break ;
      default://include LOCATE_NONE
        //OutBufferBlueTooth[4] = 3<<5; ;
        break ;
      }
    }
    
#ifdef FORCE_CONTROLLER
    unsigned int time;
    //time = (KnockMotor_GetCurrent()&0x0f)<<4;
    time = KneadMotor_GetCurrent();
    time *= 60;
#else
    unsigned int time = Data_Get_TimeSecond();
#endif
    if(nChairRunState == CHAIR_STATE_DEMO)
    {
      time /= 60;    //demo模式 时间按分显示
    }
    OutBufferBlueTooth[4] |= (time >> 7) & 0x1f;
    //标识 1	运行时间低7位 7
    OutBufferBlueTooth[5] = time & 0x7f;
    /*
    unsigned int valveAction  = 0;
    
    switch(nKeyAirBagLocate)
    {
  case  AIRBAG_LOCATE_AUTO:
    valveAction = st_AirBagAuto.nCurPumpValveState;
    break;
  case  AIRBAG_LOCATE_LEG_FOOT:
    valveAction = st_AirBagLegFoot.nCurPumpValveState;
    break;
  case AIRBAG_LOCATE_SEAT:
    valveAction = st_AirBagSeat.nCurPumpValveState;
    break;
  case AIRBAG_LOCATE_ARM_SHOLDER:
    valveAction = st_AirBagArmSholder.nCurPumpValveState;
    break;
  case AIRBAG_LOCATE_BACK_WAIST:
    valveAction = st_AirBagBackWaist.nCurPumpValveState;
    break;
  }
    */
     OutBufferBlueTooth[6] = 0x00;
        if((ValveFungares13) | (ValveFungares14))
        {
            OutBufferBlueTooth[6] |= 0x01;
        }
        if((ValveFungares11) | (ValveFungares12))
        {
            OutBufferBlueTooth[6] |= 0x02;
        }
        if((ValveFungares7) | (ValveFungares8))
        {
            OutBufferBlueTooth[6] |= 0x04;
        }
        if((ValveFungares3) | (ValveFungares4) | (ValveFungares5) | (ValveFungares6))
        {
            OutBufferBlueTooth[6] |= 0x10;
        }

        if(bRollerEnable)
        {
            if(Valve_RollerIsAuto()== 0) 
            {
                // if(bDisplayFlash) OutBuffer[6] |= (3<<5);
                // else  OutBuffer[6] |= (0<<5);

                unsigned int rollerPWM;
                rollerPWM = displayPWM;
                if(rollerPWM == ROLLER_SPEED_STOP) OutBufferBlueTooth[6] |= (0 << 5);
                else if(rollerPWM == ROLLER_SPEED_SLOW) OutBufferBlueTooth[6] |= (1 << 5);
                else if(rollerPWM == ROLLER_SPEED_MID) OutBufferBlueTooth[6] |= (2 << 5);
                else if(rollerPWM == ROLLER_SPEED_FAST) OutBufferBlueTooth[6] |= (3 << 5);
            }
            else
            {
                OutBufferBlueTooth[6] |= (Valve_GetRollerLevel() << 5);
            }
        }
        else
        {
            OutBufferBlueTooth[6] |= (0 << 5);
        }

    OutBufferBlueTooth[7] = 0x00;
    
    if((ValveFungares1) | (ValveFungares2))
    {
      OutBufferBlueTooth[7] |=  0x10;
    }
    /*
    if((bBackWaistRightUp) | (bBackWaistRightDown) | (bBackWaistLeftUp) | (bBackWaistLeftDown))
    {
      OutBufferBlueTooth[7] |=  0x20;
    }
    */
    OutBufferBlueTooth[7] &= 0xf0;
    
    BYTE state = nChairRunState;
    if(nChairRunState == CHAIR_STATE_SLEEP)
    {
      state = CHAIR_STATE_IDLE;
    }
    if(nChairRunState == CHAIR_STATE_DEMO)
    {
      state = CHAIR_STATE_RUN;
    }
    OutBufferBlueTooth[7] |= (state & 0x0f);
    
    /*
    int data;
    if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
    {
    data = nFinalWalkMotorLocate; //自动模式使用肩膀位置
  }
    else
    {
    data = TOP_POSITION;   //手动模式使用自动形成
  }
    */
    unsigned int data = Input_GetWalkMotorPosition();
    data /= 30;
    if(data >= 13) data = 13;
    OutBufferBlueTooth[8] = data;
    
    OutBufferBlueTooth[9] = 0;//fww
    if(nBackMainRunMode == BACK_MAIN_MODE_AUTO &&
                nReworkShoulderPosition == 2 && nBackSubRunMode != BACK_SUB_MODE_AUTO_5)   //开始检测
        {
            OutBufferBlueTooth[9] |= 1 << 6;
        }
        else
        {
            OutBufferBlueTooth[9] |= 0 << 6;
        }

        if(nCurSubFunction == BACK_SUB_MODE_BODY_DETECT_1)
        {
            OutBufferBlueTooth[9] |= 1 << 5;
        }
        else
        {
            OutBufferBlueTooth[9] |= 0 << 5;
        }
        if(bFail == 0 && bodyDetectSuccess == 1)
        {
            OutBufferBlueTooth[9] |= 1 << 4;
        }
        else if(bFail == 1 && bodyDetectSuccess == 0)
        {
            OutBufferBlueTooth[9] |= 0 << 4;
        }


        if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) && (nCurSubFunction == BACK_SUB_MODE_BODY_DETECT_1))
        {
            data = nShoulderPositionTop - nShoulderPositionBottom;
            time = data / 15;
            data = (Input_GetWalkMotorPosition() - nShoulderPositionBottom) / time;
            if(data == 0) data = 1;
            if(data > 15) data = 15;
        }
        else
            data = 0;

        OutBufferBlueTooth[9] |= data & 0x0f;

   
    OutBufferBlueTooth[10] = 0;
    if(isZeroPosition())
    {
      OutBufferBlueTooth[10] = 1 << 6;
    }
    /**************************fww***********************
    if(SlideMotor_GetPower() == SLIDE_MOTOR_POWER_ON)
    {
      if(SlideMotor_GetDirection() == SLIDE_MOTOR_GO_FORWARD)
      {
        OutBufferBlueTooth[10] = 0x01 << 4;
      }
      else
      {
        OutBufferBlueTooth[10] = 0x02 << 4;
      }
    }
    **************************fww*************************/
    if(BackMotor_GetPower() == BACK_MOTOR_POWER_ON)
    {
      if(BackMotor_GetDirection() == BACK_MOTOR_GO_UP)
      {
        OutBufferBlueTooth[10] |= 0x01;
      }
      else
      {
        OutBufferBlueTooth[10] |= 0x02;
      }
    }
    
    if(LegMotor_GetPower() == LEG_MOTOR_POWER_ON)
    {
      if(LegMotor_GetDirection() == LEG_MOTOR_GO_UP)
      {
        OutBufferBlueTooth[10] |= (0x01 << 2);
      }
      else
      {
        OutBufferBlueTooth[10] |= (0x02 << 2);
      }
    }
    
     //标识	1 蜂鸣器模式 2 音乐开关 1	音量 4
      if(bBlueToothSendBuzzerMode == TRUE)
      {
        OutBufferBlueTooth[11] = (nBuzzerMode&0x3)<<5;
        bBlueToothSendBuzzerMode = FALSE ;
      }
      else
      {
        OutBufferBlueTooth[11] = 0;
      }
    
    /*
    //标识	1 蜂鸣器模式 2 音乐开关 1	音量 4
    if(bBlueToothSendBuzzerMode == TRUE)
    {
      OutBufferBlueTooth[11] = (nBuzzerMode & 0x3) << 5;
      bBlueToothSendBuzzerMode = FALSE ;
    }
    else
    {
      OutBufferBlueTooth[11] = 0;
    }
    */
//    OutBufferBlueTooth[11] |= ((nvcBluetoothPower & 0x1) << 4);
    
    switch(w_PresetTime)
    {
    case RUN_TIME_10:
      OutBufferBlueTooth[12] = 1;
      break;
    case RUN_TIME_20:
      OutBufferBlueTooth[12] = 2;
      break;
    case RUN_TIME_30:
      OutBufferBlueTooth[12] = 3;
      break;
    default:
      OutBufferBlueTooth[12] = 0;
      break;
    }
    unsigned int locate = 0;
    switch(nKeyAirBagLocate)
    {
    case AIRBAG_LOCATE_NONE: break;
    case AIRBAG_LOCATE_LEG_FOOT:locate = 0x04;break;
    case AIRBAG_LOCATE_ARM_SHOLDER: locate = 0x10; break;
    case AIRBAG_LOCATE_SEAT:locate = 0x20;break;
    case AIRBAG_LOCATE_AUTO:locate = 0x40;break;
    }
    
    OutBufferBlueTooth[12] |= locate;
    
    //滚轮方向
    if(bRollerEnable)
    {
    
      if(1)
      {
        OutBufferBlueTooth[13] = 1;
      }
      
    }
    else
    {
      OutBufferBlueTooth[13] = 0;
    }
    
     if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
    {
      BYTE mode = (nBackSubRunMode + 1);
      OutBufferBlueTooth[13] |= (mode & 0x0f) << 2;
    }
    
    else if(nChairRunState == CHAIR_STATE_RUN)
    {
        OutBufferBlueTooth[13] |=  7 << 2;
    }

    OutBufferBlueTooth[14] =0;
    unsigned char checkSum = 0;
    for(int i=1;i<15;i++)
    {
      checkSum += OutBufferBlueTooth[i];
    }
    checkSum = ~checkSum;
    checkSum &= 0x7f;
    OutBufferBlueTooth[15] = checkSum;
    OutBufferBlueTooth[16] = EOI;
    nOutBufferBlueToothCount = 17;
    BlueToothUart_Transmit_Packet(OutBufferBlueTooth, nOutBufferBlueToothCount);
    bBlueToothMasterSendPacket = FALSE ;
  }
}
//
unsigned int data;
void Main_Send(void)
{
	if(bMasterSendPacket)
	{
		OutBuffer[0] = SOI ;
		//标识 1	按摩椅运行状态 1	按摩手法 3	按摩程序 3
		if(nChairRunState == CHAIR_STATE_IDLE || nChairRunState == CHAIR_STATE_SLEEP)
		{
			OutBuffer[1] = 0 << 6;
		}
		else
		{
			OutBuffer[1] = 1 << 6;
		}

		/*****************************************************/
		//按摩手法显示
		switch(nCurSubFunction)
		{
            //00：停止
            //01：揉捏
            //02：敲击
            //03：揉敲同步
            //04：叩击
            //05：指压
            //06：韵律按摩
            //07：保留
			case BACK_SUB_MODE_KNEAD:
				OutBuffer[1] |= 1 << 3;
				break;
			case BACK_SUB_MODE_KNOCK:
				OutBuffer[1] |= 2 << 3;
				break;
			case BACK_SUB_MODE_WAVELET :
				OutBuffer[1] |= 3 << 3;
				break;
			case BACK_SUB_MODE_SOFT_KNOCK:
				OutBuffer[1] |= 4 << 3;
				break;
			case BACK_SUB_MODE_PRESS:
				OutBuffer[1] |= 5 << 3;
				break;
			case BACK_SUB_MODE_MUSIC:
				OutBuffer[1] |= 6 << 3;
				break;
			default:
				OutBuffer[1] |= 0 << 3;
				break;
			case BACK_SUB_MODE_RUBBING:
				OutBuffer[1] |= 7 << 3;
				break;
		}
        /*****************************************************/
        /*
             switch(nBackSubRunMode){
              //00：停止
              //01：揉捏
              //02：敲击
              //03：揉敲同步
              //04：叩击
              //05：指压
              //06：韵律按摩
              //07：保留
            case BACK_SUB_MODE_KNEAD			: OutBuffer[1] |= 1<<3;break;
            case BACK_SUB_MODE_KNOCK			: OutBuffer[1] |= 2<<3;break;
            case BACK_SUB_MODE_WAVELET		        : OutBuffer[1] |= 3<<3;break;
            case BACK_SUB_MODE_SOFT_KNOCK		: OutBuffer[1] |= 4<<3;break;
            case BACK_SUB_MODE_PRESS			: OutBuffer[1] |= 5<<3;break;
            case BACK_SUB_MODE_MUSIC			: OutBuffer[1] |= 6<<3;break;
            default		: OutBuffer[1] |= 0<<3;break;
            }
             */
        if(nBackMainRunMode == BACK_MAIN_MODE_MANUAL)
        {
            OutBuffer[1] |=  7;
        }
        else if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
        {

            BYTE mode = (nBackSubRunMode + 1);

            if(nChairRunState == CHAIR_STATE_DEMO)
            {
                mode &= bDisplayFlash;
            }

            OutBuffer[1] |= mode & 0x7;
        }
        else  if(nChairRunState == CHAIR_STATE_RUN)
        {
            OutBuffer[1] |=  7;
        }
        /*
            if(bDisplayFlash)
            {
              OutBuffer[1] = 0xff;
            }
            else
            {
              OutBuffer[1] = 0x00;
            }
            */
        //标识 1 加热 1	保留 1	按摩机芯速度 3 	揉捏头宽度位置 2
        //00-03 自定义
        unsigned char speed;
        if(nBackMainRunMode == BACK_MAIN_MODE_IDLE)
        {
            speed = 0;
        }
        else
        {
            speed = nCurKneadKnockSpeed;
            if(nCurSubFunction == BACK_SUB_MODE_PRESS)
              speed = 0;                                   //fww 如果是指压，不显示速度。
        }
        OutBuffer[2] = ((bKeyWaistHeat & 0x1) << 6) | ((speed & 0x7) << 2) | (Input_GetKneadPosition() & 0x3);

        if(bRollerEnable)
        {
            OutBuffer[2] |= (1 << 5);
        }
        else
        {
            OutBuffer[2] &= ~(1 << 5);
        }
        // 标识 1	负离子开关 1 	 振动（或扭腰）强度 3	气压强度 3
        OutBuffer[3] = (nKeySeatVibrateStrength & 0x7) << 3;

        if(nKeyAirBagLocate != AIRBAG_LOCATE_NONE)
        {
            OutBuffer[3] |= (Valve_GetAirBagStrength() & 0x7);
        }
        //标识 1	机芯按摩部位 2	运行时间高5位 5
        //显示位置
        OutBuffer[4] = 0;
        if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
        {
            if((nBackSubRunMode == BACK_SUB_MODE_AUTO_0) ||
                    (nBackSubRunMode == BACK_SUB_MODE_AUTO_1) ||
                    (nBackSubRunMode == BACK_SUB_MODE_AUTO_2) ||
                    (nBackSubRunMode == BACK_SUB_MODE_AUTO_3))
            {
                OutBuffer[4] = 1 << 5;
            }
            else
            {
                OutBuffer[4] = 2 << 5;
            }
        }
        else
        {
            switch(nKeyBackLocate)
            {
            case LOCATE_FULL_BACK:
                OutBuffer[4] = 1 << 5;
                break ;
            case LOCATE_PARTIAL:
                OutBuffer[4] = 2 << 5;
                break ;
            case LOCATE_POINT:
                OutBuffer[4] = 3 << 5; ;
                break ;
            default://include LOCATE_NONE
                //OutBuffer[4] = 3<<5; ;
                break ;
            }
        }
        /*
        if(nKeyBackLocate==LOCATE_POINT){
          OutBuffer[4] = 3<<5;
        }else if(nKeyBackLocate==LOCATE_PARTIAL){
          OutBuffer[4] = 2<<5;
        }else if(nKeyBackLocate==LOCATE_FULL_BACK){
          OutBuffer[4] = 1<<5;
        }else{
          OutBuffer[4] = 0;
        }
        */
#ifdef FORCE_CONTROLLER
        unsigned int time;
        //time = (KnockMotor_GetCurrent()&0x0f)<<4;
        time = KneadMotor_GetCurrent();
        time *= 60;
#else
        unsigned int time = Data_Get_TimeSecond();
#endif
        if(nChairRunState == CHAIR_STATE_DEMO)
        {
          time /= 60;    //demo模式 时间按分显示
        }
        OutBuffer[4] |= (time >> 7) & 0x1f;
        //标识 1	运行时间低7位 7
        OutBuffer[5] = time & 0x7f;
        /*
        unsigned int valveAction  = 0;

        switch(nKeyAirBagLocate)
        {
        case  AIRBAG_LOCATE_AUTO:
          valveAction = st_AirBagAuto.nCurPumpValveState;
          break;
        case  AIRBAG_LOCATE_LEG_FOOT:
          valveAction = st_AirBagLegFoot.nCurPumpValveState;
          break;
        case AIRBAG_LOCATE_SEAT:
          valveAction = st_AirBagSeat.nCurPumpValveState;
          break;
        case AIRBAG_LOCATE_ARM_SHOLDER:
          valveAction = st_AirBagArmSholder.nCurPumpValveState;
          break;
        case AIRBAG_LOCATE_BACK_WAIST:
          valveAction = st_AirBagBackWaist.nCurPumpValveState;
          break;
        }
        */
        OutBuffer[6] = 0x00;
        if((ValveFungares13) | (ValveFungares14))
        {
            OutBuffer[6] |= 0x01;
        }
        if((ValveFungares11) | (ValveFungares12))
        {
            OutBuffer[6] |= 0x02;
        }
        if((ValveFungares7) | (ValveFungares8 ))
        {
            OutBuffer[6] |= 0x04;//座垫=背部
        }
        if((ValveFungares3) | (ValveFungares4) | (ValveFungares5) | (ValveFungares6))
        {
            OutBuffer[6] |= 0x10;
        }

        if(bRollerEnable)
        {
            if(Valve_RollerIsAuto())
            {
                // if(bDisplayFlash) OutBuffer[6] |= (3<<5);
                // else  OutBuffer[6] |= (0<<5);
                unsigned int rollerPWM;
                rollerPWM = displayPWM;
                if(rollerPWM == ROLLER_SPEED_STOP) OutBuffer[6] |= (0 << 5);
                else if(rollerPWM == ROLLER_SPEED_SLOW) OutBuffer[6] |= (1 << 5);
                else if(rollerPWM == ROLLER_SPEED_MID) OutBuffer[6] |= (2 << 5);
                else if(rollerPWM == ROLLER_SPEED_FAST) OutBuffer[6] |= (3 << 5);
            }
            else
            {
                OutBuffer[6] |= (Valve_GetRollerLevel() << 5);
            }
        }
        else
        {
            OutBuffer[6] |= (0 << 5);
        }
        OutBuffer[7] = 0x0;

        if((ValveFungares1) | (ValveFungares2))
        {
            OutBuffer[7] |=  0x10;
        }/*
        if((ValveFungares7) | (ValveFungares8 ))
        {
            OutBuffer[6] |= 0x20;//背部
        }*/
        /*
        if((bBackWaistRightUp) | (bBackWaistRightDown) | (bBackWaistLeftUp) | (bBackWaistLeftDown))
        {
            OutBuffer[7] |=  0x20;
        }
        */
        OutBuffer[7] &= 0xf0;

        BYTE state = nChairRunState;
        if(nChairRunState == CHAIR_STATE_SLEEP)
        {
            state = CHAIR_STATE_IDLE;
        }
        if(nChairRunState == CHAIR_STATE_DEMO)
        {
            state = CHAIR_STATE_RUN;
        }
        OutBuffer[7] |= (state & 0x0f);

        /*
        int data;
        if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
        {
          data = nFinalWalkMotorLocate; //自动模式使用肩膀位置
        }
        else
        {
          data = TOP_POSITION;   //手动模式使用自动形成
        }
        */
        data = Input_GetWalkMotorPosition();
        data /= 30;//31
        if(data >= 13) data = 13;
        OutBuffer[8] = data;

//20181019
//        if(nBackMainRunMode == BACK_MAIN_MODE_AUTO &&
//                nReworkShoulderPosition == 2 && nBackSubRunMode != BACK_SUB_MODE_AUTO_5)   //开始检测
//        {
//            OutBuffer[9] = 0 << 6;
//        }
//        else
//        {
//            OutBuffer[9] = 0 << 6;
//        }

//        if(nCurSubFunction == BACK_SUB_MODE_BODY_DETECT_1)
//        {
//            OutBuffer[9] |= 1 << 5;  //
//            OutBuffer[9] |= 1 << 6;
//        }
//        else
//        {
//            OutBuffer[9] |= 0 << 5;  //
//            OutBuffer[9] &= 0xbf ;//<< 6;
//        }
        OutBuffer[9] = 0;
        
        if(bDisplayDetect)
        {
            OutBuffer[9] |= 1 << 5;  //
            OutBuffer[9] |= 1 << 6;
        }
        else
        {
            OutBuffer[9] |= 0 << 5;  //
            OutBuffer[9] &= 0xbf ;//<< 6;
        }
/*        if(bFail == 0 && bodyDetectSuccess == 1)
        {
            OutBuffer[9] |= 1 << 4;
        }
        else if(bFail == 1 && bodyDetectSuccess == 0)
        {
            OutBuffer[9] |= 0 << 4;
        }
*/

        if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) && (nCurSubFunction == BACK_SUB_MODE_BODY_DETECT_1))
        {
            data = nShoulderPositionTop - nShoulderPositionBottom;
            time = data / 15;
            data = (Input_GetWalkMotorPosition() - nShoulderPositionBottom) / time;
            if(data == 0) data = 1;
            if(data > 15) data = 15;
        }
        else
            data = 0;

        OutBuffer[9] |= data & 0x0f;

        // OutBuffer[9] = Input_GetVout();

        //标识 1	运行指示 1	小腿电动缸运行方向指示 3	靠背电动缸运行方向指示 3
        //OutBuffer[10] = ((bLegPadMotorPowerFlag || bBackPadMotorPowerFlag)&0x1)<<6;
        //OutBuffer[10] |=(nCurLegPadMotorState&0x7)<<3;
        //OutBuffer[10] |= nCurBackPadMotorState&0x7;
        OutBuffer[10] = 0;
        if(isZeroPositionNew())
        {
            OutBuffer[10] = 1 << 6;
        }
        if(SlideMotor_GetPower() == SLIDE_MOTOR_POWER_ON)
        {
            if(SlideMotor_GetDirection() == SLIDE_MOTOR_GO_FORWARD)
            {
                OutBuffer[10] = 0x01 << 4;
            }
            else
            {
                OutBuffer[10] = 0x02 << 4;
            }
        }
        if(BackMotor_GetPower() == BACK_MOTOR_POWER_ON)
        {
            if(BackMotor_GetDirection() == BACK_MOTOR_GO_UP)
            {
                OutBuffer[10] |= 0x01;
            }
            else
            {
                OutBuffer[10] |= 0x02;
            }
        }

        if(LegMotor_GetPower() == LEG_MOTOR_POWER_ON)
        {
            if(LegMotor_GetDirection() == LEG_MOTOR_GO_UP)
            {
                OutBuffer[10] |= (0x01 << 2);
            }
            else
            {
                OutBuffer[10] |= (0x02 << 2);
            }
        }

        //标识	1 蜂鸣器模式 2 音乐开关 1	音量 4
        if(bSendBuzzerMode == TRUE)
        {
            OutBuffer[11] = (nBuzzerMode & 0x3) << 5;
            // printf("b:%d\n\r",nBuzzerMode);
            bSendBuzzerMode = FALSE ;
        }
        else
        {
            OutBuffer[11] = 0;
        }
        OutBuffer[11] |= ((nvcBluetoothPower & 0x1) << 4);
     
        switch(w_PresetTime)
        {
        case RUN_TIME_10:
            OutBuffer[12] = 1;
            break;
        case RUN_TIME_20:
            OutBuffer[12] = 2;
            break;
        case RUN_TIME_30:
            OutBuffer[12] = 3;
            break;
        default:
            OutBuffer[12] = 0;
            break;
        }
        //OutBuffer[12] |= ((nKeyAirBagLocate & 0x07) << 2);//Fungares

        
        switch(nKeyAirBagLocate)
        {
        case 0:
          OutBuffer[12] |= 0x00;
          break;
        case 1:
          OutBuffer[12] |= 0x40;
          break; 
        case 2:
          OutBuffer[12] |= 0x10;
          break;
        case 3:
          OutBuffer[12] |= 0x20;
          break;
        case 4:
          OutBuffer[12] |= 0x04;
          break;
        default:
          OutBuffer[12] |= 0x00;
          break;
        }
        /********************************************/
        //00 停止 01顺转 02反转 03正反转
        //OutBuffer[13] |= Roller_GetRollerDirection() & 0x3;//Fungares
         OutBuffer[13] |= Roller_GetRollerDirection() & 0x3; 
        
        /********************************************/
        OutBuffer[14] = EOI;
        //OutBuffer[15] = EOI;
        nOutBufferCount = 15;
        //nOutBufferCount = 16;
        DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
        
        bMasterSendPacket = FALSE ;
    }
}
/***********************************************
将显示缓冲区中的数据发送到手控器
***********************************************/
bool bBlueToothPowerSwitchFlag;
void CommProcess(void)
{
	unsigned int pw_Information[5];//fww
	
	if(bBlueToothPowerSwitchFlag == TRUE)
	{
		bBlueToothPowerSwitchFlag = FALSE;
		memset(pw_Information, 0, sizeof(pw_Information));        //fww
		PBYTE pInformation = (PBYTE)pw_Information;              //fww
		MEM_Read_Memory(pw_Information, MEMORY_LENGTH_OF_BYTES);  //fww
		nvcBluetoothPower = (nvcBluetoothPower + 1) % 2;
		*(pInformation + SLIDE_BLUETOOTH_POWER) = nvcBluetoothPower;  //fww
		MEM_Write_Memory(pw_Information, MEMORY_LENGTH_OF_BYTES);   //fww
		if(nvcBluetoothPower == 1)
		{
			Power_AMP_On();
		}
		else
		{
			Power_AMP_Off();
		}  
	}
  //数据包的处理
  if((DMAUart_GetRXStatus() == TRUE) || (BlueToothUart_GetRXStatus() == TRUE))
  {
    /*
    nChairStateCount = 0 ;  //收到按键，则wait command时间清零
    DMAUart_ClearRXStatus();//bReceivePacket = FALSE ;
    */
    if(BlueToothUart_GetRXStatus() == TRUE)
    {
      nCommandID = BlueToothUart_GetKey();
      BlueToothUart_ClearRXStatus();
      WL_SetReadyToSendFlag(false);
      //bMasterSendPacketWL = true;//fww
      //bMasterSendPacketDecide = 1;  //加此是为了在调整部位的时候的处理
    }
    if(DMAUart_GetRXStatus() == TRUE)
    {
      nCommandID = DMAUart_GetKey();
      DMAUart_ClearRXStatus();//bReceivePacket = FALSE ;
      //bMasterSendPacketDecide = 0;  //加此是为了在调整部位的时候的处理
    }
    
    //printf("key=%d\n",nCommandID);
    
    if((DMAUart_GetCtrlType() != ENGGER_CTRL)&&(BlueToothUart_GetCtrlType() != ENGGER_CTRL))
    {
      //nCommandID = DMAUart_GetKey();
      DMAUart_SetKey(H10_KEY_NONE);
      BlueToothUart_SetKey(H10_KEY_NONE);
      
      if((nChairRunState == CHAIR_STATE_SETTLE)/* || (bBackLegPadSettle == TRUE)*/)
      {
        if(nCommandID != H10_KEY_NONE)
        {
          nCommandID = H10_KEY_NONE;
          Power_All_Off();
          Main_Initial_IO();
          Main_Initial_Data();
          bBackLegPadSettle = FALSE;
          nChairRunState = CHAIR_STATE_IDLE;   //1122开机进入待运行模式
        }
      }
      nChairStateCount=0;//fww
      //140528
      /*
      if((nTimerOverResetAgainFlag == SECOND_TIME_ENTER) && (nChairRunState == CHAIR_STATE_IDLE))
      {
      if(nCommandID != H10_KEY_NONE)
      {
      nCommandID = H10_KEY_NONE;
      //Power_All_Off();
      Main_Initial_IO();
      //Main_Initial_Data();
      bBackLegPadSettle = FALSE;
      //nChairRunState = CHAIR_STATE_SETTLE_1ST;   //进入等待选择界面
      bResetStateORchangeModeDis = EnterDisReset ;
      bSelectKeyValue = KEY_NOT_CONFIRMORCANCLE ;
      bResetFlag = RESET_CHAIR ;
    }
    }*/
      //
      if((nChairRunState != CHAIR_STATE_SETTLE) && (nChairRunState != CHAIR_STATE_PROBLEM))
      {
        switch(nCommandID)
        {
        case H10_KEY_MENU  :
          
          if(nChairRunState != CHAIR_STATE_WAIT_COMMAND) break;
          nChairStateCount = 0 ;
          break;
          
          //Power Switch Key
          //bKeyPowerSwitch == FALSE 有3种情况:IDLE,SETTLE,WAIT_MEMORY
          //bKeyPowerSwitch == TRUE 有2种情况:RUN,SETTLE,WAIT_COMMAND
        case H10_KEY_POWER_SWITCH:
          /*
          static int acount=0;
          acount++;
          printf("step:%d\n",acount);
          */
          if(nChairRunState == CHAIR_STATE_SETTLE_1ST) break ;
          nCurSubFunction = 0;
          bBodyDetectSuccess = FALSE ;
          nReworkShoulderPosition = 2;
          bodyDetectSuccess = 0;
          bFail = 0;
          st_Stretch.active = FALSE;
#ifdef SN_CHECK_EN
          nSendSNFlag = FALSE ;
#endif
          if(bKeyPowerSwitch == FALSE)
          {
            //  st_Stretch.bBackLegFlag = FALSE;
            nWidthOverTime = 0;
            nWalkOverTime = 0;
            //if(nChairRunState == CHAIR_STATE_IDLE)//idle
            {
              bBackLegPadSettle = FALSE ;
              bKeyBackPadUp = FALSE ;
              bKeyBackPadDown = FALSE ;
              bKeyLegPadUp = FALSE ;
              bKeyLegPadDown = FALSE ;
              
              //140906
              //RockFunctionEnable(false);
              
              bKeyPowerSwitch = TRUE ;
              nChairRunState = CHAIR_STATE_WAIT_COMMAND ;//按摩椅等待按键命令
              nChairStateCount = 0 ;
              Data_Set_Start(0, 0);
              nBackSubRunMode = BACK_SUB_MODE_NO_ACTION ;
              //140528
              //bResetFlag = NORESET_CHAIR ;//fww
              //bResetStateORchangeModeDis =  ExitDisReset;
              //
              //bPowerOffReach = FALSE;
              //Power_On();
              //nRunTime = 0 ;
              //bRunTimeChange = TRUE ;
            }
            //else //settle,wait_memory
            {
              //break ;
            }
          }
          else//run,wait_command
          {
            if(nChairRunState == CHAIR_STATE_WAIT_COMMAND)
            {
              //nChairRunState = CHAIR_STATE_IDLE ;
              bBackLegPadSettle = TRUE ;
              nChairRunState = CHAIR_STATE_SETTLE ;
            }
            // else if(nChairRunState == CHAIR_STATE_RUN)
            {
              //按摩机构回位
              nChairRunState = CHAIR_STATE_SETTLE ;
              nChairStateCount2 = 0;//fww 急停计数开始。
              //nChairRunState = CHAIR_STATE_SETTLE_1ST ;
              nBackMainRunMode = BACK_MAIN_MODE_SETTLE ; 
              nBackSettleStep = 0 ;
              nBackSettleReason = PARK_KEY_STOP ;
            }
            bBackLegPadSettle = TRUE ;
            //nZLB_RunState = 0;
            
            nTargetMassagePosition = MASSAGE_RESET_POSITION;
            bMassagePositionUpdate = TRUE;
            
            // bZLBMotorRunFlag = TRUE;
            //140528
            //bResetFlag = RESET_CHAIR ;
            //bResetStateORchangeModeDis =  EnterDisReset ;
            //140529
            //bSelectKeyValue = KEY_NOT_CONFIRMORCANCLE ;
            
            //nChairResetReason = POWERKEY_PRESSED ;//fww
            //140530
            //w_PresetTimeStore = Data_Get_TimeSecond() ;
            //140906
           // RockFunctionEnable(false);
            //
            Main_Close_Power();
          }
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          break ;
					//ZONE1:Full Back Auto Function Key
					case H10_KEY_BLUETOOTH_POWER_SWITCH:
						//切换 
						/*	
						memset(pw_Information, 0, sizeof(pw_Information));        //fww
						PBYTE pInformation = (PBYTE)pw_Information;              //fww
						MEM_Read_Memory(pw_Information, MEMORY_LENGTH_OF_BYTES);  //fww
						nvcBluetoothPower = (nvcBluetoothPower + 1) % 2;
						*(pInformation + SLIDE_BLUETOOTH_POWER) = nvcBluetoothPower;  //fww
						MEM_Write_Memory(pw_Information, MEMORY_LENGTH_OF_BYTES);   //fww
						if(nvcBluetoothPower == 1)
						{
							Power_AMP_On();
						}
						else
						{
							Power_AMP_Off();
						}  
						*/
						bBlueToothPowerSwitchFlag = TRUE;
						nBuzzerMode = BUZZER_MODE_ONETIME ;
						bSendBuzzerMode = TRUE ;
						bBlueToothSendBuzzerMode = TRUE;
						break;
        case H10_KEY_CHAIR_AUTO_0:
#ifdef SN_CHECK_EN
          if(bKeyPowerSwitch == FALSE)
          {
            SN_CHECK_KEY_INPUT[0] = SN_CHECK_KEY_INPUT[1] ;
            SN_CHECK_KEY_INPUT[1] = SN_CHECK_KEY_INPUT[2] ;
            SN_CHECK_KEY_INPUT[2] = SN_CHECK_KEY_INPUT[3] ;
            SN_CHECK_KEY_INPUT[3] = SN_CHECK_KEY_INPUT[4] ;
            SN_CHECK_KEY_INPUT[4] = H10_KEY_CHAIR_AUTO_0 ;
            if(SNCheckKeyCompare() == TRUE)
            {
              nSendSNFlag = TRUE ;
              //                      bCancelErr = TRUE;
              nSendSNStep = 0 ;
            }
          }
#endif
          if(bKeyPowerSwitch == FALSE) break ;
          //140906
          //RockFunctionEnable(false);
                  
          bKneadMotorInProcess = FALSE ;
          if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
          {
            nTargetMassagePosition = MASSAGE_OPTIMAL_POSITION;
            bMassagePositionUpdate = TRUE;
            //140529
            //bSelectKeyValue = KEY_NOT_CONFIRMORCANCLE ;
            //bResetFlag = CHANGE_WORKMODE ;
            //nChairResetReason = CHANGE_MODE ;
            //bResetStateORchangeModeDis =  EnterDisReset ;
          }
#ifdef FOOT_ROLLER_ENABLE
          bRollerEnable = TRUE;
          bRollerEnableStore = TRUE;
          if(nRollerPWM == 0)
          {
            nRollerPWMStore = 2;
            nRollerPWM = 2;
            Valve_SetRollerPWM(nRollerPWM);
          }
#endif
          ////////////////////////////
#ifdef POWER_VIBRATION
          bKeySeatVibrate = TRUE ;
          nKeySeatVibrateStrength = 2 ;
#endif
          ////////////////////////////
          bReconfigFlag = FALSE ;
          nChairRunState = CHAIR_STATE_RUN ;
          nIndicateTimer = RUN_INDICATE_TIME;
          /////////////////////
          //设置背部功能
          if(!((nBackMainRunMode == BACK_MAIN_MODE_AUTO) &&
               (nBackSubRunMode == BACK_SUB_MODE_AUTO_0)))
          {
            nBackMainRunMode = BACK_MAIN_MODE_AUTO ;
            nBackSubRunMode = BACK_SUB_MODE_AUTO_0 ;
            nCurBackMainRunModeStore =  BACK_MAIN_MODE_AUTO ;
            nCurBackSubRunModeStore = BACK_SUB_MODE_AUTO_0 ;
            
            bBackAutoModeInit = TRUE ;
            bReconfigFlag = TRUE ;
          }
          //设置气囊功能
          // if(nKeyAirBagLocate != AIRBAG_LOCATE_AUTO)
          {
            nKeyAirBagLocate = AIRBAG_LOCATE_AUTO ;
            //140531
            nKeyAirBagLocateStore = AIRBAG_LOCATE_AUTO ;
            
            if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
            {
              Valve_SetAirBagStrength(AIRBAG_STRENGTH_3);
            }
            /*****************Fungares**********************/
            st_AirBagAuto0.init = TRUE ;
            
            bReconfigFlag = TRUE ;
          }
          //只有在需要重新配置时才起作用
          if(bReconfigFlag == TRUE)
          {
            //设置时间
            Data_Set_Start(1, w_PresetTime);
            nBuzzerMode = BUZZER_MODE_ONETIME ;
            bSendBuzzerMode = TRUE ;
            bBlueToothSendBuzzerMode = TRUE;
          }
          break ;
        case H10_KEY_CHAIR_AUTO_1:
#ifdef SN_CHECK_EN
          if(bKeyPowerSwitch == FALSE)
          {
            SN_CHECK_KEY_INPUT[0] = SN_CHECK_KEY_INPUT[1] ;
            SN_CHECK_KEY_INPUT[1] = SN_CHECK_KEY_INPUT[2] ;
            SN_CHECK_KEY_INPUT[2] = SN_CHECK_KEY_INPUT[3] ;
            SN_CHECK_KEY_INPUT[3] = SN_CHECK_KEY_INPUT[4] ;
            SN_CHECK_KEY_INPUT[4] = H10_KEY_CHAIR_AUTO_1 ;
            if(SNCheckKeyCompare() == TRUE)
            {
              nSendSNFlag = TRUE ;
              nSendSNStep = 0 ;
            }
          }
#endif
          if(bKeyPowerSwitch == FALSE) break ;

          bKneadMotorInProcess = FALSE ;

          if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
          {
            nTargetMassagePosition = MASSAGE_OPTIMAL_POSITION;
            bMassagePositionUpdate = TRUE;
          }
#ifdef FOOT_ROLLER_ENABLE
          bRollerEnable = TRUE;
          bRollerEnableStore = TRUE;
          if(nRollerPWM == 0)
          {
            nRollerPWMStore = 2;
            nRollerPWM = 2;
            Valve_SetRollerPWM(nRollerPWM);
          }
#endif
          ////////////////////////////
#ifdef POWER_VIBRATION
          bKeySeatVibrate = TRUE ;
          //bGetNextVibStep = TRUE ;
          //nCurVibStep = 0 ;
          //nCurAutoVibItem = 0 ;
          nKeySeatVibrateStrength = 2 ;
          
#endif
          ////////////////////////////
          bReconfigFlag = FALSE ;
          nChairRunState = CHAIR_STATE_RUN ;
          nIndicateTimer = RUN_INDICATE_TIME;
          /////////////////////
          //设置背部功能
          if(!((nBackMainRunMode == BACK_MAIN_MODE_AUTO) &&
               (nBackSubRunMode == BACK_SUB_MODE_AUTO_1)))
          {
            nBackMainRunMode = BACK_MAIN_MODE_AUTO ;
            nBackSubRunMode = BACK_SUB_MODE_AUTO_1 ;
            //140530
            nCurBackMainRunModeStore =  BACK_MAIN_MODE_AUTO ;
            nCurBackSubRunModeStore = BACK_SUB_MODE_AUTO_1 ;
            
            bBackAutoModeInit = TRUE ;
            bReconfigFlag = TRUE ;
          }
          //设置气囊功能
          // if(nKeyAirBagLocate != AIRBAG_LOCATE_AUTO)
          {
            nKeyAirBagLocate = AIRBAG_LOCATE_AUTO ;
            //140531
            nKeyAirBagLocateStore = AIRBAG_LOCATE_AUTO ;
            
            if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
            {
              Valve_SetAirBagStrength(AIRBAG_STRENGTH_3);
            }
            /*****************Fungares**********************/
            st_AirBagAuto0.init = TRUE ;
            
            bReconfigFlag = TRUE ;
          }
          //只有在需要重新配置时才起作用
          if(bReconfigFlag == TRUE)
          {
            stretchMode = STRETCH_GO_DOWN;
            Data_Set_Start(1, w_PresetTime);
            nBuzzerMode = BUZZER_MODE_ONETIME ;
            bSendBuzzerMode = TRUE ;
            bBlueToothSendBuzzerMode = TRUE;
          }
          break ;
        case H10_KEY_CHAIR_AUTO_2:
#ifdef SN_CHECK_EN
          if(bKeyPowerSwitch == FALSE)
          {
            SN_CHECK_KEY_INPUT[0] = SN_CHECK_KEY_INPUT[1] ;
            SN_CHECK_KEY_INPUT[1] = SN_CHECK_KEY_INPUT[2] ;
            SN_CHECK_KEY_INPUT[2] = SN_CHECK_KEY_INPUT[3] ;
            SN_CHECK_KEY_INPUT[3] = SN_CHECK_KEY_INPUT[4] ;
            SN_CHECK_KEY_INPUT[4] = H10_KEY_CHAIR_AUTO_2 ;
            if(SNCheckKeyCompare() == TRUE)
            {
              nSendSNFlag = TRUE ;
              nSendSNStep = 0 ;
            }
          }
#endif
          if(bKeyPowerSwitch == FALSE) break ;

           if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
          {
            nTargetMassagePosition = MASSAGE_OPTIMAL_POSITION;
            bMassagePositionUpdate = TRUE;
          }                                                         //fww
          //nTargetMassagePosition = MASSAGE_RESET_POSITION;        //fww
          bKneadMotorInProcess = FALSE ;
          
#ifdef FOOT_ROLLER_ENABLE
          bRollerEnable = TRUE;
          bRollerEnableStore = TRUE;
          if(nRollerPWM == 0)
          {
            nRollerPWMStore = 2;
            nRollerPWM = 2;
            Valve_SetRollerPWM(nRollerPWM);
          }
#endif
          ////////////////////////////
#ifdef POWER_VIBRATION
          bKeySeatVibrate = TRUE ;
          //bGetNextVibStep = TRUE ;
          //nCurVibStep = 0 ;
          //nCurAutoVibItem = 0 ;
          nKeySeatVibrateStrength = 2 ;
#endif
          ////////////////////////////
          bReconfigFlag = FALSE ;
          nChairRunState = CHAIR_STATE_RUN ;
          nIndicateTimer = RUN_INDICATE_TIME;

          /////////////////////
          //设置背部功能
          if(!((nBackMainRunMode == BACK_MAIN_MODE_AUTO) &&
               (nBackSubRunMode == BACK_SUB_MODE_AUTO_2)))
          {
            nBackMainRunMode = BACK_MAIN_MODE_AUTO ;
            nBackSubRunMode = BACK_SUB_MODE_AUTO_2 ;
            //140530
            nCurBackMainRunModeStore =  BACK_MAIN_MODE_AUTO ;
            nCurBackSubRunModeStore = BACK_SUB_MODE_AUTO_2 ;
            
            bBackAutoModeInit = TRUE ;
            bReconfigFlag = TRUE ;
          }
          //设置气囊功能
          //  if(nKeyAirBagLocate != AIRBAG_LOCATE_AUTO)
          {
            nKeyAirBagLocate = AIRBAG_LOCATE_AUTO ;
            //140531
            nKeyAirBagLocateStore = AIRBAG_LOCATE_AUTO ;
            
            if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
            {
              Valve_SetAirBagStrength(AIRBAG_STRENGTH_3);
            }
           /*****************Fungares**********************/
            st_AirBagAuto0.init = TRUE ;
            
            bReconfigFlag = TRUE ;
          }
          //只有在需要重新配置时才起作用
          if(bReconfigFlag == TRUE)
          {
            if(KR_PROGRAM)
              Data_Set_Start(1, 30 * 60);
            else
              Data_Set_Start(1, w_PresetTime);
            
            nBuzzerMode = BUZZER_MODE_ONETIME ;
            bSendBuzzerMode = TRUE ;
            bBlueToothSendBuzzerMode = TRUE;
          }
          break ;
        case H10_KEY_CHAIR_AUTO_3:
#ifdef SN_CHECK_EN
          if(bKeyPowerSwitch == FALSE)
          {
            SN_CHECK_KEY_INPUT[0] = SN_CHECK_KEY_INPUT[1] ;
            SN_CHECK_KEY_INPUT[1] = SN_CHECK_KEY_INPUT[2] ;
            SN_CHECK_KEY_INPUT[2] = SN_CHECK_KEY_INPUT[3] ;
            SN_CHECK_KEY_INPUT[3] = SN_CHECK_KEY_INPUT[4] ;
            SN_CHECK_KEY_INPUT[4] = H10_KEY_CHAIR_AUTO_3 ;
            if(SNCheckKeyCompare() == TRUE)
            {
              nSendSNFlag = TRUE ;
              nSendSNStep = 0 ;
            }
          }
#endif
          if(bKeyPowerSwitch == FALSE) break ;
          
          
          st_Stretch.bBackLegFlag = FALSE;
          
          bKneadMotorInProcess = FALSE ;
          
          
          if(nTargetMassagePosition == MASSAGE_RESET_POSITION)
          {
            nTargetMassagePosition = MASSAGE_OPTIMAL_POSITION;
            bMassagePositionUpdate = TRUE;
          }
          
          
#ifdef FOOT_ROLLER_ENABLE
          bRollerEnable = TRUE;
          bRollerEnableStore = TRUE;
          if(nRollerPWM == 0)
          {
            nRollerPWMStore = 2;
            nRollerPWM = 2;
            Valve_SetRollerPWM(nRollerPWM);
          }
#endif
          ////////////////////////////
#ifdef POWER_VIBRATION
          bKeySeatVibrate = TRUE ;
          //bGetNextVibStep = TRUE ;
          //nCurVibStep = 0 ;
          //nCurAutoVibItem = 0 ;
          nKeySeatVibrateStrength = 2 ;
#endif
          ////////////////////////////
          bReconfigFlag = FALSE ;
          nChairRunState = CHAIR_STATE_RUN ;
          nIndicateTimer = RUN_INDICATE_TIME;

          /////////////////////
          //设置背部功能
          if(!((nBackMainRunMode == BACK_MAIN_MODE_AUTO) &&
               (nBackSubRunMode == BACK_SUB_MODE_AUTO_3)))
          {
            nBackMainRunMode = BACK_MAIN_MODE_AUTO ;
            nBackSubRunMode = BACK_SUB_MODE_AUTO_3 ;
            //140530
            nCurBackMainRunModeStore =  BACK_MAIN_MODE_AUTO ;
            nCurBackSubRunModeStore = BACK_SUB_MODE_AUTO_3 ;
            
            bBackAutoModeInit = TRUE ;
            bReconfigFlag = TRUE ;
          }
          //设置气囊功能
          //if(nKeyAirBagLocate != AIRBAG_LOCATE_AUTO)  强制执行气囊切换程序
          {
            nKeyAirBagLocate = AIRBAG_LOCATE_AUTO ;
            //140531
            nKeyAirBagLocateStore = AIRBAG_LOCATE_AUTO ;
            
            if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
            {
              Valve_SetAirBagStrength(AIRBAG_STRENGTH_3);
            }
            
            /*****************Fungares**********************/
            st_AirBagAuto0.init = TRUE ;
            
            bReconfigFlag = TRUE ;
          }
          //只有在需要重新配置时才起作用
          if(bReconfigFlag == TRUE)
          {
            //设置时间
            Data_Set_Start(1, w_PresetTime);
            nBuzzerMode = BUZZER_MODE_ONETIME ;
            bSendBuzzerMode = TRUE ;
            bBlueToothSendBuzzerMode = TRUE;
          }
          
          
          break;
          
#ifndef RT8301_CONTROL
        case H10_KEY_ZERO_START:
          if(bKeyPowerSwitch == FALSE) break ;
          if(KR_PROGRAM)
          {
            if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2)) break;
          }
          //140906
         // RockFunctionEnable(false);
          if(isZeroPosition())
          {
            nTargetMassagePosition = MASSAGE_INIT_POSITION;
          }
          else
          {
            nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
          }
          bMassagePositionUpdate = TRUE;
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          break;
          case H10_KEY_WORK_TIME_10MIN:
          case H10_KEY_WORK_TIME_20MIN:
          case H10_KEY_WORK_TIME_30MIN:
            {
              switch(nCommandID)
              { 
              case H10_KEY_WORK_TIME_10MIN:
                if(KR_PROGRAM)
                {
                  if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2)) break;
                }
                w_PresetTime = RUN_TIME_10;
                Data_Update_Time(w_PresetTime);
                nBuzzerMode = BUZZER_MODE_ONETIME ;
                bSendBuzzerMode = TRUE ;
                bBlueToothSendBuzzerMode = TRUE;
                break;
              case H10_KEY_WORK_TIME_20MIN:
                if(KR_PROGRAM)
                {
                  if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2)) break;
                }
                w_PresetTime = RUN_TIME_20;
                Data_Update_Time(w_PresetTime);
                nBuzzerMode = BUZZER_MODE_ONETIME ;
                bSendBuzzerMode = TRUE ;
                bBlueToothSendBuzzerMode = TRUE;
                break;
              case H10_KEY_WORK_TIME_30MIN:
                if(KR_PROGRAM)
                {
                  if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2)) break;
                }
                w_PresetTime = RUN_TIME_30;
                Data_Update_Time(w_PresetTime);
                nBuzzerMode = BUZZER_MODE_ONETIME ;
                bSendBuzzerMode = TRUE ;
                bBlueToothSendBuzzerMode = TRUE;
                break;
              }
              w_PresetTime_Min = (unsigned char)(w_PresetTime/60);      //fww
              memset(pw_Information, 0, sizeof(pw_Information));        //fww
              PBYTE pInformation = (PBYTE)pw_Information;               //fww
              MEM_Read_Memory(pw_Information, MEMORY_LENGTH_OF_BYTES);   //FWW
              *(pInformation + TIME_SWITCH) = w_PresetTime_Min;           //fww
              MEM_Write_Memory(pw_Information, MEMORY_LENGTH_OF_BYTES);  //fww
              break;
            }
#endif
          
        case H10_KEY_CHAIR_AUTO_4:
          if(bKeyPowerSwitch == FALSE) break ;
          //  st_Stretch.bBackLegFlag = FALSE;
          bKneadMotorInProcess = FALSE ;

          bReconfigFlag = FALSE ;
          nChairRunState = CHAIR_STATE_RUN ;
          nIndicateTimer = RUN_INDICATE_TIME;
          /////////////////////
          //设置背部功能
          if(!((nBackMainRunMode == BACK_MAIN_MODE_AUTO) &&
               (nBackSubRunMode == BACK_SUB_MODE_AUTO_4)))
          {
            nBackMainRunMode = BACK_MAIN_MODE_AUTO ;
            nBackSubRunMode = BACK_SUB_MODE_AUTO_4 ;
            //140605
            nCurBackMainRunModeStore = BACK_MAIN_MODE_AUTO ;
            nCurBackSubRunModeStore = BACK_SUB_MODE_AUTO_4 ;
            
            bBackAutoModeInit = TRUE ;
            bReconfigFlag = TRUE ;
          }
          
#ifdef FOOT_ROLLER_ENABLE
          bRollerEnable = TRUE;
          bRollerEnableStore = TRUE;
          if(nRollerPWM == 0)
          {
            nRollerPWMStore = 2;
            nRollerPWM = 2;
            Valve_SetRollerPWM(nRollerPWM);
          }
#endif
          
          //设置气囊功能
          {
            nKeyAirBagLocate = AIRBAG_LOCATE_AUTO ;
            //140531
            nKeyAirBagLocateStore = AIRBAG_LOCATE_AUTO ;
            
            if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
            {
              Valve_SetAirBagStrength(AIRBAG_STRENGTH_3);
            }
            
            /*****************Fungares**********************/
            st_AirBagAuto0.init = TRUE ;
            
            bReconfigFlag = TRUE ;
          }
          //只有在需要重新配置时才起作用
          if(bReconfigFlag)
          {
            //设置时间
            Data_Set_Start(1, w_PresetTime);
            nBuzzerMode = BUZZER_MODE_ONETIME ;
            bSendBuzzerMode = TRUE ;
            bBlueToothSendBuzzerMode = TRUE;
          }
          break ;
        case H10_KEY_CHAIR_AUTO_5:
          if(bKeyPowerSwitch == FALSE) break ;
          //  st_Stretch.bBackLegFlag = FALSE;
          bKneadMotorInProcess = FALSE ;
          /*
          if(zeroGravityEnabled)
          {
          nZLB_RunState = 1;
          bZLBMotorRunFlag = TRUE;
        }
          */
#ifdef FOOT_ROLLER_ENABLE
          bRollerEnable = TRUE;
          bRollerEnableStore = TRUE;
          if(nRollerPWM == 0)
          {
            nRollerPWMStore = 2;
            nRollerPWM = 2;
            Valve_SetRollerPWM(nRollerPWM);
          }
#endif
          ////////////////////////////
#ifdef POWER_VIBRATION
          bKeySeatVibrate = TRUE ;
          nKeySeatVibrateStrength = 2 ;
#endif
          ////////////////////////////
          bReconfigFlag = FALSE ;
          nChairRunState = CHAIR_STATE_RUN ;
          nIndicateTimer = RUN_INDICATE_TIME;
          /////////////////////
          //设置背部功能
          if(!((nBackMainRunMode == BACK_MAIN_MODE_AUTO) &&
               (nBackSubRunMode == BACK_SUB_MODE_AUTO_5)))
          {
            nBackMainRunMode = BACK_MAIN_MODE_AUTO ;
            nBackSubRunMode = BACK_SUB_MODE_AUTO_5 ;
            //140605
            nCurBackMainRunModeStore = BACK_MAIN_MODE_AUTO ;
            nCurBackSubRunModeStore = BACK_SUB_MODE_AUTO_5 ;
            
            bBackAutoModeInit = TRUE ;
            bReconfigFlag = TRUE ;
          }
          //设置气囊功能
          
          {
            nKeyAirBagLocate = AIRBAG_LOCATE_AUTO ;
            //140531
            nKeyAirBagLocateStore = AIRBAG_LOCATE_AUTO ;
            
            if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
            {
              Valve_SetAirBagStrength(AIRBAG_STRENGTH_3);
            }
            
            /*****************Fungares**********************/
            st_AirBagAuto0.init = TRUE ;
            
            bReconfigFlag = TRUE ;
          }
          
          //只有在需要重新配置时才起作用
          if(bReconfigFlag)
          {
            //设置时间
            Data_Set_Start(1, w_PresetTime);
            //   Data_Set_Time(runTime);
            nBuzzerMode = BUZZER_MODE_ONETIME ;
            bSendBuzzerMode = TRUE ;
            bBlueToothSendBuzzerMode = TRUE;
          }
          break ;
          

        case H10_KEY_AIRBAG_AUTO:
          if(bKeyPowerSwitch == FALSE) break ;
          if(KR_PROGRAM)
          {
            if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2)) break;
          }
          if(nChairRunState == CHAIR_STATE_WAIT_COMMAND)
          {
            nChairRunState = CHAIR_STATE_RUN ;
            nIndicateTimer = RUN_INDICATE_TIME;
            nKeyAirBagLocate = AIRBAG_LOCATE_AUTO ;
            //140531
            nKeyAirBagLocateStore = AIRBAG_LOCATE_AUTO ;
             /*****************Fungares**********************/
            st_AirBagAuto0.init = TRUE ;
                     
            bRollerEnable = TRUE;
            bRollerEnableStore = TRUE;
            if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
            {
              Valve_SetAirBagStrength(AIRBAG_STRENGTH_3);
            }
            Data_Set_Start(1, w_PresetTime);
          }
          else
          {
            if(nKeyAirBagLocate != AIRBAG_LOCATE_AUTO)
            {
              nKeyAirBagLocate = AIRBAG_LOCATE_AUTO ;
              //140531
              nKeyAirBagLocateStore = AIRBAG_LOCATE_AUTO ;
              
              if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
              {
                Valve_SetAirBagStrength(AIRBAG_STRENGTH_3);
              }
            /*****************Fungares**********************/
            st_AirBagAuto0.init = TRUE ;
            
              bRollerEnable = TRUE;
              bRollerEnableStore = TRUE;
            }
            else
            {
              nKeyAirBagLocate = AIRBAG_LOCATE_NONE ;
              //140531
              nKeyAirBagLocateStore = AIRBAG_LOCATE_NONE ;
              nRollerPWM = 0;
              nRollerPWMStore = 0;
              bRollerEnable = FALSE;
              bRollerEnableStore = FALSE;
              Valve_SetRollerPWM(nRollerPWM);
            }
          }
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          break ;
          
        case H10_KEY_AIRBAG_STRENGTH_1:
          if(KR_PROGRAM)
          {
            if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2)) break;
          }
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          Valve_SetAirBagStrength(AIRBAG_STRENGTH_1);
          break;
        case H10_KEY_AIRBAG_STRENGTH_2:
          if(KR_PROGRAM)
          {
            if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2)) break;
          }
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          Valve_SetAirBagStrength(AIRBAG_STRENGTH_2);
          break;
        case H10_KEY_AIRBAG_STRENGTH_3:
          if(KR_PROGRAM)
          {
            if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2)) break;
          }
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          Valve_SetAirBagStrength(AIRBAG_STRENGTH_3);
          break;
        case H10_KEY_AIRBAG_STRENGTH_4:
          if(KR_PROGRAM)
          {
            if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2)) break;
          }
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          Valve_SetAirBagStrength(AIRBAG_STRENGTH_4);
          break;
        case H10_KEY_AIRBAG_STRENGTH_5:
          if(KR_PROGRAM)
          {
            if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2)) break;
          }
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          Valve_SetAirBagStrength(AIRBAG_STRENGTH_5);
          break;
        case H10_KEY_AIRBAG_STRENGTH_OFF:
          if(KR_PROGRAM)
          {
            if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2)) break;
          }
          if(nKeyAirBagLocate == AIRBAG_LOCATE_AUTO)
          {
            nRollerPWM = 0;
            nRollerPWMStore = 0;
            bRollerEnable = FALSE;
            bRollerEnableStore = FALSE;
            Valve_SetRollerPWM(nRollerPWM);
            Valve_SetAirBagStrength(AIRBAG_STRENGTH_0);
          }
          nKeyAirBagLocate = AIRBAG_LOCATE_NONE;
          //140531
          nKeyAirBagLocateStore = AIRBAG_LOCATE_NONE ;
	  
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          break;
          
	case H10_KEY_PRESS:
	case H10_KEY_RUBBING:
        case H10_KEY_SOFT_KNOCK:
        case H10_KEY_KNOCK:
        case H10_KEY_KNEAD:
        case H10_KEY_WAVELET:  
        case H10_KEY_MUSIC:
	case H10_KEY_MANUAL:
		if(bKeyPowerSwitch == FALSE) break ;
		// RockFunctionEnable(false);//140906
		if(KR_PROGRAM)
		{
		//	if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2)) break;
		}
		if(nBackMainRunMode != BACK_MAIN_MODE_MANUAL)
		{
			nBackMainRunMode = BACK_MAIN_MODE_MANUAL;
			//140530
			nCurBackMainRunModeStore =  BACK_MAIN_MODE_MANUAL ;

			nMaunalSubMode = 1;
			//bRollerEnable = 0;
			//bKeySeatVibrate = FALSE ;
		}
		else
		{
			nMaunalSubMode++;
			if(nMaunalSubMode > 7)
			{
				nMaunalSubMode = 1;
			}
		}
          nChairRunState = CHAIR_STATE_RUN ;
          nIndicateTimer = RUN_INDICATE_TIME;
          if(nCommandID == H10_KEY_MANUAL)
          {
            //设置背部功能
            BackManualModeNoAction() ;
          }
          
          //  bDemoFlag = FALSE;
          
          //设置气囊功能
          //设置运行时间
          if(Data_Get_Time() == 0)
          {
            Data_Set_Start(1, w_PresetTime);
            // Data_Set_Start(1);
          }
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          /*
          switch(nCommandID)
          {
          case H10_KEY_KNEAD:
            nMaunalSubMode = nMaunalSubMode_KNEAD;
            break;
          case H10_KEY_KNOCK:
            nMaunalSubMode = nMaunalSubMode_KNOCK;
            break;
          case H10_KEY_WAVELET:
            nMaunalSubMode = nMaunalSubMode_WAVELET;
            break;
          case H10_KEY_SOFT_KNOCK:
            nMaunalSubMode = nMaunalSubMode_SOFT_KNOCK;
            break;
          case H10_KEY_PRESS:
            nMaunalSubMode = nMaunalSubMode_PRESS;
            break;
          case H10_KEY_MUSIC:
            nMaunalSubMode = nMaunalSubMode_MUSIC;
            break;
          case H10_KEY_MANUAL:
            nMaunalSubMode++;
	    if(nMaunalSubMode > 7)
               nMaunalSubMode = 1;
            break;
          }
	  */
          //140530
          nCurMaunalSubModeStore = nMaunalSubMode ;
          switch(nMaunalSubMode)		// == nMaunalSubMode_KNEAD)
          {
          case nMaunalSubMode_KNEAD:
            /*
            if(nBackSubRunMode == BACK_SUB_MODE_MUSIC)
            {
            bKeySeatVibrate = FALSE ;
            //nKeySeatVibrateStrength = 0 ;
          }
            */
            
            if(nBackSubRunMode == BACK_SUB_MODE_KNEAD)
            {
              //设置背部功能
              BackManualModeNoAction() ;
              break ;
            }
            nBackSubRunMode = BACK_SUB_MODE_KNEAD ;
            //根据当前的状态设置nKeyKneadKnockSpeed,nKeyBackLocate和nKeyKneadWidth
            if(nKeyKneadKnockSpeed == SPEED_0)
            {
              nKeyKneadKnockSpeed = SPEED_2 ;
            }
            if(nKeyBackLocate == LOCATE_NONE)
            {
              nKeyBackLocate = LOCATE_FULL_BACK ;
              /*
              if((nCurWalkMotorState == STATE_IDLE) ||
              (nCurWalkMotorState == STATE_RUN_CLOCK) ||
              (nCurWalkMotorState == STATE_STOP_CLOCK_ZV) ||
              (nCurWalkMotorState == STATE_STOP_CLOCK_HV))
              */
              {
                ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
                ManualDirector[0].nWalkMotorLocateParam = 0 ;
                ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
                ManualDirector[1].nWalkMotorLocateParam = 0 ;
              }
              /*
                            else
              {
              ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
              ManualDirector[0].nWalkMotorLocateParam = 0 ;
              ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
              ManualDirector[1].nWalkMotorLocateParam = 0 ;
            }
              */
            }
            if(nKeyKneadWidth == KNEAD_WIDTH_UNKNOWN)
            {
              nKeyKneadWidth = KNEAD_WIDTH_MED ;
            }
            ManualDirector[0].nSubFunction = BACK_SUB_MODE_KNEAD ;
            ManualDirector[0].nKneadMotorState = KNEAD_RUN ;
            ManualDirector[0].nKneadMotorCycles = 0 ;
            ManualDirector[0].nKnockMotorState = KNOCK_STOP ;
            ManualDirector[0].nKnockMotorRunTime = 0 ;
            ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            ManualDirector[1].nSubFunction = BACK_SUB_MODE_KNEAD ;
            ManualDirector[1].nKneadMotorState = KNEAD_RUN ;
            ManualDirector[1].nKneadMotorCycles = 0 ;
            ManualDirector[1].nKnockMotorState = KNOCK_STOP ;
            ManualDirector[1].nKnockMotorRunTime = 0 ;
            ManualDirector[1].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            ManualDirector[2].nSubFunction = BACK_SUB_MODE_KNEAD ;
            ManualDirector[2].nKneadMotorState = KNEAD_RUN ;
            ManualDirector[2].nKneadMotorCycles = 0 ;
            ManualDirector[2].nKnockMotorState = KNOCK_STOP ;
            ManualDirector[2].nKnockMotorRunTime = 0 ;
            ManualDirector[2].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            ManualDirector[3].nSubFunction = BACK_SUB_MODE_KNEAD ;
            ManualDirector[3].nKneadMotorState = KNEAD_RUN ;
            ManualDirector[3].nKneadMotorCycles = 0 ;
            ManualDirector[3].nKnockMotorState = KNOCK_STOP ;
            ManualDirector[3].nKnockMotorRunTime = 0 ;
            ManualDirector[3].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            //设置揉捏电机
            bKneadMotorInProcess = TRUE ;
            nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
            nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
            //设置捶击电机
            bKnockMotorInProcess = TRUE ;
            nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
            nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
            nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
            
            nMaxActionStep = 2 ;
            nStartActionStep = 0 ;
            bBackManualModeInit = TRUE ;
            break;
            
          case nMaunalSubMode_KNOCK:
            /*
            if(nBackSubRunMode == BACK_SUB_MODE_MUSIC)
            {
            bKeySeatVibrate = FALSE ;
            //nKeySeatVibrateStrength = 0 ;
          }
            */
            if(nBackSubRunMode == BACK_SUB_MODE_KNOCK)
            {
              //设置背部功能
              BackManualModeNoAction() ;
              break ;
            }
            nBackSubRunMode = BACK_SUB_MODE_KNOCK ;
            //根据当前的状态设置nKeyKneadKnockSpeed,nKeyBackLocate
            if(nKeyKneadKnockSpeed == SPEED_0)
            {
              nKeyKneadKnockSpeed = SPEED_2 ;
            }
            if(nKeyBackLocate == LOCATE_NONE)
            {
              nKeyBackLocate = LOCATE_FULL_BACK ;
              /*
              if((nCurWalkMotorState == STATE_IDLE) ||
              (nCurWalkMotorState == STATE_RUN_CLOCK) ||
              (nCurWalkMotorState == STATE_STOP_CLOCK_ZV) ||
              (nCurWalkMotorState == STATE_STOP_CLOCK_HV))
              */
              {
                ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
                ManualDirector[0].nWalkMotorLocateParam = 0 ;
                ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
                ManualDirector[1].nWalkMotorLocateParam = 0 ;
              }
              /*
                            else
              {
              ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
              ManualDirector[0].nWalkMotorLocateParam = 0 ;
              ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
              ManualDirector[1].nWalkMotorLocateParam = 0 ;
            }
              */
            }
            if(nKeyKneadWidth == KNEAD_WIDTH_UNKNOWN)
            {
              nKeyKneadWidth = KNEAD_WIDTH_MED ;
            }
            switch(nKeyKneadWidth)
            {
            case KNEAD_WIDTH_MIN:
              ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MIN ;
              ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MIN ;
              ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MIN ;
              ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MIN ;
              break ;
            case KNEAD_WIDTH_MED:
              ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MED ;
              ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MED ;
              ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MED ;
              ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MED ;
              break ;
            case KNEAD_WIDTH_MAX:
              ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MAX ;
              ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MAX ;
              ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MAX ;
              ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MAX ;
              break ;
            }
            ManualDirector[0].nSubFunction = BACK_SUB_MODE_KNOCK ;
            ManualDirector[0].nKneadMotorCycles = 0 ;
            ManualDirector[0].nKnockMotorState = KNOCK_RUN_WIDTH ;
            ManualDirector[0].nKnockMotorRunTime = 0 ;
            ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            ManualDirector[1].nSubFunction = BACK_SUB_MODE_KNOCK ;
            ManualDirector[1].nKneadMotorCycles = 0 ;
            ManualDirector[1].nKnockMotorState = KNOCK_RUN_WIDTH ;
            ManualDirector[1].nKnockMotorRunTime = 0 ;
            ManualDirector[1].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            ManualDirector[2].nSubFunction = BACK_SUB_MODE_KNOCK ;
            ManualDirector[2].nKneadMotorCycles = 0 ;
            ManualDirector[2].nKnockMotorState = KNOCK_RUN_WIDTH ;
            ManualDirector[2].nKnockMotorRunTime = 0 ;
            ManualDirector[2].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            ManualDirector[3].nSubFunction = BACK_SUB_MODE_KNOCK ;
            ManualDirector[3].nKneadMotorCycles = 0 ;
            ManualDirector[3].nKnockMotorState = KNOCK_RUN_WIDTH ;
            ManualDirector[3].nKnockMotorRunTime = 0 ;
            ManualDirector[3].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            //设置揉捏电机
            bKneadMotorInProcess = TRUE ;
            nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
            nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
            //设置捶击电机
            bKnockMotorInProcess = TRUE ;
            nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
            nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
            nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
            
            nMaxActionStep = 2 ;
            nStartActionStep = 0 ;
            bBackManualModeInit = TRUE ;
            break;
            
          case nMaunalSubMode_WAVELET:
            /*
            if(nBackSubRunMode == BACK_SUB_MODE_MUSIC)
            {
            bKeySeatVibrate = FALSE ;
            //nKeySeatVibrateStrength = 0 ;
          }
            */
            if(nBackSubRunMode == BACK_SUB_MODE_WAVELET)
            {
              //设置背部功能
              BackManualModeNoAction() ;
              break ;
            }
            nBackSubRunMode = BACK_SUB_MODE_WAVELET ;
            //根据当前的状态设置nKeyKneadKnockSpeed,nKeyBackLocate和nKeyKneadWidth
            if(nKeyKneadKnockSpeed == SPEED_0)
            {
              nKeyKneadKnockSpeed = SPEED_2 ;
            }
            if(nKeyBackLocate == LOCATE_NONE)
            {
              nKeyBackLocate = LOCATE_FULL_BACK ;
              /*
              if((nCurWalkMotorState == STATE_IDLE) ||
              (nCurWalkMotorState == STATE_RUN_CLOCK) ||
              (nCurWalkMotorState == STATE_STOP_CLOCK_ZV) ||
              (nCurWalkMotorState == STATE_STOP_CLOCK_HV))
              */
              {
                ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
                ManualDirector[0].nWalkMotorLocateParam = 0 ;
                ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
                ManualDirector[1].nWalkMotorLocateParam = 0 ;
              }
              /*
                            else
              {
              ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
              ManualDirector[0].nWalkMotorLocateParam = 0 ;
              ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
              ManualDirector[1].nWalkMotorLocateParam = 0 ;
            }
              */
            }
            if(nKeyKneadWidth == KNEAD_WIDTH_UNKNOWN)
            {
              nKeyKneadWidth = KNEAD_WIDTH_MED ;
            }
            ManualDirector[0].nSubFunction = BACK_SUB_MODE_WAVELET ;
            ManualDirector[0].nKneadMotorState = KNEAD_RUN ;
            ManualDirector[0].nKneadMotorCycles = 0 ;
            ManualDirector[0].nKnockMotorState = KNOCK_RUN ;
            ManualDirector[0].nKnockMotorRunTime = 0 ;
            ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            ManualDirector[1].nSubFunction = BACK_SUB_MODE_WAVELET ;
            ManualDirector[1].nKneadMotorState = KNEAD_RUN ;
            ManualDirector[1].nKneadMotorCycles = 0 ;
            ManualDirector[1].nKnockMotorState = KNOCK_RUN ;
            ManualDirector[1].nKnockMotorRunTime = 0 ;
            ManualDirector[1].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            ManualDirector[2].nSubFunction = BACK_SUB_MODE_WAVELET ;
            ManualDirector[2].nKneadMotorState = KNEAD_RUN ;
            ManualDirector[2].nKneadMotorCycles = 0 ;
            ManualDirector[2].nKnockMotorState = KNOCK_RUN ;
            ManualDirector[2].nKnockMotorRunTime = 0 ;
            ManualDirector[2].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            ManualDirector[3].nSubFunction = BACK_SUB_MODE_WAVELET ;
            ManualDirector[3].nKneadMotorState = KNEAD_RUN ;
            ManualDirector[3].nKneadMotorCycles = 0 ;
            ManualDirector[3].nKnockMotorState = KNOCK_RUN ;
            ManualDirector[3].nKnockMotorRunTime = 0 ;
            ManualDirector[3].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            //设置揉捏电机
            bKneadMotorInProcess = TRUE ;
            nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
            nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
            //设置捶击电机
            bKnockMotorInProcess = TRUE ;
            nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
            nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
            nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
            
            nMaxActionStep = 2 ;
            nStartActionStep = 0 ;
            bBackManualModeInit = TRUE ;
            break;
            
          case nMaunalSubMode_SOFT_KNOCK:
            /*
            if(nBackSubRunMode == BACK_SUB_MODE_MUSIC)
            {
            bKeySeatVibrate = FALSE ;
            //nKeySeatVibrateStrength = 0 ;
          }
            */
            if(nBackSubRunMode == BACK_SUB_MODE_SOFT_KNOCK)
            {
              //设置背部功能
              BackManualModeNoAction() ;
              break ;
            }
            nBackSubRunMode = BACK_SUB_MODE_SOFT_KNOCK ;
            //根据当前的状态设置nKeyKneadKnockSpeed,nKeyBackLocate
            if(nKeyKneadKnockSpeed == SPEED_0)
            {
              nKeyKneadKnockSpeed = SPEED_2 ;
            }
            if(nKeyBackLocate == LOCATE_NONE)
            {
              nKeyBackLocate = LOCATE_FULL_BACK ;
              /*
              if((nCurWalkMotorState == STATE_IDLE) ||
              (nCurWalkMotorState == STATE_RUN_CLOCK) ||
              (nCurWalkMotorState == STATE_STOP_CLOCK_ZV) ||
              (nCurWalkMotorState == STATE_STOP_CLOCK_HV))
              */
              {
                ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
                ManualDirector[0].nWalkMotorLocateParam = 0 ;
                ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
                ManualDirector[1].nWalkMotorLocateParam = 0 ;
              }
              /*
                            else
              {
              ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
              ManualDirector[0].nWalkMotorLocateParam = 0 ;
              ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
              ManualDirector[1].nWalkMotorLocateParam = 0 ;
            }
              */
            }
            if(nKeyKneadWidth == KNEAD_WIDTH_UNKNOWN)
            {
              nKeyKneadWidth = KNEAD_WIDTH_MED ;
            }
            switch(nKeyKneadWidth)
            {
            case KNEAD_WIDTH_MIN:
              ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MIN ;
              ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MIN ;
              ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MIN ;
              ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MIN ;
              break ;
            case KNEAD_WIDTH_MED:
              ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MED ;
              ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MED ;
              ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MED ;
              ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MED ;
              break ;
            case KNEAD_WIDTH_MAX:
              ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MAX ;
              ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MAX ;
              ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MAX ;
              ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MAX ;
              break ;
            }
            ManualDirector[0].nSubFunction = BACK_SUB_MODE_SOFT_KNOCK ;
            ManualDirector[0].nKneadMotorCycles = 0 ;
            ManualDirector[0].nKnockMotorState = KNOCK_RUN_STOP ;
            ManualDirector[0].nKnockMotorRunTime = 1 ;
            ManualDirector[0].nKnockMotorStopTime = 4 ;
            ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            
            ManualDirector[1].nSubFunction = BACK_SUB_MODE_SOFT_KNOCK ;
            ManualDirector[1].nKneadMotorCycles = 0 ;
            ManualDirector[1].nKnockMotorState = KNOCK_RUN_STOP ;
            ManualDirector[1].nKnockMotorRunTime = 1 ;
            ManualDirector[1].nKnockMotorStopTime = 4 ;
            ManualDirector[1].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            //设置揉捏电机(立即更新动作)
            bKneadMotorInProcess = TRUE ;
            nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
            nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
            //设置捶击电机(立即更新动作)
            bKnockMotorInProcess = TRUE ;
            nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
            nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
            nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
            
            nMaxActionStep = 2 ;
            nStartActionStep = 0 ;
            bBackManualModeInit = TRUE ;
            break;
            
          case nMaunalSubMode_RUBBING:
            if(nBackSubRunMode == BACK_SUB_MODE_RUBBING)
            {
              //设置背部功能
              BackManualModeNoAction() ;
              break ;
            }
            nBackSubRunMode = BACK_SUB_MODE_RUBBING ;
            //根据当前的状态设置nKeyKneadKnockSpeed,nKeyBackLocate
            if(nKeyKneadKnockSpeed == SPEED_0)
            {
              nKeyKneadKnockSpeed = SPEED_2 ;
            }
            if(nKeyBackLocate == LOCATE_NONE)
            {
              nKeyBackLocate = LOCATE_FULL_BACK ;
              ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
              ManualDirector[0].nWalkMotorLocateParam = 0 ;
              ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
              ManualDirector[1].nWalkMotorLocateParam = 0 ;
            }
            
            ManualDirector[0].nKneadMotorState = KNEAD_RUN_RUBBING ;
            ManualDirector[1].nKneadMotorState = KNEAD_RUN_RUBBING ;
            ManualDirector[2].nKneadMotorState = KNEAD_RUN_RUBBING ;
            ManualDirector[3].nKneadMotorState = KNEAD_RUN_RUBBING ;
            
            ManualDirector[0].nSubFunction = BACK_SUB_MODE_RUBBING ;
            ManualDirector[0].nKneadMotorCycles = 4 ;
            ManualDirector[0].nKnockMotorState = KNOCK_STOP ;
            ManualDirector[0].nKnockMotorRunTime = 0 ;
            ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            ManualDirector[1].nSubFunction = BACK_SUB_MODE_RUBBING ;
            ManualDirector[1].nKneadMotorCycles = 4 ;
            ManualDirector[1].nKnockMotorState = KNOCK_STOP ;
            ManualDirector[1].nKnockMotorRunTime = 0 ;
            ManualDirector[1].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            ManualDirector[2].nSubFunction = BACK_SUB_MODE_RUBBING ;
            ManualDirector[2].nKneadMotorCycles = 4 ;
            ManualDirector[2].nKnockMotorState = KNOCK_STOP ;
            ManualDirector[2].nKnockMotorRunTime = 0 ;
            ManualDirector[2].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            ManualDirector[3].nSubFunction = BACK_SUB_MODE_RUBBING ;
            ManualDirector[3].nKneadMotorCycles = 4 ;
            ManualDirector[3].nKnockMotorState = KNOCK_STOP ;
            ManualDirector[3].nKnockMotorRunTime = 0 ;
            ManualDirector[3].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            //设置揉捏电机
            bKneadMotorInProcess = TRUE ;
            nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
            nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
            //设置捶击电机
            bKnockMotorInProcess = TRUE ;
            nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
            nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
            nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
            
            nMaxActionStep = 2 ;
            nStartActionStep = 0 ;
            bBackManualModeInit = TRUE ;
            break;
	    
          case nMaunalSubMode_PRESS:
            if(nBackSubRunMode == BACK_SUB_MODE_PRESS)
            {
              //设置背部功能
              BackManualModeNoAction() ;
              break ;
            }
            nBackSubRunMode = BACK_SUB_MODE_PRESS ;
            //根据当前的状态设置nKeyKneadKnockSpeed,nKeyBackLocate
            if(nKeyKneadKnockSpeed == SPEED_0)
            {
              nKeyKneadKnockSpeed = SPEED_2 ;
            }
            if(nKeyBackLocate == LOCATE_NONE)
            {
              nKeyBackLocate = LOCATE_FULL_BACK ;
              ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
              ManualDirector[0].nWalkMotorLocateParam = 0 ;
              ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
              ManualDirector[1].nWalkMotorLocateParam = 0 ;
            }
            if(nKeyKneadWidth == KNEAD_WIDTH_UNKNOWN)
            {
              nKeyKneadWidth = KNEAD_WIDTH_MED ;
            }
            switch(nKeyKneadWidth)
            {
            case KNEAD_WIDTH_MIN:
              ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MIN ;
              ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MIN ;
              ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MIN ;
              ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MIN ;
              break ;
            case KNEAD_WIDTH_MED:
              ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MED ;
              ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MED ;
              ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MED ;
              ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MED ;
              break ;
            case KNEAD_WIDTH_MAX:
              ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MAX ;
              ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MAX ;
              ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MAX ;
              ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MAX ;
              break ;
            }
            ManualDirector[0].nSubFunction = BACK_SUB_MODE_PRESS ;
            ManualDirector[0].nKneadMotorCycles = 0 ;
            ManualDirector[0].nKnockMotorState = KNOCK_STOP ;
            ManualDirector[0].nKnockMotorRunTime = 0 ;
            ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            ManualDirector[1].nSubFunction = BACK_SUB_MODE_PRESS ;
            ManualDirector[1].nKneadMotorCycles = 0 ;
            ManualDirector[1].nKnockMotorState = KNOCK_STOP ;
            ManualDirector[1].nKnockMotorRunTime = 0 ;
            ManualDirector[1].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            //设置揉捏电机
            bKneadMotorInProcess = TRUE ;
            nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
            nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
            //设置捶击电机
            bKnockMotorInProcess = TRUE ;
            nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
            nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
            nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
            
            nMaxActionStep = 2 ;
            nStartActionStep = 0 ;
            bBackManualModeInit = TRUE ;
            break;
            
          case nMaunalSubMode_MUSIC:
            if(nBackSubRunMode == BACK_SUB_MODE_MUSIC)
            {
              //设置背部功能
              BackManualModeNoAction() ;
              break ;
            }
            nBackSubRunMode = BACK_SUB_MODE_MUSIC ;
            //根据当前的状态设置nKeyKneadKnockSpeed,nKeyBackLocate
            if(nKeyKneadKnockSpeed == SPEED_0)
            {
              nKeyKneadKnockSpeed = SPEED_2 ;
            }
            if(nKeyBackLocate == LOCATE_NONE)// || (nKeyBackLocate == LOCATE_POINT))
            {
              nKeyBackLocate = LOCATE_FULL_BACK ;
              /*
              if((nCurWalkMotorState == STATE_IDLE) ||
              (nCurWalkMotorState == STATE_RUN_CLOCK) ||
              (nCurWalkMotorState == STATE_STOP_CLOCK_ZV) ||
              (nCurWalkMotorState == STATE_STOP_CLOCK_HV))
              */
              {
                ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
                ManualDirector[0].nWalkMotorLocateParam = 0 ;
                ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
                ManualDirector[1].nWalkMotorLocateParam = 0 ;
                ManualDirector[2].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
                ManualDirector[2].nWalkMotorLocateParam = 0 ;
                ManualDirector[3].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
                ManualDirector[3].nWalkMotorLocateParam = 0 ;
              }
              /*
                            else
              {
              ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
              ManualDirector[0].nWalkMotorLocateParam = 0 ;
              ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
              ManualDirector[1].nWalkMotorLocateParam = 0 ;
              ManualDirector[2].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
              ManualDirector[2].nWalkMotorLocateParam = 0 ;
              ManualDirector[3].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
              ManualDirector[3].nWalkMotorLocateParam = 0 ;
            }
              */
            }
            if(nKeyKneadWidth == KNEAD_WIDTH_UNKNOWN)
            {
              nKeyKneadWidth = KNEAD_WIDTH_MED ;
            }
            ManualDirector[0].nSubFunction = BACK_SUB_MODE_MUSIC ;
            ManualDirector[0].nKneadMotorState = KNEAD_RUN ;
            ManualDirector[0].nKneadMotorCycles = 0 ;
            ManualDirector[0].nKnockMotorState = KNOCK_RUN_MUSIC ;
            ManualDirector[0].nKnockMotorRunTime = 0 ;
            ManualDirector[0].nKnockMotorStopTime = 0 ;
            ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            
            ManualDirector[1].nSubFunction = BACK_SUB_MODE_MUSIC ;
            ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MIN ;
            ManualDirector[1].nKneadMotorCycles = 0 ;
            ManualDirector[1].nKnockMotorState = KNOCK_RUN_MUSIC ;
            ManualDirector[1].nKnockMotorRunTime = 0 ;
            ManualDirector[1].nKnockMotorStopTime = 0 ;
            ManualDirector[1].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            
            ManualDirector[2].nSubFunction = BACK_SUB_MODE_MUSIC ;
            ManualDirector[2].nKneadMotorState = KNEAD_RUN ;
            ManualDirector[2].nKneadMotorCycles = 0 ;
            ManualDirector[2].nKnockMotorState = KNOCK_RUN_MUSIC ;
            ManualDirector[2].nKnockMotorRunTime = 0 ;
            ManualDirector[2].nKnockMotorStopTime = 0 ;
            ManualDirector[2].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            
            ManualDirector[3].nSubFunction = BACK_SUB_MODE_MUSIC ;
            ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MIN ;//fww
            ManualDirector[3].nKneadMotorCycles = 0 ;
            ManualDirector[3].nKnockMotorState = KNOCK_RUN_MUSIC ;
            ManualDirector[3].nKnockMotorRunTime = 0 ;
            ManualDirector[3].nKnockMotorStopTime = 0 ;
            ManualDirector[3].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
            
            //设置揉捏电机(立即更新动作)
            bKneadMotorInProcess = TRUE ;
            nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
            nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
            //设置捶击电机(立即更新动作)
            bKnockMotorInProcess = TRUE ;
            nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
            nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
            nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
            
            nMaxActionStep = 4 ;
            nStartActionStep = 0 ;
            bBackManualModeInit = TRUE ;
            //开启振动
            // bKeySeatVibrate = TRUE ;
            break;
          default:
            //设置背部功能
            BackManualModeNoAction() ;
            break ;
          }
          
          nKeyBackLocateStore = nKeyBackLocate ;
          break ;
          /******************************************************/
          
          
          /*
        case H10_KEY_MANUAL:
          if(bKeyPowerSwitch == FALSE) break ;
          if(nBackMainRunMode != BACK_MAIN_MODE_MANUAL)
          {
          nBackMainRunMode = BACK_MAIN_MODE_MANUAL;
          nMaunalSubMode = 5;
          //bRollerEnable = 0;
          //bKeySeatVibrate = FALSE ;
        }
          nChairRunState = CHAIR_STATE_RUN ;
          nIndicateTimer = RUN_INDICATE_TIME;
          //设置背部功能
          BackManualModeNoAction() ;
          //设置气囊功能
          //设置运行时间
          if(Data_Get_Time() == 0)
          {
          Data_Set_Start(1,w_PresetTime);
        }
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          
          nMaunalSubMode++;
          nMaunalSubMode %= 6;
          switch(nMaunalSubMode)		// == nMaunalSubMode_KNEAD)
          {
        case nMaunalSubMode_KNEAD:
          nBackSubRunMode = BACK_SUB_MODE_KNEAD ;
          //根据当前的状态设置nKeyKneadKnockSpeed,nKeyBackLocate和nKeyKneadWidth
          if(nKeyKneadKnockSpeed == SPEED_0)
          {
          nKeyKneadKnockSpeed = SPEED_2 ;
        }
          if(nKeyBackLocate == LOCATE_NONE)
          {
          nKeyBackLocate = LOCATE_FULL_BACK ;
          if((nCurWalkMotorState == STATE_IDLE) ||
          (nCurWalkMotorState == STATE_RUN_CLOCK) ||
          (nCurWalkMotorState == STATE_STOP_CLOCK_ZV) ||
          (nCurWalkMotorState == STATE_STOP_CLOCK_HV))
          {
          ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
          ManualDirector[0].nWalkMotorLocateParam = 0 ;
          ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
          ManualDirector[1].nWalkMotorLocateParam = 0 ;
        }
          else
          {
          ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
          ManualDirector[0].nWalkMotorLocateParam = 0 ;
          ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
          ManualDirector[1].nWalkMotorLocateParam = 0 ;
        }
        }
          if(nKeyKneadWidth == KNEAD_WIDTH_UNKNOWN)
          {
          nKeyKneadWidth = KNEAD_WIDTH_MED ;
        }
          ManualDirector[0].nSubFunction = BACK_SUB_MODE_KNEAD ;
          ManualDirector[0].nKneadMotorState = KNEAD_RUN ;
          ManualDirector[0].nKneadMotorCycles = 0 ;
          ManualDirector[0].nKnockMotorState = KNOCK_STOP ;
          ManualDirector[0].nKnockMotorRunTime = 0 ;
          ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          ManualDirector[1].nSubFunction = BACK_SUB_MODE_KNEAD ;
          ManualDirector[1].nKneadMotorState = KNEAD_RUN ;
          ManualDirector[1].nKneadMotorCycles = 0 ;
          ManualDirector[1].nKnockMotorState = KNOCK_STOP ;
          ManualDirector[1].nKnockMotorRunTime = 0 ;
          ManualDirector[1].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          ManualDirector[2].nSubFunction = BACK_SUB_MODE_KNEAD ;
          ManualDirector[2].nKneadMotorState = KNEAD_RUN ;
          ManualDirector[2].nKneadMotorCycles = 0 ;
          ManualDirector[2].nKnockMotorState = KNOCK_STOP ;
          ManualDirector[2].nKnockMotorRunTime = 0 ;
          ManualDirector[2].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          ManualDirector[3].nSubFunction = BACK_SUB_MODE_KNEAD ;
          ManualDirector[3].nKneadMotorState = KNEAD_RUN ;
          ManualDirector[3].nKneadMotorCycles = 0 ;
          ManualDirector[3].nKnockMotorState = KNOCK_STOP ;
          ManualDirector[3].nKnockMotorRunTime = 0 ;
          ManualDirector[3].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          //设置揉捏电机
          bKneadMotorInProcess = TRUE ;
          nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
          nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
          //设置捶击电机
          bKnockMotorInProcess = TRUE ;
          nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
          nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
          nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
          
          nMaxActionStep = 2 ;
          nStartActionStep = 0 ;
          bBackManualModeInit = TRUE ;
          break;
        case nMaunalSubMode_KNOCK:
          
          nBackSubRunMode = BACK_SUB_MODE_KNOCK ;
          //根据当前的状态设置nKeyKneadKnockSpeed,nKeyBackLocate
          if(nKeyKneadKnockSpeed == SPEED_0)
          {
          nKeyKneadKnockSpeed = SPEED_2 ;
        }
          if(nKeyBackLocate == LOCATE_NONE)
          {
          nKeyBackLocate = LOCATE_FULL_BACK ;
          if((nCurWalkMotorState == STATE_IDLE) ||
          (nCurWalkMotorState == STATE_RUN_CLOCK) ||
          (nCurWalkMotorState == STATE_STOP_CLOCK_ZV) ||
          (nCurWalkMotorState == STATE_STOP_CLOCK_HV))
          {
          ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
          ManualDirector[0].nWalkMotorLocateParam = 0 ;
          ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
          ManualDirector[1].nWalkMotorLocateParam = 0 ;
        }
          else
          {
          ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
          ManualDirector[0].nWalkMotorLocateParam = 0 ;
          ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
          ManualDirector[1].nWalkMotorLocateParam = 0 ;
        }
        }
          if(nKeyKneadWidth == KNEAD_WIDTH_UNKNOWN)
          {
          nKeyKneadWidth = KNEAD_WIDTH_MED ;
        }
          switch(nKeyKneadWidth)
          {
        case KNEAD_WIDTH_MIN:
          ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MIN ;
          ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MIN ;
          ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MIN ;
          ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MIN ;
          break ;
        case KNEAD_WIDTH_MED:
          ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MED ;
          ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MED ;
          ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MED ;
          ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MED ;
          break ;
        case KNEAD_WIDTH_MAX:
          ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MAX ;
          ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MAX ;
          ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MAX ;
          ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MAX ;
          break ;
        }
          ManualDirector[0].nSubFunction = BACK_SUB_MODE_KNOCK ;
          ManualDirector[0].nKneadMotorCycles = 0 ;
          ManualDirector[0].nKnockMotorState = KNOCK_RUN_WIDTH ;
          ManualDirector[0].nKnockMotorRunTime = 0 ;
          ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          ManualDirector[1].nSubFunction = BACK_SUB_MODE_KNOCK ;
          ManualDirector[1].nKneadMotorCycles = 0 ;
          ManualDirector[1].nKnockMotorState = KNOCK_RUN_WIDTH ;
          ManualDirector[1].nKnockMotorRunTime = 0 ;
          ManualDirector[1].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          ManualDirector[2].nSubFunction = BACK_SUB_MODE_KNOCK ;
          ManualDirector[2].nKneadMotorCycles = 0 ;
          ManualDirector[2].nKnockMotorState = KNOCK_RUN_WIDTH ;
          ManualDirector[2].nKnockMotorRunTime = 0 ;
          ManualDirector[2].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          ManualDirector[3].nSubFunction = BACK_SUB_MODE_KNOCK ;
          ManualDirector[3].nKneadMotorCycles = 0 ;
          ManualDirector[3].nKnockMotorState = KNOCK_RUN_WIDTH ;
          ManualDirector[3].nKnockMotorRunTime = 0 ;
          ManualDirector[3].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          //设置揉捏电机
          bKneadMotorInProcess = TRUE ;
          nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
          nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
          //设置捶击电机
          bKnockMotorInProcess = TRUE ;
          nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
          nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
          nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
          
          nMaxActionStep = 2 ;
          nStartActionStep = 0 ;
          bBackManualModeInit = TRUE ;
          break;
          
        case nMaunalSubMode_WAVELET:
          nBackSubRunMode = BACK_SUB_MODE_WAVELET ;
          //根据当前的状态设置nKeyKneadKnockSpeed,nKeyBackLocate和nKeyKneadWidth
          if(nKeyKneadKnockSpeed == SPEED_0)
          {
          nKeyKneadKnockSpeed = SPEED_2 ;
        }
          if(nKeyBackLocate == LOCATE_NONE)
          {
          nKeyBackLocate = LOCATE_FULL_BACK ;
          if((nCurWalkMotorState == STATE_IDLE) ||
          (nCurWalkMotorState == STATE_RUN_CLOCK) ||
          (nCurWalkMotorState == STATE_STOP_CLOCK_ZV) ||
          (nCurWalkMotorState == STATE_STOP_CLOCK_HV))
          {
          ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
          ManualDirector[0].nWalkMotorLocateParam = 0 ;
          ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
          ManualDirector[1].nWalkMotorLocateParam = 0 ;
        }
          else
          {
          ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
          ManualDirector[0].nWalkMotorLocateParam = 0 ;
          ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
          ManualDirector[1].nWalkMotorLocateParam = 0 ;
        }
        }
          if(nKeyKneadWidth == KNEAD_WIDTH_UNKNOWN)
          {
          nKeyKneadWidth = KNEAD_WIDTH_MED ;
        }
          ManualDirector[0].nSubFunction = BACK_SUB_MODE_WAVELET ;
          ManualDirector[0].nKneadMotorState = KNEAD_RUN ;
          ManualDirector[0].nKneadMotorCycles = 0 ;
          ManualDirector[0].nKnockMotorState = KNOCK_RUN ;
          ManualDirector[0].nKnockMotorRunTime = 0 ;
          ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          ManualDirector[1].nSubFunction = BACK_SUB_MODE_WAVELET ;
          ManualDirector[1].nKneadMotorState = KNEAD_RUN ;
          ManualDirector[1].nKneadMotorCycles = 0 ;
          ManualDirector[1].nKnockMotorState = KNOCK_RUN ;
          ManualDirector[1].nKnockMotorRunTime = 0 ;
          ManualDirector[1].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          ManualDirector[2].nSubFunction = BACK_SUB_MODE_WAVELET ;
          ManualDirector[2].nKneadMotorState = KNEAD_RUN ;
          ManualDirector[2].nKneadMotorCycles = 0 ;
          ManualDirector[2].nKnockMotorState = KNOCK_RUN ;
          ManualDirector[2].nKnockMotorRunTime = 0 ;
          ManualDirector[2].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          ManualDirector[3].nSubFunction = BACK_SUB_MODE_WAVELET ;
          ManualDirector[3].nKneadMotorState = KNEAD_RUN ;
          ManualDirector[3].nKneadMotorCycles = 0 ;
          ManualDirector[3].nKnockMotorState = KNOCK_RUN ;
          ManualDirector[3].nKnockMotorRunTime = 0 ;
          ManualDirector[3].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          //设置揉捏电机
          bKneadMotorInProcess = TRUE ;
          nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
          nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
          //设置捶击电机
          bKnockMotorInProcess = TRUE ;
          nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
          nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
          nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
          
          nMaxActionStep = 2 ;
          nStartActionStep = 0 ;
          bBackManualModeInit = TRUE ;
          break;
          
        case nMaunalSubMode_SOFT_KNOCK:
          nBackSubRunMode = BACK_SUB_MODE_SOFT_KNOCK ;
          //根据当前的状态设置nKeyKneadKnockSpeed,nKeyBackLocate
          if(nKeyKneadKnockSpeed == SPEED_0)
          {
          nKeyKneadKnockSpeed = SPEED_2 ;
        }
          if(nKeyBackLocate == LOCATE_NONE)
          {
          nKeyBackLocate = LOCATE_FULL_BACK ;
          if((nCurWalkMotorState == STATE_IDLE) ||
          (nCurWalkMotorState == STATE_RUN_CLOCK) ||
          (nCurWalkMotorState == STATE_STOP_CLOCK_ZV) ||
          (nCurWalkMotorState == STATE_STOP_CLOCK_HV))
          {
          ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
          ManualDirector[0].nWalkMotorLocateParam = 0 ;
          ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
          ManualDirector[1].nWalkMotorLocateParam = 0 ;
        }
          else
          {
          ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
          ManualDirector[0].nWalkMotorLocateParam = 0 ;
          ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
          ManualDirector[1].nWalkMotorLocateParam = 0 ;
        }
        }
          if(nKeyKneadWidth == KNEAD_WIDTH_UNKNOWN)
          {
          nKeyKneadWidth = KNEAD_WIDTH_MED ;
        }
          switch(nKeyKneadWidth)
          {
        case KNEAD_WIDTH_MIN:
          ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MIN ;
          ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MIN ;
          ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MIN ;
          ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MIN ;
          break ;
        case KNEAD_WIDTH_MED:
          ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MED ;
          ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MED ;
          ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MED ;
          ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MED ;
          break ;
        case KNEAD_WIDTH_MAX:
          ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MAX ;
          ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MAX ;
          ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MAX ;
          ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MAX ;
          break ;
        }
          ManualDirector[0].nSubFunction = BACK_SUB_MODE_SOFT_KNOCK ;
          ManualDirector[0].nKneadMotorCycles = 0 ;
          ManualDirector[0].nKnockMotorState = KNOCK_RUN_STOP ;
          ManualDirector[0].nKnockMotorRunTime = 1 ;
          ManualDirector[0].nKnockMotorStopTime = 4 ;
          ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          
          ManualDirector[1].nSubFunction = BACK_SUB_MODE_SOFT_KNOCK ;
          ManualDirector[1].nKneadMotorCycles = 0 ;
          ManualDirector[1].nKnockMotorState = KNOCK_RUN_STOP ;
          ManualDirector[1].nKnockMotorRunTime = 1 ;
          ManualDirector[1].nKnockMotorStopTime = 4 ;
          ManualDirector[1].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          //设置揉捏电机(立即更新动作)
          bKneadMotorInProcess = TRUE ;
          nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
          nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
          //设置捶击电机(立即更新动作)
          bKnockMotorInProcess = TRUE ;
          nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
          nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
          nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
          
          nMaxActionStep = 2 ;
          nStartActionStep = 0 ;
          bBackManualModeInit = TRUE ;
          break;
          
        case nMaunalSubMode_PRESS:
          nBackSubRunMode = BACK_SUB_MODE_PRESS ;
          //根据当前的状态设置nKeyKneadKnockSpeed,nKeyBackLocate
          if(nKeyKneadKnockSpeed == SPEED_0)
          {
          nKeyKneadKnockSpeed = SPEED_2 ;
        }
          if(nKeyBackLocate == LOCATE_NONE)
          {
          nKeyBackLocate = LOCATE_FULL_BACK ;
          if((nCurWalkMotorState == STATE_IDLE) ||
          (nCurWalkMotorState == STATE_RUN_CLOCK) ||
          (nCurWalkMotorState == STATE_STOP_CLOCK_ZV) ||
          (nCurWalkMotorState == STATE_STOP_CLOCK_HV))
          {
          ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
          ManualDirector[0].nWalkMotorLocateParam = 0 ;
          ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
          ManualDirector[1].nWalkMotorLocateParam = 0 ;
        }
          else
          {
          ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
          ManualDirector[0].nWalkMotorLocateParam = 0 ;
          ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
          ManualDirector[1].nWalkMotorLocateParam = 0 ;
        }
        }
          if(nKeyKneadWidth == KNEAD_WIDTH_UNKNOWN)
          {
          nKeyKneadWidth = KNEAD_WIDTH_MED ;
        }
          switch(nKeyKneadWidth)
          {
        case KNEAD_WIDTH_MIN:
          ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MIN ;
          ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MIN ;
          ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MIN ;
          ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MIN ;
          break ;
        case KNEAD_WIDTH_MED:
          ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MED ;
          ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MED ;
          ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MED ;
          ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MED ;
          break ;
        case KNEAD_WIDTH_MAX:
          ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MAX ;
          ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MAX ;
          ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MAX ;
          ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MAX ;
          break ;
        }
          ManualDirector[0].nSubFunction = BACK_SUB_MODE_PRESS ;
          ManualDirector[0].nKneadMotorCycles = 0 ;
          ManualDirector[0].nKnockMotorState = KNOCK_STOP ;
          ManualDirector[0].nKnockMotorRunTime = 0 ;
          ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          ManualDirector[1].nSubFunction = BACK_SUB_MODE_PRESS ;
          ManualDirector[1].nKneadMotorCycles = 0 ;
          ManualDirector[1].nKnockMotorState = KNOCK_STOP ;
          ManualDirector[1].nKnockMotorRunTime = 0 ;
          ManualDirector[1].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          //设置揉捏电机
          bKneadMotorInProcess = TRUE ;
          nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
          nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
          //设置捶击电机
          bKnockMotorInProcess = TRUE ;
          nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
          nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
          nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
          
          nMaxActionStep = 2 ;
          nStartActionStep = 0 ;
          bBackManualModeInit = TRUE ;
          break;
          
        case nMaunalSubMode_MUSIC:
          nBackSubRunMode = BACK_SUB_MODE_MUSIC ;
          //根据当前的状态设置nKeyKneadKnockSpeed,nKeyBackLocate
          if(nKeyKneadKnockSpeed == SPEED_0)
          {
          nKeyKneadKnockSpeed = SPEED_2 ;
        }
          if((nKeyBackLocate == LOCATE_NONE) || (nKeyBackLocate == LOCATE_POINT))
          {
          nKeyBackLocate = LOCATE_FULL_BACK ;
          if((nCurWalkMotorState == STATE_IDLE) ||
          (nCurWalkMotorState == STATE_RUN_CLOCK) ||
          (nCurWalkMotorState == STATE_STOP_CLOCK_ZV) ||
          (nCurWalkMotorState == STATE_STOP_CLOCK_HV))
          {
          ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
          ManualDirector[0].nWalkMotorLocateParam = 0 ;
          ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
          ManualDirector[1].nWalkMotorLocateParam = 0 ;
          ManualDirector[2].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
          ManualDirector[2].nWalkMotorLocateParam = 0 ;
          ManualDirector[3].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
          ManualDirector[3].nWalkMotorLocateParam = 0 ;
        }
          else
          {
          ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
          ManualDirector[0].nWalkMotorLocateParam = 0 ;
          ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
          ManualDirector[1].nWalkMotorLocateParam = 0 ;
          ManualDirector[2].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
          ManualDirector[2].nWalkMotorLocateParam = 0 ;
          ManualDirector[3].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
          ManualDirector[3].nWalkMotorLocateParam = 0 ;
        }
        }
          if(nKeyKneadWidth == KNEAD_WIDTH_UNKNOWN)
          {
          nKeyKneadWidth = KNEAD_WIDTH_MED ;
        }
          ManualDirector[0].nSubFunction = BACK_SUB_MODE_MUSIC ;
          ManualDirector[0].nKneadMotorState = KNEAD_RUN ;
          ManualDirector[0].nKneadMotorCycles = 0 ;
          ManualDirector[0].nKnockMotorState = KNOCK_RUN_MUSIC ;
          ManualDirector[0].nKnockMotorRunTime = 0 ;
          ManualDirector[0].nKnockMotorStopTime = 0 ;
          ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          
          ManualDirector[1].nSubFunction = BACK_SUB_MODE_MUSIC ;
          ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MIN ;
          ManualDirector[1].nKneadMotorCycles = 0 ;
          ManualDirector[1].nKnockMotorState = KNOCK_RUN_MUSIC ;
          ManualDirector[1].nKnockMotorRunTime = 0 ;
          ManualDirector[1].nKnockMotorStopTime = 0 ;
          ManualDirector[1].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          
          ManualDirector[2].nSubFunction = BACK_SUB_MODE_MUSIC ;
          ManualDirector[2].nKneadMotorState = KNEAD_RUN ;
          ManualDirector[2].nKneadMotorCycles = 0 ;
          ManualDirector[2].nKnockMotorState = KNOCK_RUN_MUSIC ;
          ManualDirector[2].nKnockMotorRunTime = 0 ;
          ManualDirector[2].nKnockMotorStopTime = 0 ;
          ManualDirector[2].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          
          ManualDirector[3].nSubFunction = BACK_SUB_MODE_MUSIC ;
          ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MAX ;
          ManualDirector[3].nKneadMotorCycles = 0 ;
          ManualDirector[3].nKnockMotorState = KNOCK_RUN_MUSIC ;
          ManualDirector[3].nKnockMotorRunTime = 0 ;
          ManualDirector[3].nKnockMotorStopTime = 0 ;
          ManualDirector[3].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          
          //设置揉捏电机(立即更新动作)
          bKneadMotorInProcess = TRUE ;
          nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
          nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
          //设置捶击电机(立即更新动作)
          bKnockMotorInProcess = TRUE ;
          nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
          nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
          nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
          
          nMaxActionStep = 4 ;
          nStartActionStep = 0 ;
          bBackManualModeInit = TRUE ;
          //开启振动
          // bKeySeatVibrate = TRUE ;
          break;
          
        }
          break ;
          */
          
          //case KEY_LOCATE_PARTIAL://以当前点为中心上下移动
        case H10_KEY_LOCATE_FULL:
        case H10_KEY_LOCATE_POINT:
        case H10_KEY_LOCATE_PART:  //新手控器为局部
          if(bKeyPowerSwitch == FALSE) break ;
          if(KR_PROGRAM)
          {
            if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2)) break;
          }
          if(nBackMainRunMode != BACK_MAIN_MODE_MANUAL) break ;
          
          switch(nCommandID)
          {
          case H10_KEY_LOCATE_FULL:
            nKeyBackLocate = LOCATE_FULL_BACK;
            break;
          case H10_KEY_LOCATE_PART:
            nKeyBackLocate = LOCATE_PARTIAL;
            break;
          case H10_KEY_LOCATE_POINT:
            nKeyBackLocate = LOCATE_POINT;
            break;
          }
          nKeyBackLocateStore = nKeyBackLocate ;
          /*
          switch(nKeyBackLocate)
          {
        case LOCATE_FULL_BACK:     nKeyBackLocate = LOCATE_PARTIAL;  break;
        case LOCATE_POINT: nKeyBackLocate = LOCATE_FULL_BACK; break;
        case LOCATE_PARTIAL: nKeyBackLocate = LOCATE_POINT; break;
        }
          */
          //nKeyBackLocate = LOCATE_PARTIAL;
          
          /*
          if(nPartSubMode<2)
          nPartSubMode++;
          else
          nPartSubMode = 0;
          */
          //if(nPartSubMode == 0)		//全程
          if(nKeyBackLocate == LOCATE_FULL_BACK)		//全程
          {
            /**************/
            nKeyBackLocate = LOCATE_FULL_BACK ;
            /*
            if((nCurWalkMotorState == STATE_IDLE) ||
            (nCurWalkMotorState == STATE_RUN_CLOCK) ||
            (nCurWalkMotorState == STATE_STOP_CLOCK_ZV) ||
            (nCurWalkMotorState == STATE_STOP_CLOCK_HV))
            */
            {
              ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
              ManualDirector[0].nWalkMotorLocateParam = 0 ;
              ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
              ManualDirector[1].nWalkMotorLocateParam = 0 ;
              ManualDirector[2].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
              ManualDirector[2].nWalkMotorLocateParam = 0 ;
              ManualDirector[3].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
              ManualDirector[3].nWalkMotorLocateParam = 0 ;
            }
            /*
                        else
            {
            ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
            ManualDirector[0].nWalkMotorLocateParam = 0 ;
            ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
            ManualDirector[1].nWalkMotorLocateParam = 0 ;
            ManualDirector[2].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
            ManualDirector[2].nWalkMotorLocateParam = 0 ;
            ManualDirector[3].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
            ManualDirector[3].nWalkMotorLocateParam = 0 ;
          }
            */
            /**************/
          }
          // if(nPartSubMode == 1)
          if(nKeyBackLocate == LOCATE_PARTIAL)
          {
            nKeyBackLocate = LOCATE_PARTIAL ;
            if(Input_GetWalkMotorPosition() >= (TOP_POSITION - HALF_PARTIAL_DIFF))
            {
              nPartialTop = TOP_POSITION ;
              nPartialBottom = Input_GetWalkMotorPosition() - PARTIAL_DIFF ;
            }
            else if(Input_GetWalkMotorPosition() <= HALF_PARTIAL_DIFF)
            {
              nPartialTop = PARTIAL_DIFF ;
              nPartialBottom = 0 ;
            }
            else
            {
              nPartialTop = Input_GetWalkMotorPosition() + HALF_PARTIAL_DIFF ;
              nPartialBottom = Input_GetWalkMotorPosition() - HALF_PARTIAL_DIFF ;
            }
            /*
            if((nCurWalkMotorState == STATE_IDLE) ||
            (nCurWalkMotorState == STATE_RUN_CLOCK) ||
            (nCurWalkMotorState == STATE_STOP_CLOCK_ZV) ||
            (nCurWalkMotorState == STATE_STOP_CLOCK_HV))
            */
            {
              ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
              ManualDirector[0].nWalkMotorLocateParam = nPartialBottom ;
              ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
              ManualDirector[1].nWalkMotorLocateParam = nPartialTop ; ;
              ManualDirector[2].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
              ManualDirector[2].nWalkMotorLocateParam = nPartialBottom ;
              ManualDirector[3].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
              ManualDirector[3].nWalkMotorLocateParam = nPartialTop ; ;
            }
            /*
                        else
            {
            ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
            ManualDirector[0].nWalkMotorLocateParam = nPartialTop ;
            ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
            ManualDirector[1].nWalkMotorLocateParam = nPartialBottom ;
            ManualDirector[2].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
            ManualDirector[2].nWalkMotorLocateParam = nPartialTop ;
            ManualDirector[3].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
            ManualDirector[3].nWalkMotorLocateParam = nPartialBottom ;
          }
            */
          }
          if(nKeyBackLocate == LOCATE_POINT)
          {
            nKeyBackLocate = LOCATE_POINT ;
            ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_PARK ;
            ManualDirector[0].nWalkMotorLocateParam = MAX_PARK_TIME ;
            ManualDirector[1].nWalkMotorLocateMethod = WALK_LOCATE_PARK ;
            ManualDirector[1].nWalkMotorLocateParam = MAX_PARK_TIME ;
            ManualDirector[2].nWalkMotorLocateMethod = WALK_LOCATE_PARK ;
            ManualDirector[2].nWalkMotorLocateParam = MAX_PARK_TIME ;
            ManualDirector[3].nWalkMotorLocateMethod = WALK_LOCATE_PARK ;
            ManualDirector[3].nWalkMotorLocateParam = MAX_PARK_TIME ;
          }
          bBackManualModeInit = TRUE ;
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          break ;
	  
        case H10_KEY_SPEED_INCREASE:
        case H10_KEY_SPEED_DECREASE:
        case H10_KEY_SPEED_1:
        case H10_KEY_SPEED_2:
        case H10_KEY_SPEED_3:
        case H10_KEY_SPEED_4:
        case H10_KEY_SPEED_5:
        case H10_KEY_SPEED_6:
          if(bKeyPowerSwitch == FALSE) break ;
          if(nBackMainRunMode != BACK_MAIN_MODE_MANUAL) break;
          if((nBackSubRunMode == BACK_SUB_MODE_PRESS) || (nBackSubRunMode == BACK_SUB_MODE_NO_ACTION) || (nMaunalSubMode == nMaunalSubMode_MUSIC)) break ;
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          // VfdLedBuzzer.bLedBackSpeed = 1;
          if(nCommandID == H10_KEY_SPEED_INCREASE)
          {
            if(nKeyKneadKnockSpeed < 6)
            {
              nKeyKneadKnockSpeed++ ;
            }
            else
            {
              nKeyKneadKnockSpeed = 1;
            }
          }
          if(nCommandID == H10_KEY_SPEED_1)
          {
            nKeyKneadKnockSpeed = 1;
          }
          
          if(nCommandID == H10_KEY_SPEED_2)
          {
            nKeyKneadKnockSpeed = 2;
          }
          if(nCommandID == H10_KEY_SPEED_3)
          {
            nKeyKneadKnockSpeed = 3;
          }
          if(nCommandID == H10_KEY_SPEED_4)
          {
            nKeyKneadKnockSpeed = 4;
          }
          if(nCommandID == H10_KEY_SPEED_5)
          {
            nKeyKneadKnockSpeed = 5;
          }
          if(nCommandID == H10_KEY_SPEED_6)
          {
            nKeyKneadKnockSpeed = 6;
          }
          ManualDirector[0].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          ManualDirector[1].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          ManualDirector[2].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          ManualDirector[3].nKneadKnockSpeed = nKeyKneadKnockSpeed ;
          nCurKneadKnockSpeed = nKeyKneadKnockSpeed ;
          break ;
          //#ifdef RT8301_CONTROL
        case H10_KEY_WIDTH_INCREASE:
        case H10_KEY_WIDTH_DECREASE:
        case H10_KEY_WIDTH_MIN:
        case H10_KEY_WIDTH_MED:
        case H10_KEY_WIDTH_MAX:
          if(bKeyPowerSwitch == FALSE) break ;
          if(nBackMainRunMode != BACK_MAIN_MODE_MANUAL) break ;
          if(!((nBackSubRunMode == BACK_SUB_MODE_KNOCK) || (nBackSubRunMode == BACK_SUB_MODE_PRESS) || (nBackSubRunMode == BACK_SUB_MODE_SOFT_KNOCK))) break ;
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          bKneadWidthChange = FALSE ;
          bKneadWidthChange = TRUE ;
          switch(nCommandID)
          {
          case  H10_KEY_WIDTH_INCREASE:
            {
              if(nKeyKneadWidth < 3)
              {
                nKeyKneadWidth++ ;
              }
              else
              {
                nKeyKneadWidth = 1 ;
              }
            }
            break;
          case H10_KEY_WIDTH_MIN:
            nKeyKneadWidth = KNEAD_WIDTH_MIN;
            break;
          case H10_KEY_WIDTH_MED:
            nKeyKneadWidth = KNEAD_WIDTH_MED;
            break;
          case H10_KEY_WIDTH_MAX:
            nKeyKneadWidth = KNEAD_WIDTH_MAX;
            break;
          }
          if(bKneadWidthChange == TRUE)
          {
            switch(nKeyKneadWidth)
            {
            case KNEAD_WIDTH_MIN:
              ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MIN ;
              ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MIN ;
              ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MIN ;
              ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MIN ;
              break ;
            case KNEAD_WIDTH_MED:
              ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MED ;
              ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MED ;
              ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MED ;
              ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MED ;
              break ;
            case KNEAD_WIDTH_MAX:
              ManualDirector[0].nKneadMotorState = KNEAD_STOP_AT_MAX ;
              ManualDirector[1].nKneadMotorState = KNEAD_STOP_AT_MAX ;
              ManualDirector[2].nKneadMotorState = KNEAD_STOP_AT_MAX ;
              ManualDirector[3].nKneadMotorState = KNEAD_STOP_AT_MAX ;
              break ;
            }
            ManualDirector[0].nKneadMotorCycles = 0 ;
            //重新定位
            nKneadMotorControlParam1 = ManualDirector[0].nKneadMotorState ;
            //if(nKneadMotorControlParam1)
            nKneadMotorControlParam2 = 0 ;
            bKneadMotorInProcess = TRUE ;
            //Knock motor 要等定位完成后进行
            bKnockMotorInProcess = TRUE ;
          }
          break ;
          //#else

          
          //#endif
          
        case H10_KEY_AIRBAG_LEG:
          if(KR_PROGRAM)
          {
            if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2)) break;
          }
          {
            if(bKeyPowerSwitch == FALSE) break ;
            // if(nBackMainRunMode != BACK_MAIN_MODE_MANUAL) break ;
            if(nKeyAirBagLocate == AIRBAG_LOCATE_AUTO)
            {
              nRollerPWM = 0;
              nRollerPWMStore = 0;
              Valve_SetRollerPWM(nRollerPWM);
              bRollerEnable = FALSE;
              bRollerEnableStore = FALSE;
              nKeyAirBagLocate = AIRBAG_LOCATE_NONE;
              //140531
              nKeyAirBagLocateStore = AIRBAG_LOCATE_NONE ;
            }
            if(nKeyAirBagLocate != AIRBAG_LOCATE_LEG_FOOT)
            {
              Valve_CloseAll();//Fungares
              nKeyAirBagLocate = AIRBAG_LOCATE_LEG_FOOT ;
              //140531
              nKeyAirBagLocateStore = AIRBAG_LOCATE_LEG_FOOT ;
              
              st_AirBagAuto3.init = TRUE ;
              if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
              {
                Valve_SetAirBagStrength(AIRBAG_STRENGTH_3);
              }
              nChairRunState = CHAIR_STATE_RUN ;
              nIndicateTimer = RUN_INDICATE_TIME;
              if(Data_Get_Time() == 0)
              {
                Data_Set_Start(1, w_PresetTime);
              }
            }
            else
            {
              nKeyAirBagLocate = AIRBAG_LOCATE_NONE;
              //140531
              nKeyAirBagLocateStore = AIRBAG_LOCATE_NONE ;
            }
            nBuzzerMode = BUZZER_MODE_ONETIME ;
            bSendBuzzerMode = TRUE ;
            bBlueToothSendBuzzerMode = TRUE;
          }
          break;
        case H10_KEY_AIRBAG_ARM:
          if(KR_PROGRAM)
          {
            if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2)) break;
          }
          {
            if(bKeyPowerSwitch == FALSE) break ;
            //  if(nBackMainRunMode != BACK_MAIN_MODE_MANUAL) break ;
            if(nKeyAirBagLocate == AIRBAG_LOCATE_AUTO)
            {
              nRollerPWM = 0;
              nRollerPWMStore = 0;
              Valve_SetRollerPWM(nRollerPWM);
              bRollerEnable = FALSE;
              bRollerEnableStore = FALSE;
              nKeyAirBagLocate = AIRBAG_LOCATE_NONE;
              //140531
              nKeyAirBagLocateStore = AIRBAG_LOCATE_NONE ;
            }
            if(nKeyAirBagLocate != AIRBAG_LOCATE_ARM_SHOLDER)
            {
              Valve_CloseAll();//Fungares
              nKeyAirBagLocate = AIRBAG_LOCATE_ARM_SHOLDER ;
              //140531
              nKeyAirBagLocateStore = AIRBAG_LOCATE_ARM_SHOLDER ;
              
              st_AirBagAuto1.init = TRUE ;
              if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
              {
                Valve_SetAirBagStrength(AIRBAG_STRENGTH_3);
              }
              
              nChairRunState = CHAIR_STATE_RUN ;
              nIndicateTimer = RUN_INDICATE_TIME;
              if(Data_Get_Time() == 0)
              {
                Data_Set_Start(1, w_PresetTime);
              }
            }
            else
            {
              nKeyAirBagLocate = AIRBAG_LOCATE_NONE;
              //140531
              nKeyAirBagLocateStore = AIRBAG_LOCATE_NONE ;
            }
            nBuzzerMode = BUZZER_MODE_ONETIME ;
            bSendBuzzerMode = TRUE ;
            bBlueToothSendBuzzerMode = TRUE;
          }
          break;
         /**********************************Fungares**********************************
        case H10_KEY_AIRBAG_WAIST:
          if(KR_PROGRAM)
          {
            if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2)) break;
          }
          {
            if(bKeyPowerSwitch == FALSE) break ;
            //  if(nBackMainRunMode != BACK_MAIN_MODE_MANUAL) break ;
            if(nKeyAirBagLocate == AIRBAG_LOCATE_AUTO)
            {
              nRollerPWM = 0;
              nRollerPWMStore = 0;
              Valve_SetRollerPWM(nRollerPWM);
              bRollerEnable = FALSE;
              bRollerEnableStore = FALSE;
              nKeyAirBagLocate = AIRBAG_LOCATE_NONE;
              //140531
              nKeyAirBagLocateStore = AIRBAG_LOCATE_NONE ;
            }
            if(nKeyAirBagLocate != AIRBAG_LOCATE_BACK_WAIST)
            {
              nKeyAirBagLocate = AIRBAG_LOCATE_BACK_WAIST ;
              //140531
              nKeyAirBagLocateStore = AIRBAG_LOCATE_BACK_WAIST ;
              
              st_AirBagBackWaist.init = TRUE ;
              if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
              {
                Valve_SetAirBagStrength(AIRBAG_STRENGTH_3);
              }
              nChairRunState = CHAIR_STATE_RUN ;
              nIndicateTimer = RUN_INDICATE_TIME;
              if(Data_Get_Time() == 0)
              {
                Data_Set_Start(1, w_PresetTime);
              }
            }
            else
            {
              nKeyAirBagLocate = AIRBAG_LOCATE_NONE;
              //140531
              nKeyAirBagLocateStore = AIRBAG_LOCATE_NONE ;
            }
            nBuzzerMode = BUZZER_MODE_ONETIME ;
            bSendBuzzerMode = TRUE ;
            bBlueToothSendBuzzerMode = TRUE;
          }
          break;
          **********************************Fungares***************************/
        case H10_KEY_AIRBAG_BUTTOCKS:
          if(KR_PROGRAM)
          {
            if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2)) break;
          }
          {
            if(bKeyPowerSwitch == FALSE) break ;
            //  if(nBackMainRunMode != BACK_MAIN_MODE_MANUAL) break ;
            if(nKeyAirBagLocate == AIRBAG_LOCATE_AUTO)
            {
              nRollerPWM = 0;
              nRollerPWMStore = 0;
              Valve_SetRollerPWM(nRollerPWM);
              bRollerEnable = FALSE;
              bRollerEnableStore = FALSE;
              nKeyAirBagLocate = AIRBAG_LOCATE_NONE;
              //140531
              nKeyAirBagLocateStore = AIRBAG_LOCATE_NONE ;
            }
            if(nKeyAirBagLocate != AIRBAG_LOCATE_SEAT)
            {
              Valve_CloseAll();//Fungares
              nKeyAirBagLocate = AIRBAG_LOCATE_SEAT ;
              //140531
              nKeyAirBagLocateStore = AIRBAG_LOCATE_SEAT ;
              
              st_AirBagAuto2.init = TRUE ;
              if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
              {
                Valve_SetAirBagStrength(AIRBAG_STRENGTH_3);
              }
              nChairRunState = CHAIR_STATE_RUN ;
              nIndicateTimer = RUN_INDICATE_TIME;
              if(Data_Get_Time() == 0)
              {
                Data_Set_Start(1, w_PresetTime);
              }
            }
            else
            {
              nKeyAirBagLocate = AIRBAG_LOCATE_NONE;
              //140531
              nKeyAirBagLocateStore = AIRBAG_LOCATE_NONE ;
            }
            nBuzzerMode = BUZZER_MODE_ONETIME ;
            bSendBuzzerMode = TRUE ;
            bBlueToothSendBuzzerMode = TRUE;
          }
          break;
          
          //ZONE4:Memory Function Key
          //ZONE5:Walk Up/Down,Backpad Up/Down,Legpad Up/Down
          //Walk Up/Down
        case H10_KEY_WALK_UP_START:
          w_KeyWalkHoldTimer = 1;
          if(nChairRunState == CHAIR_STATE_IDLE)
          {
            nvcBluetoothDescoverable = 1;
          }
          if((nBackMainRunMode == BACK_MAIN_MODE_MANUAL) && (nKeyBackLocate == LOCATE_POINT))
          {
            ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_TOP ;
            ManualDirector[0].nWalkMotorLocateParam = 0 ;
            bBackManualModeInit = TRUE ;
            bKeyWalkUp = TRUE ;
          }
          else if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) && (nCurSubFunction == BACK_SUB_MODE_BODY_DETECT_1))
          {
            nWalkMotorControlParam1 = WALK_LOCATE_ABSULATE ;
            nWalkMotorControlParam2 = nShoulderPositionTop ;
            bUpdateLocate = TRUE ;
            bKeyWalkUp = TRUE ;
            bWalkMotorInProcess = TRUE ;
            //VfdLedBuzzer.bodyDetectSuccess = 1;
            bodyDetectSuccess = 1;
            bFail = 0;
          }
          break ;
        case H10_KEY_WALK_UP_STOP:
          w_KeyWalkHoldTimer = 0;
          bKeyWalkUp = FALSE ;
          bKeyWalkDown = FALSE ; //only pc test
          nBuzzerMode = BUZZER_MODE_OFF ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          if((nBackMainRunMode == BACK_MAIN_MODE_MANUAL) && (nKeyBackLocate == LOCATE_POINT))
          {
            ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_PARK ;
            ManualDirector[0].nWalkMotorLocateParam = MAX_PARK_TIME ;
            bBackManualModeInit = TRUE ;
          }
          else if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) && (nCurSubFunction == BACK_SUB_MODE_BODY_DETECT_1))
          {
            nWalkMotorControlParam1 = WALK_LOCATE_PARK ;
            nWalkMotorControlParam2 = 0 ;
            bUpdateLocate = TRUE ;
            bWalkMotorInProcess = TRUE ;
          }
          break ;
        case H10_KEY_WALK_DOWN_START:
          if(nChairRunState == CHAIR_STATE_IDLE)
          {
            nvcBluetoothPair = 1;
          }
          
          if((nBackMainRunMode == BACK_MAIN_MODE_MANUAL) && (nKeyBackLocate == LOCATE_POINT))
          {
            ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_ABSULATE ;
            ManualDirector[0].nWalkMotorLocateParam = 0 ;
            bBackManualModeInit = TRUE ;
            bKeyWalkDown = TRUE ;
          }
          else if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) && (nCurSubFunction == BACK_SUB_MODE_BODY_DETECT_1))
          {
            nWalkMotorControlParam1 = WALK_LOCATE_ABSULATE ;
            nWalkMotorControlParam2 = nShoulderPositionBottom ;
            bKeyWalkDown = TRUE ;
            bUpdateLocate = TRUE ;
            bWalkMotorInProcess = TRUE ;
            //VfdLedBuzzer.bSuccess = 1;
            bodyDetectSuccess = 1;
            bFail = 0;
          }
          break ;
        case H10_KEY_WALK_DOWN_STOP:
          bKeyWalkDown = FALSE ;
          nBuzzerMode = BUZZER_MODE_OFF ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          if((nBackMainRunMode == BACK_MAIN_MODE_MANUAL) && (nKeyBackLocate == LOCATE_POINT))
          {
            ManualDirector[0].nWalkMotorLocateMethod = WALK_LOCATE_PARK ;
            ManualDirector[0].nWalkMotorLocateParam = MAX_PARK_TIME ;
            bBackManualModeInit = TRUE ;
          }
          else if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) && (nCurSubFunction == BACK_SUB_MODE_BODY_DETECT_1))
          {
            nWalkMotorControlParam1 = WALK_LOCATE_PARK ;
            nWalkMotorControlParam2 = 0 ;
            bUpdateLocate = TRUE ;
            bWalkMotorInProcess = TRUE ;
          }
          break ;
        case H10_KEY_BACKPAD_UP_START:
          if(KR_PROGRAM)
          {
            if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2)) break;
          }
          //退出摇摆模式  140906
          //RockFunctionEnable(false);
          nTargetMassagePosition = MASSAGE_UNKNOW_POSITION;//fww
          st_Stretch.active = FALSE;
          bKeyBackPadUp = TRUE ;
          bKeyBackPadDown = FALSE ;
          //小腿联动设置
          bKeyLegPadDown = TRUE ;
          bKeyLegPadUp = FALSE ;
          bLegPadLinkage = TRUE ;
          //bKeyFlexOut = FALSE ;  //fww
          //bKeyFlexIn = FALSE ;   //fww
          break ;
        case H10_KEY_BACKPAD_UP_STOP:
#ifdef TEST_VALVE
          ValveTestData <<= 1;
          if(ValveTestData >= 0xFFFFFF)
          {
            ValveTestData = 0x01;
          }
          Valve_Test_Set_Data(ValveTestData);
          printf("Data:0X%06x\r\n", ValveTestData);
#endif
          st_Stretch.active = FALSE;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = FALSE ;
          //小腿联动设置
          bKeyLegPadDown = FALSE ;
          bKeyLegPadUp = FALSE ;
          bLegPadLinkage = TRUE ;
          //bKeyFlexOut = FALSE ;  //fww
          //bKeyFlexIn = FALSE ;
          break ;
        case H10_KEY_BACKPAD_DOWN_START:
          if(KR_PROGRAM)
          {
            if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2)) break;
          }
          //退出摇摆模式
          //RockFunctionEnable(false);
          //if(bKeyPowerSwitch == FALSE) break ;
          nTargetMassagePosition = MASSAGE_UNKNOW_POSITION;//fww
          st_Stretch.active = FALSE;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = TRUE ;
          //小腿联动设置
          bKeyLegPadDown = FALSE ;
          bKeyLegPadUp = TRUE ;
          bLegPadLinkage = TRUE ;
          //bKeyFlexOut = FALSE ;//fww
          //bKeyFlexIn = FALSE ;
          break ;
        case H10_KEY_BACKPAD_DOWN_STOP:
#ifdef TEST_VALVE
          ValveTestData <<= 1;
          if(ValveTestData >= 0xFFFFFF)
          {
            ValveTestData = 0x01;
          }
          Valve_Test_Set_Data(ValveTestData);
          printf("Data:0X%06x\r\n", ValveTestData);
#endif
          st_Stretch.active = FALSE;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = FALSE ;
          //小腿联动设置
          bKeyLegPadDown = FALSE ;
          bKeyLegPadUp = FALSE ;
          bLegPadLinkage = TRUE ;
          //bKeyFlexOut = FALSE ;//fww
          //bKeyFlexIn = FALSE ;
          break ;
          /*************************************************
        case H10_KEY_LEGPAD_EXTEND_START:
#ifdef  FORCE_CONTROLLER
          if(nBackMainRunMode == BACK_MAIN_MODE_MANUAL)
          {
            switch(nCurSubFunction)
            {
            case BACK_SUB_MODE_KNEAD:
              KneadMotor_AdcCurrent();
              break;
            case BACK_SUB_MODE_KNOCK:
              KnockMotor_AdcCurrent();
              break;
            case BACK_SUB_MODE_WAVELET:
              KnockMotor_AdcCurrent();
              KneadMotor_AdcCurrent();
              break;
            case BACK_SUB_MODE_SOFT_KNOCK:
              KnockMotor_AdcCurrent();
              break;
            case BACK_SUB_MODE_PRESS:
              break;
            case BACK_SUB_MODE_MUSIC:
              KneadMotor_AdcCurrent();
              KnockMotor_AdcCurrent();
              break;
            default		:
              break;
            case BACK_SUB_MODE_RUBBING:
              KneadMotor_AdcCurrent();
              break;
            }
          }
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          break;
#endif
          //退出摇摆模式 140906
          //RockFunctionEnable(false);
          st_Stretch.active = FALSE;
          bKeyLegPadUp = FALSE ;
          bKeyLegPadDown = FALSE ;
          bLegPadLinkage = FALSE ;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = FALSE ;
          bKeyFlexOut = TRUE ;
          bKeyFlexIn = FALSE ;
          break;
        case H10_KEY_LEGPAD_EXTEND_STOP:
        case H10_KEY_LEGPAD_CONTRACT_STOP:
          st_Stretch.active = FALSE;
          bKeyLegPadUp = FALSE ;
          bKeyLegPadDown = FALSE ;
          bLegPadLinkage = FALSE ;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = FALSE ;
          bKeyFlexOut = FALSE;
          bKeyFlexIn = FALSE ;
          break;
        case H10_KEY_LEGPAD_CONTRACT_START:
#ifdef  FORCE_CONTROLLER
          if(nBackMainRunMode == BACK_MAIN_MODE_MANUAL)
          {
            switch(nCurSubFunction)
            {
            case BACK_SUB_MODE_KNEAD:
              KneadMotor_DecCurrent();
              break;
            case BACK_SUB_MODE_KNOCK:
              KnockMotor_DecCurrent();
              break;
            case BACK_SUB_MODE_WAVELET:
              KnockMotor_DecCurrent();
              KneadMotor_DecCurrent();
              break;
            case BACK_SUB_MODE_SOFT_KNOCK:
              KnockMotor_DecCurrent();
              break;
            case BACK_SUB_MODE_PRESS:
              break;
            case BACK_SUB_MODE_MUSIC:
              KneadMotor_DecCurrent();
              KnockMotor_DecCurrent();
              break;
            default		:
              break;
            case BACK_SUB_MODE_RUBBING:
              KneadMotor_DecCurrent();
              break;
            }
          }
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          break;
#endif
          //退出摇摆模式 140906
         // RockFunctionEnable(false);
          st_Stretch.active = FALSE;
          bKeyLegPadUp = FALSE ;
          bKeyLegPadDown = FALSE ;
          bLegPadLinkage = FALSE ;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = FALSE ;
          bKeyFlexOut = FALSE;
          bKeyFlexIn = TRUE ;
          break;
          ************************************/
        case H10_KEY_LEGPAD_UP_START:
          if(KR_PROGRAM)
          {
            if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2)) break;
          }
          //退出摇摆模式 140906
          //RockFunctionEnable(false);
          nTargetMassagePosition = MASSAGE_UNKNOW_POSITION;//fww
          st_Stretch.active = FALSE;
          bKeyLegPadUp = TRUE ;
          bKeyLegPadDown = FALSE ;
          bLegPadLinkage = FALSE ;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = FALSE ;
          //bKeyFlexOut = FALSE ;//fww
          //bKeyFlexIn = FALSE ;
          break ;
        case H10_KEY_LEGPAD_UP_STOP:
          st_Stretch.active = FALSE;
          bKeyLegPadUp = FALSE ;
          bKeyLegPadDown = FALSE ;
          bLegPadLinkage = FALSE ;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = FALSE ;
          //bKeyFlexOut = FALSE ;//fww
          //bKeyFlexIn = FALSE ;
          break ;
        case H10_KEY_LEGPAD_DOWN_START:
          if(KR_PROGRAM)
          {
            if((nChairRunState == CHAIR_STATE_RUN) && (nBackSubRunMode == BACK_SUB_MODE_AUTO_2)) break;
          }
          //退出摇摆模式 140906
          //RockFunctionEnable(false);
          nTargetMassagePosition = MASSAGE_UNKNOW_POSITION;//fww
          st_Stretch.active = FALSE;
          bKeyLegPadUp = FALSE ;
          bKeyLegPadDown = TRUE ;
          bLegPadLinkage = FALSE ;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = FALSE ;
          //bKeyFlexOut = FALSE ;//fww
          //bKeyFlexIn = FALSE ;
          break ;
        case H10_KEY_LEGPAD_DOWN_STOP:
          st_Stretch.active = FALSE;
          bKeyLegPadUp = FALSE ;
          bKeyLegPadDown = FALSE ;
          bLegPadLinkage = FALSE ;
          bKeyBackPadUp = FALSE ;
          bKeyBackPadDown = FALSE ;
          //bKeyFlexOut = FALSE ;//fww
          //bKeyFlexIn = FALSE ;
          break ;
  /**************************************************************************        
        case H10_KEY_WHEEL_SPEED_OFF:
          if(bKeyPowerSwitch == FALSE) break ;
          bRollerEnable = FALSE;
          bRollerEnableStore = FALSE;
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          nRollerPWM = 0;
          nRollerPWMStore = 0;
          Valve_SetRollerPWM(nRollerPWM);
          break;
        case H10_KEY_WHEEL_SPEED_SLOW:
        case H10_KEY_WHEEL_SPEED_MED:
        case H10_KEY_WHEEL_SPEED_FAST:
          if(bKeyPowerSwitch == FALSE) break ;
          
          if(bRollerEnable != FALSE)
          {
            if(nKeyAirBagLocate == AIRBAG_LOCATE_AUTO) break; //在自动气囊程序中滚速度不可以调整
          }
          if(bRollerEnable == FALSE)
          {
            bRollerEnable = TRUE;
            bRollerEnableStore = TRUE;
          }
          if(nCommandID ==  H10_KEY_WHEEL_SPEED_SLOW)
          {
            nRollerPWM = 1;
          }
          if(nCommandID ==  H10_KEY_WHEEL_SPEED_MED)
          {
            nRollerPWM = 2;
          }
          if(nCommandID ==  H10_KEY_WHEEL_SPEED_FAST)
          {
            nRollerPWM = 3;
          }
          nRollerPWMStore = nRollerPWM ;
          Valve_SetRollerPWM(nRollerPWM);
          if(nRollerPWM != 0)
          {
            nChairRunState = CHAIR_STATE_RUN ;
            nIndicateTimer = RUN_INDICATE_TIME;
            if(Data_Get_Time() == 0)
            {
              Data_Set_Start(1, w_PresetTime);
            }
          }
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          break;
 **********************************************************************/  
          
          case H10_KEY_WHEEL_SPEED_OFF:
          if(bKeyPowerSwitch == FALSE) break ;
          if(nKeyAirBagLocate == AIRBAG_LOCATE_AUTO)
          {
            if(bRollerEnable)
            {
              nRollerPWM = 0;
            bRollerEnable = 0;
            }
           else
           {
             bRollerEnable =1;
           }
           nBuzzerMode = BUZZER_MODE_ONETIME ;
           bSendBuzzerMode = TRUE ;
           break;
          }
          nRollerPWM++;
          nRollerPWM %= 4;
          if(nRollerPWM)
          {
           nRollerPWMStore = nRollerPWM;
           bRollerEnable = TRUE;
           nRollerPWMStore = TRUE;
           Valve_SetRollerPWM(nRollerPWM); 
          }
          else
          {
            bRollerEnable = FALSE;
            bRollerEnableStore = FALSE;
            nRollerPWM = 0;
            nRollerPWMStore = 0;
            Valve_SetRollerPWM(nRollerPWM);
          }
           if(nRollerPWM != 0)
          {
            nChairRunState = CHAIR_STATE_RUN ;
            nIndicateTimer = RUN_INDICATE_TIME;
            if(Data_Get_Time() == 0)
            {
              Data_Set_Start(1, w_PresetTime);
            }
          }
            nBuzzerMode = BUZZER_MODE_ONETIME ;
            bSendBuzzerMode = TRUE ;
            bBlueToothSendBuzzerMode = TRUE;
          break;

		case H10_KEY_HEAT:    //加热
			if(bKeyPowerSwitch == FALSE) break ;
			if(bKeyWaistHeat == FALSE)
			{
				bKeyWaistHeat = TRUE ;
				//140623
				bKeyWaistHeatStore = TRUE ;
				nChairRunState = CHAIR_STATE_RUN ;
				nIndicateTimer = RUN_INDICATE_TIME;
            
				if(Data_Get_Time() == 0)
				{
					Data_Set_Start(1, w_PresetTime);
				}
			}
			else
			{
				bKeyWaistHeat = FALSE ;
				//140623
				bKeyWaistHeatStore = FALSE ;
			}
			nBuzzerMode = BUZZER_MODE_ONETIME ;
			bSendBuzzerMode = TRUE ;
			bBlueToothSendBuzzerMode = TRUE;
			break;
          //RESET_CONFIRM PROCESS
       // case H10_KEY_RESET_CONFIRM:
          /*if(nChairRunState != CHAIR_STATE_SETTLE_1ST) break;
          //bPowerOffReach =  FALSE;
          //140526
          FlexMotor_Control(STATE_FLEX_IDLE,FLEX_SPEED_FAST,FLEX_CURRENT_3A_1);
          //Massage Chair Reset To The Original Position
          nBackMainRunMode = BACK_MAIN_MODE_SETTLE ; 
          nChairRunState = CHAIR_STATE_SETTLE ;
          bBackLegPadSettle = TRUE ;
          nTargetMassagePosition = MASSAGE_RESET_POSITION;
          bMassagePositionUpdate = TRUE;
          //140527 start count flag
          bFlexMotorRestTimeStartFlag = 1;
          //FlexMotor_ResetTime = 0;*/
          
          //bSelectKeyValue = KEYCONFIRM ;
          
         // break;
        //case H10_KEY_RESET_CANCLE:
          
          //bSelectKeyValue = KEYCANCLE ;
          
         // break;
        default:
          break;
        }
      }
#ifdef  OTG_MP3
      switch(nCommandID)
      {
        //MP3 Player
      case H10_KEY_MP3_PLAY_PAUSE:
        if(Input_GetMp3Status() == MP3_ERROR)
        {
          nBuzzerMode = BUZZER_MODE_TWOTIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          break ;
        }
        bMP3RunMode = MP3_PLAY ;
        MP3KeyControl1_PlayPause();
        bMP3_AD_Enable = TRUE;
        Power_AMP_On();
        Timer_Counter_Clear(C_TIMER_MP3);
        nBuzzerMode = BUZZER_MODE_ONETIME ;
        bSendBuzzerMode = TRUE ;
        bBlueToothSendBuzzerMode = TRUE;
        break ;
      case H10_KEY_MP3_STOP:
        if(Input_GetMp3Status() == MP3_ERROR)
        {
          nBuzzerMode = BUZZER_MODE_TWOTIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          break ;
        }
        bMP3RunMode = MP3_STOP ;
        MP3Power_Off();
        bMP3_AD_Enable = FALSE;
        MP3KeyControl1_Stop();
        Timer_Counter_Clear(C_TIMER_MP3);
        nBuzzerMode = BUZZER_MODE_ONETIME ;
        bSendBuzzerMode = TRUE ;
        break ;
      case H10_KEY_MP3_PREVIOUS:
        if(Input_GetMp3Status() == MP3_ERROR)
        {
          nBuzzerMode = BUZZER_MODE_TWOTIME ;
          bSendBuzzerMode = TRUE ;
          break ;
        }
        if(bMP3RunMode == MP3_STOP) break ;
        MP3KeyControl1_Previous();
        Timer_Counter_Clear(C_TIMER_MP3);
        nBuzzerMode = BUZZER_MODE_ONETIME ;
        bSendBuzzerMode = TRUE ;
        break ;
      case H10_KEY_MP3_NEXT:
        if(Input_GetMp3Status() == MP3_ERROR)
        {
          nBuzzerMode = BUZZER_MODE_TWOTIME ;
          bSendBuzzerMode = TRUE ;
          break ;
        }
        if(bMP3RunMode == MP3_STOP) break ;
        MP3KeyControl1_Next();
        Timer_Counter_Clear(C_TIMER_MP3);
        nBuzzerMode = BUZZER_MODE_ONETIME ;
        bSendBuzzerMode = TRUE ;
        break ;
      case H10_KEY_MP3_VOLUME_INCREASE:
        if(Input_GetMp3Status() == MP3_ERROR)
        {
          nBuzzerMode = BUZZER_MODE_TWOTIME ;
          bSendBuzzerMode = TRUE ;
          break ;
        }
        if(bMP3RunMode == MP3_STOP) break ;
        MP3KeyControl1_VolumeInc();
        Timer_Counter_Clear(C_TIMER_MP3);
        nBuzzerMode = BUZZER_MODE_ONETIME ;
        bSendBuzzerMode = TRUE ;
        break ;
      case H10_KEY_MP3_VOLUME_DECREASE:
        if(Input_GetMp3Status() == MP3_ERROR)
        {
          nBuzzerMode = BUZZER_MODE_TWOTIME ;
          bSendBuzzerMode = TRUE ;
          break ;
        }
        if(bMP3RunMode == MP3_STOP) break ;
        MP3KeyControl1_VolumeDec();
        Timer_Counter_Clear(C_TIMER_MP3);
        nBuzzerMode = BUZZER_MODE_ONETIME ;
        bSendBuzzerMode = TRUE ;
        break ;
      }
#endif /* OTG_MP3  */
    }
    //以下按键用于工程模式
    else
    {
      if(nChairRunState == CHAIR_STATE_IDLE)
      {
        nChairRunState = CHAIR_STATE_ENGINEERING;
      }
    }
  }
#ifndef RT8301_CONTROL
  Main_Send();
  Main_BlueToothSend();
#else
  if(bMasterSendPacket == TRUE)
  {
    //加快蜂鸣器单音的响应时间,不适应连续声音，因为其连续发送，会阻止Get_Command和Set_State
    //而单音只会阻止一次
    if((bSendBuzzerMode == TRUE) && ((nBuzzerMode == BUZZER_MODE_ONETIME) || (nBuzzerMode == BUZZER_MODE_TWOTIME)))
    {
      nSendPacketID = PACKET_MASTER_ACK_COMMAND ;
    }
    
    if(nvcBluetoothDescoverable == 1)
    {
      nvcBluetoothDescoverable = 2;
      //开启蓝牙模块的可发现功能
      OutBuffer[0] = 'B' ;
      OutBuffer[1] = 'C' ;
      OutBuffer[2] = ':' ;
      OutBuffer[3] = 'M' ;
      OutBuffer[4] = 'D' ;
      OutBuffer[5] = '=' ;
      OutBuffer[6] = '0' ;
      OutBuffer[7] = '1' ;
      OutBuffer[8] = '\r' ;
      OutBuffer[9] = '\n' ;
      nOutBufferCount = 10;
      DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
      bMasterSendPacket = FALSE;
      return;
    }
    if(nvcBluetoothPair == 1)
    {
      nvcBluetoothPair = 2;
      //配对
      OutBuffer[0] = 'B' ;
      OutBuffer[1] = 'C' ;
      OutBuffer[2] = ':' ;
      OutBuffer[3] = 'N' ;
      OutBuffer[4] = 'C' ;
      OutBuffer[5] = '=' ;
      OutBuffer[6] = '0' ;
      OutBuffer[7] = '1' ;
      OutBuffer[8] = '\r' ;
      OutBuffer[9] = '\n' ;
      nOutBufferCount = 10;
      DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
      bMasterSendPacket = FALSE;
      return;
    }
    
    
    // memset(VfdLedBuzzer.nVfdLedBuzzer,0,sizeof(VfdLedBuzzer.nVfdLedBuzzer));  //先将显示全部清0
    // VfdLedBuzzer.nVfdLedBuzzer[3] = 0x20;
    //VfdLedBuzzer.nVfdLedBuzzer[0] = 0x04;
    //VfdLedBuzzer.nVfdLedBuzzer[] = 0xff;
    //VfdLedBuzzer.bThighAirBag1 = bDisplayFlash ;
    //VfdLedBuzzer.bFootAirBag = bDisplayFlash ;
    //VfdLedBuzzer.bLowerLegAirBag = bDisplayFlash ;
    //VfdLedBuzzer.bArmAirBag =  bDisplayFlash;
    //VfdLedBuzzer.bLeftThighAirBag = bDisplayFlash ;
    //VfdLedBuzzer.bRightThighAirBag = bDisplayFlash ;
    //VfdLedBuzzer.bButtocksAirBag = bDisplayFlash ;
    // VfdLedBuzzer.bShoulderAirBAag  = bDisplayFlash ;
    switch(nSendPacketID)
    {
    case PACKET_MASTER_GET_COMMAND:
      nOutBufferCount = 18 ;
      nSendCount = 0 ;
      OutBuffer[0] = SOI ;
      OutBuffer[1] = ValToAscii((PACKET_MASTER_GET_COMMAND & 0xf0) >> 4) ;
      OutBuffer[2] = ValToAscii(PACKET_MASTER_GET_COMMAND & 0x0f) ;
      OutBuffer[3] = ValToAscii((VfdLedBuzzer.nVfdLedBuzzer[8] & 0xf0) >> 4) ;
      OutBuffer[4] = ValToAscii(VfdLedBuzzer.nVfdLedBuzzer[8] & 0x0f) ;
      OutBuffer[5] = ValToAscii((VfdLedBuzzer.nVfdLedBuzzer[9] & 0xf0) >> 4) ;
      OutBuffer[6] = ValToAscii(VfdLedBuzzer.nVfdLedBuzzer[9] & 0x0f) ;
      OutBuffer[7] = ValToAscii((VfdLedBuzzer.nVfdLedBuzzer[10] & 0xf0) >> 4) ;
      OutBuffer[8] = ValToAscii(VfdLedBuzzer.nVfdLedBuzzer[10] & 0x0f) ;
      OutBuffer[9] = ValToAscii((VfdLedBuzzer.nVfdLedBuzzer[11] & 0xf0) >> 4) ;
      OutBuffer[10] = ValToAscii(VfdLedBuzzer.nVfdLedBuzzer[11] & 0x0f) ;
      OutBuffer[11] = ValToAscii((VfdLedBuzzer.nVfdLedBuzzer[12] & 0xf0) >> 4) ;
      OutBuffer[12] = ValToAscii(VfdLedBuzzer.nVfdLedBuzzer[12] & 0x0f) ;
      OutBuffer[13] = ValToAscii((VfdLedBuzzer.nVfdLedBuzzer[13] & 0xf0) >> 4) ;
      OutBuffer[14] = ValToAscii(VfdLedBuzzer.nVfdLedBuzzer[13] & 0x0f) ;
      OutBuffer[15] = ValToAscii((VfdLedBuzzer.nVfdLedBuzzer[14] & 0xf0) >> 4) ;
      OutBuffer[16] = ValToAscii(VfdLedBuzzer.nVfdLedBuzzer[14] & 0x0f) ;
      OutBuffer[17] = EOI;
      //PIR1bits.TX1IF为ReadOnly,要禁止中断只能清除TXEN
      //PIR1bits.TX1IF = 0 ;
      //while(BusyUSART()) ;
      //TXSTA1bits.TXEN = 1 ;//Enable TXEN will also set TXIF to trigger TX interrupt
      //PIE1bits.TXIE = 1 ;//Enable Transmite Interrupt
      nSendPacketID = PACKET_MASTER_SET_STATE ;
      DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
      break ;
    case PACKET_MASTER_SET_STATE:
      nOutBufferCount = 20 ;
      nSendCount = 0 ;
      OutBuffer[0] = SOI ;
      OutBuffer[1] = ValToAscii((PACKET_MASTER_SET_STATE & 0xf0) >> 4) ;
      OutBuffer[2] = ValToAscii(PACKET_MASTER_SET_STATE & 0x0f) ;
      OutBuffer[3] = ValToAscii((VfdLedBuzzer.nVfdLedBuzzer[0] & 0xf0) >> 4) ;
      OutBuffer[4] = ValToAscii(VfdLedBuzzer.nVfdLedBuzzer[0] & 0x0f) ;
      OutBuffer[5] = ValToAscii((VfdLedBuzzer.nVfdLedBuzzer[1] & 0xf0) >> 4) ;
      OutBuffer[6] = ValToAscii(VfdLedBuzzer.nVfdLedBuzzer[1] & 0x0f) ;
      OutBuffer[7] = ValToAscii((VfdLedBuzzer.nVfdLedBuzzer[2] & 0xf0) >> 4) ;
      OutBuffer[8] = ValToAscii(VfdLedBuzzer.nVfdLedBuzzer[2] & 0x0f) ;
      OutBuffer[9] = ValToAscii((VfdLedBuzzer.nVfdLedBuzzer[3] & 0xf0) >> 4) ;
      OutBuffer[10] = ValToAscii(VfdLedBuzzer.nVfdLedBuzzer[3] & 0x0f) ;
      OutBuffer[11] = ValToAscii((VfdLedBuzzer.nVfdLedBuzzer[4] & 0xf0) >> 4) ;
      OutBuffer[12] = ValToAscii(VfdLedBuzzer.nVfdLedBuzzer[4] & 0x0f) ;
      OutBuffer[13] = ValToAscii((VfdLedBuzzer.nVfdLedBuzzer[5] & 0xf0) >> 4) ;
      OutBuffer[14] = ValToAscii(VfdLedBuzzer.nVfdLedBuzzer[5] & 0x0f) ;
      OutBuffer[15] = ValToAscii((VfdLedBuzzer.nVfdLedBuzzer[6] & 0xf0) >> 4) ;
      OutBuffer[16] = ValToAscii(VfdLedBuzzer.nVfdLedBuzzer[6] & 0x0f) ;
      OutBuffer[17] = ValToAscii((VfdLedBuzzer.nVfdLedBuzzer[7] & 0xf0) >> 4) ;
      OutBuffer[18] = ValToAscii(VfdLedBuzzer.nVfdLedBuzzer[7] & 0x0f) ;
      OutBuffer[19] = EOI;
      //while(BusyUSART()) ;
      //TXSTA1bits.TXEN = 1 ;//Enable TXEN will also set TXIF to trigger TX interrupt
      //PIE1bits.TXIE = 1 ;//Enable Transmite Interrupt
      //nSendPacketID = PACKET_MASTER_GET_COMMAND ;
      if(bSendBuzzerMode == TRUE)
      {
        nSendPacketID = PACKET_MASTER_ACK_COMMAND ;
      }
      else
      {
        nSendPacketID = PACKET_MASTER_GET_COMMAND ;
      }
      DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
      break ;
    case PACKET_MASTER_ACK_COMMAND:
      //蜂鸣器的处理
      bSendBuzzerMode = FALSE ;
      nOutBufferCount = 6 ;
      nSendCount = 0 ;
      OutBuffer[0] = SOI ;
      OutBuffer[1] = ValToAscii((PACKET_MASTER_ACK_COMMAND & 0xf0) >> 4) ;
      OutBuffer[2] = ValToAscii(PACKET_MASTER_ACK_COMMAND & 0x0f) ;
      OutBuffer[3] = ValToAscii((nBuzzerMode & 0xf0) >> 4) ;
      OutBuffer[4] = ValToAscii(nBuzzerMode & 0x0f) ;
      OutBuffer[5] = EOI;
      //while(BusyUSART()) ;
      //TXSTA1bits.TXEN = 1 ;//Enable TXEN will also set TXIF to trigger TX interrupt
      //PIE1bits.TXIE = 1 ;//Enable Transmite Interrupt
      nSendPacketID = PACKET_MASTER_GET_COMMAND ;
      DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
      break ;
    default:
      nSendPacketID = PACKET_MASTER_GET_COMMAND ;
      break ;
    }
    bMasterSendPacket = FALSE ;
  }
#endif
  
}

unsigned char WalkMotorControl(unsigned char nWalkMotorLocateMethod, unsigned short nWalkMotorLocateParam)
{
  //if(bResetStateORchangeModeDis == EnterDisReset) return 0;
    Main_Shoulder_Detect();
    //坐标更新，只有在更换动作时才执行一次
    unsigned short by_TopPosition = TOP_POSITION;
    if(bUpdateLocate == TRUE)
    {
        bUpdateLocate = FALSE ;
        nWalkMotorLocateState = nWalkMotorLocateMethod;
        switch(nWalkMotorLocateMethod)
        {
        default:
            bWalkMotorInProcess = FALSE ;
            break;
        case WALK_LOCATE_ABSULATE:    //运行到绝对位置
            nFinalWalkMotorLocate = nWalkMotorLocateParam ;
            break ;
        case WALK_LOCATE_SHOULDER:    //运行到肩膀位置
            if(by_TopPosition - nWalkMotorLocateParam > nShoulderPosition)
            {
                nFinalWalkMotorLocate = nShoulderPosition + nWalkMotorLocateParam ;
            }
            else
            {
                nFinalWalkMotorLocate = by_TopPosition ;
            }
            break ;
        case WALK_LOCATE_TOP:  //运行到上端行程
#ifdef TOP_BY_LIMIT
            nFinalWalkMotorLocate = by_TopPosition ;
#else
            if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
            {
                //nFinalWalkMotorLocate = nShoulderPosition + DEFAULT_NECK_LENGTH ;//有缺陷，可能会溢出
                if(nShoulderPosition >= by_TopPosition - DEFAULT_NECK_LENGTH)
                {
                    nFinalWalkMotorLocate = by_TopPosition ;
                }
                else
                {
                    nFinalWalkMotorLocate = nShoulderPosition + DEFAULT_NECK_LENGTH ;
                }
            }
            else
            {
                nFinalWalkMotorLocate = by_TopPosition ;
            }
#endif
            break ;
        case WALK_LOCATE_SHOULDER_OR_ABSULATE:  //由肩部位置和绝对坐标中的较小者决定
            if(nWalkMotorLocateParam > nShoulderPosition)
            {
                nFinalWalkMotorLocate = nShoulderPosition ;
            }
            else
            {
                nFinalWalkMotorLocate = nWalkMotorLocateParam ;
            }
            break ;
        case WALK_LOCATE_PARK: //停留在当前位置
            WalkMotor_Control(STATE_WALK_IDLE, 0);
            //nFinalWalkMotorState = STATE_IDLE ;
            nCurActionStepCounter = 0 ;
            break ;

        case WALK_LOCATE_NeckSwitch:
            nFinalWalkMotorLocate = by_TopPosition ;
            break;

        case WALK_LOCATE_NeckMed: //脖子位置
            if(nShoulderPosition >= by_TopPosition - Med_NECK_LENGTH)
            {
                nFinalWalkMotorLocate = by_TopPosition ;
            }
            else
            {
                nFinalWalkMotorLocate = nShoulderPosition + Med_NECK_LENGTH ;
            }

            break;
        case WALK_LOCATE_PressNeck: //肩膀位置
            nFinalWalkMotorLocate = nShoulderPosition;	// - 10 ;
            break;
        }//end switch
        //保证不超过最高位
        if(nFinalWalkMotorLocate > by_TopPosition)
            nFinalWalkMotorLocate = by_TopPosition;
    }//end if

    //以下判断 walk 行程（bWalkMotorInProcess）何时停止

    if(nWalkMotorLocateMethod == WALK_LOCATE_PARK)
    {
        //判断是否到达停留时间
        WalkMotor_Control(STATE_WALK_IDLE, 0);
        if((nWalkMotorLocateParam != MAX_PARK_TIME) && (nCurActionStepCounter >= nWalkMotorLocateParam))
        {
            bWalkMotorInProcess = FALSE ;
        }
    }
    else
    {
        if(nFinalWalkMotorLocate == 0)  //行程最终位置为0
        {
            if(WalkMotor_Control(STATE_RUN_WALK_DOWN, 0))
            {
                bWalkMotorInProcess = FALSE ;
            }
        }
        else if(nFinalWalkMotorLocate >= by_TopPosition) //行程最终位置为最高
        {
            if(WalkMotor_Control(STATE_RUN_WALK_UP, 0))
            {
                bWalkMotorInProcess = FALSE ;
            }
        }
        else
        {
            //行程最终位置为任意位置
            if(WalkMotor_Control(STATE_RUN_WALK_POSITION, nFinalWalkMotorLocate))
            {
                bWalkMotorInProcess = FALSE ;
            }
        }
    }
    return 0;
}


//Knead Motor Control with Brake function
//No Relay, Only 5 states:
//1 STATE_IDLE
//2 STATE_RUN_CLOCK
//3 STATE_STOP_CLOCK_HV
//4 STATE_STOP_CLOCK_BRAKE
//5 STATE_STOP_CLOCK_ZV

//1.RUN_CLOCK->STOP_CLOCK_HV
void KneadMotor_RunClock_StopClockHV(void)
{
    bKneadMotorPowerFlag = FALSE ;
    nCurKneadMotorState = STATE_STOP_CLOCK_HV ;
    nCurKneadMotorStateCounter = 0 ;
}
//2.STOP_CLOCK_HV->RUN_CLOCK
void KneadMotor_StopClockHV_RunClock(void)
{
    bKneadMotorPowerFlag = TRUE ;
    nCurKneadMotorState = STATE_RUN_CLOCK ;
    nCurKneadMotorStateCounter = 0 ;
}
//3.STOP_CLOCK_HV->STOP_CLOCK_BRAKE
void KneadMotor_StopClockHV_StopClockBrake(void)
{
    if(nCurKneadMotorStateCounter >= PRE_BRAKE_TIME)
    {
        //bKneadMotorBrake = BRAKE_ON ;
        KneadMotorBrake_On();
        nCurKneadMotorState = STATE_STOP_CLOCK_BRAKE ;
        nCurKneadMotorStateCounter = 0 ;
    }
}
//4.STOP_CLOCK_BRAKE->STOP_CLOCK_ZV
void KneadMotor_StopClockBrake_StopClockZV(void)
{
    if(nCurKneadMotorStateCounter >= BRAKE_TIME)
    {
        //bKneadMotorBrake = BRAKE_OFF ;
        KneadMotorBrake_Off();
        nCurKneadMotorState = STATE_STOP_CLOCK_ZV ;
        nCurKneadMotorStateCounter = 0 ;
        nPrevKneadMotorState = STATE_STOP_CLOCK_BRAKE ;
    }
}
//5.STOP_CLOCK_ZV->IDLE
void KneadMotor_StopClockZV_Idle(void)
{
    unsigned char nRealDelayTime ;
    if(nPrevKneadMotorState == STATE_STOP_CLOCK_BRAKE)
    {
        nRealDelayTime = POST_BRAKE_TIME + 100 ; //考虑到行走电机的特殊性
    }
    else
    {
        nRealDelayTime = RELAY_STABLE_TIME ;
    }
    if(nCurKneadMotorStateCounter >= nRealDelayTime)
    {
        nCurKneadMotorState = STATE_IDLE ;
        nCurKneadMotorStateCounter = 0 ;
    }
}
//6.IDLE->RUN_CLOCK
void KneadMotor_Idle_RunClock(void)
{
    bKneadMotorPowerFlag = TRUE ;
    nCurKneadMotorState = STATE_RUN_CLOCK ;
    nCurKneadMotorStateCounter = 0 ;
}



//行走电机控制
//bWalkMotorInProcess:每次更新动作时置位,完成动作时复位
//与行走电机相关的变量

void KneadMotorControl(unsigned char nKneadMotorState, unsigned char nKneadMotorCycles)
{
  //if(bResetStateORchangeModeDis == EnterDisReset) return;
    unsigned int speed;
    unsigned int step;
// static unsigned int nFinalKneadMotorState = STATE_IDLE;
    if(bKneadMotorInProcess == TRUE)
    {
        switch(nKneadMotorState)
        {
        default:
        case KNEAD_STOP:
            nFinalKneadMotorState = STATE_IDLE ;
            bKneadMotorInProcess = FALSE ;
            break ;
        case KNEAD_STOP_AT_MIN:
            if(nCurKneadWidth == KNEAD_WIDTH_MIN)
            {
                nFinalKneadMotorState = STATE_IDLE ;
                bKneadMotorInProcess = FALSE ;
            }
            else
            {
                if(bHasKneadWidthMinPulse == TRUE)
                {
                    bHasKneadWidthMinPulse = FALSE ;
                    if(Input_GetKneadMin() == 0)
                    {
                        nCurKneadWidth = KNEAD_WIDTH_MIN ;
                        nFinalKneadMotorState = STATE_IDLE ;
                        bKneadMotorInProcess = FALSE ;
                    }
                }
                else
                {
                    nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
                    nFinalKneadMotorState = STATE_RUN_CLOCK ;
                }
            }
            break ;
        case KNEAD_STOP_AT_MED:
            if(nCurKneadWidth == KNEAD_WIDTH_MED)
            {
                nFinalKneadMotorState = STATE_IDLE ;
                bKneadMotorInProcess = FALSE ;
            }
            else
            {
                if(bHasKneadWidthMedPulse == TRUE)
                {
                    bHasKneadWidthMedPulse = FALSE ;
                    if(Input_GetKneadMid() == 0)
                    {
                        nCurKneadWidth = KNEAD_WIDTH_MED ;
                        nFinalKneadMotorState = STATE_IDLE ;
                        bKneadMotorInProcess = FALSE ;
                    }
                }
                else
                {
                    nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
                    nFinalKneadMotorState = STATE_RUN_CLOCK ;
                }
            }
            break ;
        case KNEAD_STOP_AT_MAX:
            if(nCurKneadWidth == KNEAD_WIDTH_MAX)
            {
                nFinalKneadMotorState = STATE_IDLE ;
                bKneadMotorInProcess = FALSE ;
            }
            else
            {
                if(bHasKneadWidthMaxPulse == TRUE)
                {
                    bHasKneadWidthMaxPulse = FALSE ;
                    if(Input_GetKneadMax() == 0)
                    {
                        nCurKneadWidth = KNEAD_WIDTH_MAX ;
                        nFinalKneadMotorState = STATE_IDLE ;
                        bKneadMotorInProcess = FALSE ;
                    }
                }
                else
                {
                    nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
                    nFinalKneadMotorState = STATE_RUN_CLOCK ;
                }
            }
            break ;
        case KNEAD_RUN:
            nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
            nFinalKneadMotorState = STATE_RUN_CLOCK ;
            bKneadMotorInProcess = FALSE ;
            break ;
        case KNEAD_RUN_STOP:
        case KNEAD_RUN_STOP_AT_MIN:
            if(bHasKneadWidthMinPulse == TRUE)
            {
                bHasKneadWidthMinPulse = FALSE ;
                nCurKneadMotorCycles++ ;
                if(nCurKneadMotorCycles > nKneadMotorCycles)
                {
                    if(Input_GetKneadMin() == 0)
                    {
                        nCurKneadWidth = KNEAD_WIDTH_MIN ;
                        nFinalKneadMotorState = STATE_IDLE ;
                        bKneadMotorInProcess = FALSE ;
                    }
                }
            }
            else
            {
                nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
                nFinalKneadMotorState = STATE_RUN_CLOCK ;
            }
            break ;
        case KNEAD_RUN_STOP_AT_MED:
            if(bHasKneadWidthMedPulse == TRUE)
            {
                bHasKneadWidthMedPulse = FALSE ;
                nCurKneadMotorCycles++ ;
                if(nCurKneadMotorCycles > nKneadMotorCycles)
                {
                    if(Input_GetKneadMid() == 0)
                    {
                        nCurKneadWidth = KNEAD_WIDTH_MED ;
                        nFinalKneadMotorState = STATE_IDLE ;
                        bKneadMotorInProcess = FALSE ;
                    }
                }
            }
            else
            {
                nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
                nFinalKneadMotorState = STATE_RUN_CLOCK ;
            }
            break ;
        case KNEAD_RUN_STOP_AT_MAX:
            if(bHasKneadWidthMaxPulse == TRUE)
            {
                bHasKneadWidthMaxPulse = FALSE ;
                nCurKneadMotorCycles++ ;
                if(nCurKneadMotorCycles > nKneadMotorCycles)
                {
                    if(Input_GetKneadMax() == 0)
                    {
                        nCurKneadWidth = KNEAD_WIDTH_MAX ;
                        nFinalKneadMotorState = STATE_IDLE ;
                        bKneadMotorInProcess = FALSE ;
                    }
                }
            }
            else
            {
                nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
                nFinalKneadMotorState = STATE_RUN_CLOCK ;
            }
            break ;
//顺时针：窄-中-宽-半圈空闲-窄
//逆时针：宽-中-窄-半圈空闲-宽
            /*
            搓背程序依据  nCurKneadMotorCycles的值控制揉捏电机
            */
        case KNEAD_RUN_RUBBING:
            step = nCurKneadMotorCycles % 4;
            switch(step)
            {
            case 0:
                /**************判断是否到达最窄处*************************/
                if(bHasKneadWidthMinPulse == TRUE)
                {
                    bHasKneadWidthMinPulse = FALSE ;
                    nCurKneadWidth = KNEAD_WIDTH_MIN ;
                    nCurKneadMotorCycles++ ;       //到达窄位置加1
                    Timer_Counter_Clear(C_TIME_RUBBING);
                    nFinalKneadMotorState = STATE_IDLE ;
                }
                else
                {
                    nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
                    nFinalKneadMotorState = STATE_RUN_UNCLOCK ; //还未到达最窄处，继续逆时针转动
                }
                /*********************************************/
                break;
            case 1:  //停在最窄处
                /**************判断刹车时间************************/
                if(Timer_Counter(C_TIME_RUBBING, 1))
                {
                    nCurKneadMotorCycles++ ;       //加1
                    nFinalKneadMotorState = STATE_RUN_CLOCK ; //从最窄到最宽，进行顺时针运动
                }
                else
                {
                    nFinalKneadMotorState = STATE_IDLE ;
                }
                /*********************************************/
                break;
            case 2:
                /**************判断是否到达最窄处*************************/
                if(bHasKneadWidthMaxPulse == TRUE)
                {
                    bHasKneadWidthMaxPulse = FALSE ;
                    nCurKneadWidth = KNEAD_WIDTH_MAX ;
                    nCurKneadMotorCycles++ ;       //到达宽位置加1
                    Timer_Counter_Clear(C_TIME_RUBBING);
                    nFinalKneadMotorState = STATE_IDLE ;
                }
                else
                {
                    nCurKneadWidth = KNEAD_WIDTH_UNKNOWN ;
                    nFinalKneadMotorState = STATE_RUN_CLOCK ; //从窄到宽顺时针转动
                }
                /*********************************************/
                break;
            case 3:
                /**************判断刹车时间************************/
                if(Timer_Counter(C_TIME_RUBBING, 1))
                {
                    nCurKneadMotorCycles++ ;       //加1

                    if(nCurKneadMotorCycles > nKneadMotorCycles)
                    {
                        nFinalKneadMotorState = STATE_IDLE ;

                        if(bWalkMotorInProcess == FALSE)
                        {
                            bKneadMotorInProcess = FALSE ;
                        }

                        //bKneadMotorInProcess = FALSE ;
                    }
                    else
                    {
                        nFinalKneadMotorState = STATE_RUN_UNCLOCK ; //从最窄到最宽，进行顺时针运动
                    }
                }
                else
                {
                    nFinalKneadMotorState = STATE_IDLE ;
                }
                /*********************************************/
                break;
            }

            break;
        }
    }
    //确定揉捏马达的速度
    if((nKneadMotorState == KNEAD_STOP_AT_MIN) ||
            (nKneadMotorState == KNEAD_STOP_AT_MED) ||
            (nKneadMotorState == KNEAD_STOP_AT_MAX) ||
            (nKneadMotorState == KNEAD_RUN_STOP) )
    {
        speed =  KNEAD_SPEED2_PWM;
    }
    else
    {
        switch(nCurKneadKnockSpeed)
        {
        default:
        case 1:
            speed = KNEAD_SPEED1_PWM;
            break ;
        case 2:
            speed = KNEAD_SPEED2_PWM;
            break ;
        case 3:
            speed = KNEAD_SPEED3_PWM;
            break ;
        case 4:
            speed = KNEAD_SPEED4_PWM;
            break ;
        case 5:
            speed = KNEAD_SPEED5_PWM;
            break ;
        case 6:
            speed = KNEAD_SPEED6_PWM;
            break ;
       // case 0:
       //     speed = KNEAD_SPEED0_PWM;
       //     break ;    
        }
    }
    if(nFinalKneadMotorState == STATE_RUN_CLOCK)
    {
        KneadMotor_Control(STATE_KNEAD_CLOCK_RUN, speed);
    }
    if(nFinalKneadMotorState == STATE_RUN_UNCLOCK)
    {
        KneadMotor_Control(STATE_KNEAD_UNCLOCK_RUN, speed);
    }
    if(nFinalKneadMotorState == STATE_IDLE)
    {
        KneadMotor_Control(STATE_KNEAD_IDLE, speed);
    }
    /*
    if(nFinalKneadMotorState == STATE_RUN_CLOCK)
    {
      switch(nCurKneadMotorState)
      {
      case STATE_IDLE:
        KneadMotor_Idle_RunClock() ;
        break ;
      case STATE_RUN_CLOCK:
        break ;
      case STATE_STOP_CLOCK_HV:
        KneadMotor_StopClockHV_RunClock() ;
        break ;
      case STATE_STOP_CLOCK_BRAKE:
        KneadMotor_StopClockBrake_StopClockZV() ;
        break ;
      case STATE_STOP_CLOCK_ZV:
        KneadMotor_StopClockZV_Idle() ;
        break ;
      default://考虑最不利的情况
        KneadMotor_StopClockBrake_StopClockZV() ;
        break ;
      }
    }
    else //STATE_IDLE
    {
      switch(nCurKneadMotorState)
      {
      case STATE_IDLE:
        break ;
      case STATE_RUN_CLOCK:
        KneadMotor_RunClock_StopClockHV() ;
        break ;
      case STATE_STOP_CLOCK_HV:
        KneadMotor_StopClockHV_StopClockBrake() ;
        break ;
      case STATE_STOP_CLOCK_BRAKE:
        KneadMotor_StopClockBrake_StopClockZV() ;
        break ;
      case STATE_STOP_CLOCK_ZV:
        KneadMotor_StopClockZV_Idle() ;
        break ;
      default://考虑最不利的情况
        KneadMotor_StopClockBrake_StopClockZV() ;
        break ;
      }
    }
    if(bKneadMotorPowerFlag == TRUE)
    {
      if((nKneadMotorState == KNEAD_STOP_AT_MIN) ||
         (nKneadMotorState == KNEAD_STOP_AT_MED) ||
           (nKneadMotorState == KNEAD_STOP_AT_MAX) ||
             (nKneadMotorState == KNEAD_RUN_STOP) )
      {
         //用2档定位幅度
          KneadMotorUpdateSpeed(2);
      }
      else
      {
        KneadMotorUpdateSpeed(nCurKneadKnockSpeed);
      }
      KneadMotor_ClockRun();
    }
    else
    {
        KneadMotor_Set_Pwm_Data(KNEAD_SPEED0_PWM);
        KneadMotor_Reset();
    }
    */
}

//敲打音乐互动
unsigned int AD_KNOCK_PWM(unsigned int nADValue)
{
    unsigned int nRetPWM ;
    if(nADValue < ADSTRONG1)  nRetPWM = KNOCK_SPEED0_PWM;
    else if(nADValue < ADSTRONG2)  nRetPWM = KNOCK_SPEED1_PWM;
    else if(nADValue < ADSTRONG3)  nRetPWM = KNOCK_SPEED2_PWM;
    else if(nADValue < ADSTRONG4)  nRetPWM = KNOCK_SPEED3_PWM;
    else if(nADValue < ADSTRONG5)  nRetPWM = KNOCK_SPEED4_PWM;
    else if(nADValue < ADSTRONG6)  nRetPWM = KNOCK_SPEED5_PWM;
    else nRetPWM = KNOCK_SPEED6_PWM;
    return nRetPWM ;
}

//捶击电机控制
void KnockMotorControl(unsigned char nKnockMotorState, unsigned char nKnockingMotorRunTime, unsigned char nKnockingMotorStopTime)
{
  //if(bResetStateORchangeModeDis == EnterDisReset) return;
    //static int step = 0;
    //敲打电机音乐互动（高频）
    if((nBackMainRunMode == BACK_MAIN_MODE_MANUAL) &&
            (nBackSubRunMode == BACK_SUB_MODE_MUSIC) &&
            (nKnockMotorState == KNOCK_RUN_MUSIC))
    {
        nMusicKnockPWM = AD_KNOCK_PWM(nAvrADResult0) ;
        //unsigned int nLastPWM1Value = nMusicKnockPWM ;
        //SetDCPWM1(nMusicKnockPWM) ;
        KnockMotor_Set_Pwm_Data(nMusicKnockPWM);
        bKnockMotorInProcess = FALSE ;
        /*
        if(nMusicKnockPWM != 0)
        {
        bKnockMotorPowerFlag = TRUE ;
        }
        */
    }
    else
    {
        if(bKnockMotorInProcess == TRUE)
        {
            switch(nKnockMotorState)
            {
            default:
                bKnockMotorInProcess = FALSE ;
                while(1)
                {
                    WDOG_Feed();
                }
                break;
            case KNOCK_STOP:
                bKnockMotorPowerFlag = FALSE ;
                if(nCurActionStepCounter >= nKnockingMotorRunTime)
                {
                    bKnockMotorInProcess = FALSE ;
                }
                break ;
            case KNOCK_RUN_WIDTH://定位完成后进行
                if(bKneadMotorInProcess == TRUE)
                {
                    bKnockMotorPowerFlag = FALSE ;
                    nCurActionStepCounter = 0 ;
                }
                else
                {
                    bKnockMotorPowerFlag = TRUE ;
                    if(nCurActionStepCounter >= nKnockingMotorRunTime)
                    {
                        bKnockMotorInProcess = FALSE ;
                    }
                }
                break ;
            case KNOCK_RUN:
                bKnockMotorPowerFlag = TRUE ;
                if(nCurActionStepCounter >= nKnockingMotorRunTime)
                {
                    bKnockMotorInProcess = FALSE ;
                }
                break ;
            case KNOCK_RUN_STOP:  //叩击
                if(bKneadMotorInProcess == TRUE)
                {
                    bKnockMotorPowerFlag = FALSE ;
                    nCurKnockRunStopCounter = 0 ;//叩击动作记数器
                }
                else
                {
                    if(nCurKnockRunStopCounter < nKnockingMotorRunTime)//nCurKnockRunStopCounter:单位:2ms; nKnockingMotorRunTime:单位:100ms;
                    {
                        bKnockMotorPowerFlag = TRUE ;
                    }
                    if((nCurKnockRunStopCounter >= nKnockingMotorRunTime) && (nCurKnockRunStopCounter < (nKnockingMotorRunTime + nKnockingMotorStopTime)))
                    {
                        bKnockMotorPowerFlag = FALSE ;
                        //行走电机完成动作时，该动作也结束

                        if(bWalkMotorInProcess == FALSE)
                        {
                            bKnockMotorInProcess = FALSE ;
                        }

                    }
                    if(nCurKnockRunStopCounter >= (nKnockingMotorRunTime + nKnockingMotorStopTime))
                    {
                        nCurKnockRunStopCounter = 0 ;
                    }
                }
                /*
                if(bKneadMotorInProcess == TRUE)
                {
                bKnockMotorPowerFlag = FALSE ;
                nCurActionStepCounter = 0 ;
                }
                else
                {
                if(nCurActionStepCounter < nKnockingMotorRunTime)
                {
                bKnockMotorPowerFlag = TRUE ;
                }
                if((nCurActionStepCounter >= nKnockingMotorRunTime) && (nCurActionStepCounter < (nKnockingMotorRunTime + nKnockingMotorStopTime)))
                {
                bKnockMotorPowerFlag = FALSE ;
                //行走电机完成动作时，该动作也结束
                if(bWalkMotorInProcess == FALSE)
                {
                bKnockMotorInProcess = FALSE ;
                }
                }
                if(nCurActionStepCounter >= (nKnockingMotorRunTime + nKnockingMotorStopTime))
                {
                nCurActionStepCounter = 0 ;
                }
                }
                */
                break ;
            case KNOCK_RUN_MUSIC:
                break ;
            }
        }
        if(bKnockMotorPowerFlag == TRUE)
        {
            switch(nCurKneadKnockSpeed)
            {
            case 1:
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED1_PWM);
                break ;
            case 2:
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED2_PWM);
                break ;
            case 3:
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED3_PWM);
                break ;
            case 4:
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED4_PWM);
                break ;
            case 5:
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED5_PWM);
                break ;
            case 6:
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED6_PWM);
                break ;
            default:
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED1_PWM);
                break ;
            }
            KnockMotor_ClockRun();
        }
        else
        {

            KnockMotor_Set_Pwm_Data(KNOCK_SPEED0_PWM);
            KnockMotor_Break();
        }
    }
}

/***************************************************************/
#ifdef TWIST_ON
void WaveMotorStop(void)
{
    if(WaveMotor_IsRun())
    {
        if((Input_GetWaveMotorPosition() == 0) || (bWaveMotorFail))
        {
            Waveringly_Set_Pwm_Data(VIB_STRENGTH[0]);
        }
        else
        {
            Waveringly_Set_Pwm_Data(VIB_STRENGTH[1]);
        }
    }
}
#endif


//摇摆电机制函数
void VibrateMotorControl(void)
{
    if((!ValveFungares6)&&(!ValveFungares5))
    {
      VibrateLeftMotorControl(STATE_SLIDE_IDLE,Vibrate_Pwm_Speed);
    }
    if((ValveFungares6||ValveFungares5)&&bVibrateEnable)
    {
      VibrateLeftMotorControl(STATE_RUN_SLIDE_FORWARD,Vibrate_Pwm_Speed);
    }
    if((!ValveFungares4)&&(!ValveFungares3))
    {
      VibrateRightMotorControl(STATE_FLEX_IDLE,Vibrate_Pwm_Speed);
    }
    if((ValveFungares4||ValveFungares3)&&bVibrateEnable)
    {
      VibrateRightMotorControl(STATE_RUN_FLEX_TEST_OUT,Vibrate_Pwm_Speed);
    }
}

/********************************************/
void MusicSampling(void)
{
    unsigned short adcAudio_L, adcAudio_R;
    ADC_Get_ADC(ADC_AUDIO_L, &adcAudio_L);
    ADC_Get_ADC(ADC_AUDIO_R, &adcAudio_R);

    if(adcAudio_L >= adcAudio_R)
    {
        nAvrADResult0 = adcAudio_L - adcAudio_R  ;
    }
    else
    {
        nAvrADResult0 = adcAudio_R - adcAudio_L  ;
    }
}

void Display_ErrorCode(unsigned char ErrCode)
{
    SNSend(ErrCode);
}

void Close_Power(void)
{
    RollerMotor_Reset();
    KneadMotor_Reset();
    KnockMotor_Reset();
    //WalkMotor_Reset();
    WalkMotor_Control(STATE_WALK_IDLE, 0);
    LegMotor_Control(STATE_LEG_IDLE);
    BackMotor_Control(STATE_BACK_IDLE);
    WaistHeat_Off();
}
/*
void main_Problem(unsigned char ErrCode)
{
 // memset(VfdLedBuzzer.nVfdLedBuzzer,0,sizeof(VfdLedBuzzer.nVfdLedBuzzer));  //先将显示全部清0
  Display_ErrorCode(ErrCode);
  nBuzzerMode = BUZZER_MODE_FAST ;
  bSendBuzzerMode = TRUE ;
  Close_Power();
  Valve_Test_Set_Data(0);
  //Valve_ArmAirPumpACPowerOff();
  Valve_BodyUpAirPumpACPowerOff();
  Valve_LegFootAirPumpACPowerOff();
   nChairRunState = CHAIR_STATE_PROBLEM;
   nIndicateTimer = PROBLEM_INDICATE_TIME;
  while(1)
  {
    WDOG_Feed();
     if(Timer_Counter(C_TIMER_INDICATE + T_LOOP,nIndicateTimer))
        {
          IndicateLED_Toggle();
        }
 	 CommProcess() ;
     Valve_Send_Data();
     LED_RGB_Proce(nChairRunState);
  }
}
*/
/********************************************/
void Main_Initial_IO(void)
{
    __disable_irq();
    System_Initial_IO();
    Power_Initial_IO();
    IndicateLED_Initial_IO();
    HotRooler_Initial_IO();
    KneadMotor_Initial_IO();
    DMAUart_Initial_IO();
    BlueToothUart_Initial_IO();
    Valve_Initial_IO();
    ZeroMotor_Initial_IO();
    LegMotor_Initial_IO();
    BackMotor_Initial_IO();
    FlexMotor_Initial_IO();
    WalkMotor_Initial_IO();
    Input_Initial_IO();
    KnockMotor_Initial_IO();
    //Waveringly_Initial_IO();
    MP3Control1_Initial_IO();
    WaistHeat_Initial_IO();
    LED_RGB_Initial_IO();
    ADC_Data_Init();
    DAC_Initial_IO();
    //MotorPosCheck_Init() ;
    __enable_irq();
}

void Main_Initial_Data(void)
{
  GlobalFlags0.nByte = 0;
  GlobalFlags1.nByte = 0;
  GlobalFlags2.nByte = 0;
  GlobalFlags3.nByte = 0;
  GlobalFlags4.nByte = 0;
  GlobalFlags5.nByte = 0;
  GlobalFlags6.nByte = 0;
  GlobalFlags7.nByte = 0;
  GlobalFlags8.nByte = 0;
  GlobalFlags9.nByte = 0;
  GlobalFlags10.nByte = 0;
  GlobalFlags11.nByte = 0;
  nCurKneadKnockSpeed = 0; 
  unsigned int pw_Information[5];
  memset(pw_Information, 0, sizeof(pw_Information));
  PBYTE pInformation = (PBYTE)pw_Information;
  //
  if((SOFT_MAIN_VER != ReadEEByte(USER_DATA_BASE + SOFT_MAIN_VER_ADDRESS)) || (SOFT_SECONDARY_VER != ReadEEByte(USER_DATA_BASE + SOFT_SECONDARY_VER_ADDRESS)))
  {
    //首次使用需要初始化数据
    *(pInformation + SOFT_MAIN_VER_ADDRESS) = SOFT_MAIN_VER;
    *(pInformation + SOFT_SECONDARY_VER_ADDRESS) = SOFT_SECONDARY_VER;
    *(pInformation + SETTLE_ADDRESS) = MEMORY_DEFAULT_SETTLE;                    //Power off is pressed ,then reset or not
    *(pInformation + AIRBAG_STRETCH_ADDRESS) = MEMORY_DEFAULT_AIR;               //
    *(pInformation + SLIDE_MOTOR_ENABLE_ADDRESS) = SLIDE_DEFAULT_ENABLE;
    *(pInformation + STRETCH_OUT_ADDRESS) = 0;
    *(pInformation + SLEEP_PROGRAM_ADDRESS) = 0;
    *(pInformation + SLIDE_BLUETOOTH_POWER) = 0;  //FWW
    *(pInformation + TIME_SWITCH) = 20;
    
    MEM_Write_Memory(pw_Information, MEMORY_LENGTH_OF_BYTES);
  }
	//拉退程序依据行程开关控制
	nvcBluetoothPower = ReadEEByte(USER_DATA_BASE + SLIDE_BLUETOOTH_POWER); //fww
	if(nvcBluetoothPower == 1)
	{
		Power_AMP_On();
	}
	else
	{
		Power_AMP_Off();
	}
	w_PresetTime_Min = ReadEEByte(USER_DATA_BASE + TIME_SWITCH);   //fww 读Flash
  
	KR_PROGRAM = ReadEEByte(USER_DATA_BASE + SLEEP_PROGRAM_ADDRESS);
  //140528 confirm later ,add this case or not
  //bResetFlag = ReadEEByte(USER_DATA_BASE + SETTLE_ADDRESS) ;//fww
  
  st_Stretch.mode = STRETCH_MODE_SWITCH;
  st_Stretch.PresetTime = 200;
  st_Stretch.active = false;
  
            /*****************Fungares**********************/
            st_AirBagAuto0.pAirBagArray = AirBagAuto0;
            st_AirBagAuto0.nTotalSteps = sizeof(AirBagAuto0) / sizeof(struct AirBagStruct);
            st_AirBagAuto0.locate = AIRBAG_LOCATE_AUTO;
            st_AirBagAuto0.init = TRUE ;
            
            st_AirBagAuto1.pAirBagArray = AirBagAuto1;
            st_AirBagAuto1.nTotalSteps = sizeof(AirBagAuto1) / sizeof(struct AirBagStruct);
            st_AirBagAuto1.locate = AIRBAG_LOCATE_ARM_SHOLDER;
            st_AirBagAuto1.init = TRUE ;
            
            st_AirBagAuto2.pAirBagArray = AirBagAuto2;
            st_AirBagAuto2.nTotalSteps = sizeof(AirBagAuto2) / sizeof(struct AirBagStruct);
            st_AirBagAuto2.locate = AIRBAG_LOCATE_SEAT;
            st_AirBagAuto2.init = TRUE ;
            
            st_AirBagAuto3.pAirBagArray = AirBagAuto3;
            st_AirBagAuto3.nTotalSteps = sizeof(AirBagAuto3) / sizeof(struct AirBagStruct);
            st_AirBagAuto3.locate = AIRBAG_LOCATE_LEG_FOOT;
            st_AirBagAuto3.init = TRUE ;
            
  
  bKneckCheckSwitchLast = Input_GetVout();
  
  //Back Variables
  nBackMainRunMode = BACK_MAIN_MODE_IDLE ;
  //140530
  nCurBackMainRunModeStore = BACK_MAIN_MODE_IDLE ;
  nCurBackSubRunModeStore = BACK_SUB_MODE_NO_ACTION ;
  
  nKeyBackLocate = LOCATE_NONE;
  nKeyKneadWidth = KNEAD_WIDTH_UNKNOWN ;
  nKeyKneadKnockSpeed = SPEED_0 ;
  //Walk Motor Variables
  bWalkMotorInProcess = FALSE ;
  nWalkMotorControlParam1 = WALK_LOCATE_PARK ;
  nWalkMotorControlParam2 = 0 ;
  bUpdateLocate = TRUE ;
  nShoulderPosition = DEFAULT_SHOULDER_POSITION ;
  BodyDataRefresh() ;
  nKneadMotorControlParam1 = KNEAD_STOP ;
  nFinalKneadMotorState = STATE_IDLE ;
  nLastKneadPosition = KNEAD_WIDTH_UNKNOWN ;
  nCurBackPadMotorState = STATE_IDLE ;
  nCurLegPadMotorState = STATE_IDLE ;
  
  nKeyAirBagLocate = AIRBAG_LOCATE_NONE ;
  //140531
  nKeyAirBagLocateStore = AIRBAG_LOCATE_NONE ;
  nFinalWalkMotorLocate = TOP_POSITION;
  bRunTimeChange = TRUE ;
  //  bMP3TimeEnable = FALSE ;
  //Power_AMP_On();
  bMP3_AD_Enable = FALSE;
  //Communication
  bMasterSendPacket = FALSE ;
  nSendPacketID = PACKET_MASTER_GET_COMMAND ;
  nBuzzerMode = BUZZER_MODE_OFF ;
  bSendBuzzerMode = TRUE ;
  bBlueToothSendBuzzerMode = TRUE;
  //bZeroRunFlag = TRUE;
  //nNewZeroMotorState =  Zero_MOTOR_STATE_CLOCK;	//向下到底零重力初始化
  
  nTargetMassagePosition = MASSAGE_RESET_POSITION;
  // bMassagePositionUpdate = TRUE;
  bMassagePositionUpdate = FALSE;
  w_PresetTime = (unsigned int)w_PresetTime_Min*60;  //fww
  
  
  //滚轮数据初始化
  nRollerPWM = 0 ;
  nRollerPWMStore = 0 ;
  bRollerEnable = false ;
  bRollerEnableStore = false ;
  //扭腰数据初始化
  
  Data_Init();
  
  memset(OutBuffer, 0, sizeof(OutBuffer))  ;
  memset(InBuffer, 0, sizeof(InBuffer))  ;
  
  memset(&st_Stretch, 0, sizeof(StretchStruct));
  
  
  nReworkShoulderPosition = 0;
  nKeySeatVibrateStrength = 0 ;
  Valve_Initial_Data();
  nIndicateTimer = 0;
  nCurActionStep = 0;
  Timer_Initial();
  //140528
  //bResetStateORchangeModeDis = ExitDisReset ;//
  //bSelectKeyValue = KEY_NOT_CONFIRMORCANCLE ;//first in ,the mcu should think //fww
 // nTimerOverResetAgainFlag = FIRST_TIME_ENTER ;//fww
  
  //140529
  //nWaitCustomActionTime = 0 ;//fww
  //140530
  //  nBuzzer100msFlag = FALSE ;
  nSendBuzzerTimes = 0 ;
  //140530
  nCurMaunalSubModeStore = nMaunalSubMode_NO_ACTION ;
  //140531
  //FlexMotor_Data_Init() ;
  
  //140623
  bKeyWaistHeatStore = FALSE ;
  //141116
  nKneadBalanceCounter = 0;
  
}
//int aaaa;
//高电平 未压住
//低电平 压住

void Main_Shoulder_Detect(void)
{
	unsigned char nLimitTimes;
    //1.先往上走到顶,走到顶时会重新设置脉冲数
    if(!topPositionRefreshedFlag && Input_GetWalkUpSwitch() == REACH_LIMIT)
    {
        //手动设置顶点的脉冲数,第一次的正脉冲数不正确，需要手动设置
        Input_SetWalkMotorPosition(TOP_POSITION);
        Input_SetCounterWalkMotorPosition(0);
        //设置顶点参数只需一次
        topPositionRefreshedFlag = 1;
        // printf("s1\n");
    }
    bool bKneckCheckPosition = Input_GetVout();
    if(!bodyDetectSuccess && nCurSubFunction == BACK_SUB_MODE_BODY_DETECT_0 && topPositionRefreshedFlag)
    {
        unsigned short currentPosition = Input_GetWalkMotorPosition();
        //未检测成功时循环
        //超下界则往上
        if(currentPosition < LIMIT_POSITION)
        {
            nFinalWalkMotorLocate = DEFAULT_SHOULDER_POSITION;//TOP_POSITION;
            //越界后重新检测
            shoulderPositionScanStep = 1;
            /*
            nWalkMotorControlParam1 = WALK_LOCATE_TOP ;
            nWalkMotorControlParam2 = 0 ;
            bWalkMotorInProcess = TRUE ;
            bUpdateLocate = TRUE ;
            */
            //printf("越界\n");
        }
        //肩部位置检测第一步往上走，到这里第二步往下走
        //超上界则往下
        //在程序运行的时候，有时间上判断的误差
        else if(currentPosition >= TOP_POSITION)
        {
            if(shoulderPositionScanStep == 0)
            {
                //第一次在顶点，开始检测,第二次在顶点退出检测
                shoulderPositionScanStep = 1;
                nFinalWalkMotorLocate = DEFAULT_SHOULDER_POSITION;//0;
            }
        }
	/*
	//高电平=未压住 , 低电平=压住
        //从上往下 碰到身体
        if((bKneckCheckSwitchLast != BODY_TOUCHED) && (bKneckCheckPosition == BODY_TOUCHED))
        {
            //test_p2 = currentPosition;
            //判断是否合理
            if(currentPosition > LIMIT_POSITION && currentPosition <= TOP_POSITION)
            {
                switch(shoulderPositionScanStep)
                {
                case 1:
                    //采第一点
                    shoulderPos[0] = currentPosition;
                    shoulderPositionScanStep++;
					nLimitTimes = 0;
                    //printf("s2\n");
           
                    break;
                case 2:
                    //第二点在放开时采
                    //shoulderPos[1] = currentPosition;
                    break;
                case 3:
                    //采第三点
                    shoulderPos[2] = currentPosition;
                    //判断是否3点是否在允许误差范围内
                    if(((shoulderPos[0] - shoulderPos[1]) / LIMIT_PRECISION == 0)
                            && ((shoulderPos[0] - shoulderPos[2]) / LIMIT_PRECISION == 0)
                            && ((shoulderPos[1] - shoulderPos[2]) / LIMIT_PRECISION == 0))
                    {
						nLimitTimes = 0;
                        bodyDetectSuccess = 1;
                        //取平均值
                        nShoulderPosition = (shoulderPos[0] + shoulderPos[1] + shoulderPos[2]) / 3;
                        //上移,找到实际肩部位置
                        nShoulderPosition += 20;
                        BodyDataRefresh();
                        nFinalWalkMotorLocate = currentPosition;
                        return;
                    }
                    else
                    {
                        //无效数据时，重新检测
                        shoulderPositionScanStep = 1;
						nLimitTimes++;
						if(nLimitTimes >= 2)
						{
						nLimitTimes = 0;
                        bodyDetectSuccess = 1;
                        //取平均值
                        nShoulderPosition = shoulderPos[2];
                        //上移,找到实际肩部位置
                        nShoulderPosition += 20;
                        BodyDataRefresh();
                        nFinalWalkMotorLocate = currentPosition;
						}
                    }
                    break;
                }
            }
            //往上走
            nFinalWalkMotorLocate = TOP_POSITION;
			//nLimit++;
        }
        //从下往上 放开身体
        else if((bKneckCheckSwitchLast == BODY_TOUCHED) && (bKneckCheckPosition != BODY_TOUCHED))
        {
            if(currentPosition > LIMIT_POSITION && currentPosition <= TOP_POSITION)
            {
                if(shoulderPositionScanStep == 2)
                {
                    //采第二点
                    shoulderPos[1] = currentPosition;
                    shoulderPositionScanStep++;
                }
            }
            //放开后继续向下
            nFinalWalkMotorLocate = 0;//nShoulderPosition;//fww 解决机芯抖动问题，原来为0
        }*/
    }
    bKneckCheckSwitchLast = bKneckCheckPosition;
}

void main_GetKneadPosition(void)
{
    static unsigned char nLastKneadPosition = KNEAD_WIDTH_UNKNOWN ;
    unsigned char nNowKneadPosition = Input_GetKneadPosition();
    if(nNowKneadPosition != nLastKneadPosition)
    {
        nWidthOverTime = 0;
        if(nNowKneadPosition == KNEAD_WIDTH_MIN)
        {
            bHasKneadWidthMinPulse = TRUE ;
            bHasKneadWidthMedPulse = FALSE ;
            bHasKneadWidthMaxPulse = FALSE ;
            bDisplayKneadTrackMin = TRUE ;
            bDisplayKneadTrackMed = FALSE ;
            bDisplayKneadTrackMax = FALSE ;
            bDisplayKneadWidthMin = TRUE ;
            bDisplayKneadWidthMed = FALSE ;
            bDisplayKneadWidthMax = FALSE ;
            nLastKneadPosition = KNEAD_WIDTH_MIN ;
        }
        if(nNowKneadPosition == KNEAD_WIDTH_MED)
        {
            bHasKneadWidthMinPulse = FALSE ;
            bHasKneadWidthMedPulse = TRUE ;
            bHasKneadWidthMaxPulse = FALSE ;
            bDisplayKneadTrackMin = FALSE ;
            bDisplayKneadTrackMed = TRUE ;
            bDisplayKneadTrackMax = FALSE ;
            bDisplayKneadWidthMin = FALSE ;
            bDisplayKneadWidthMed = TRUE ;
            bDisplayKneadWidthMax = FALSE ;
            nLastKneadPosition = KNEAD_WIDTH_MED ;
        }
        if(nNowKneadPosition == KNEAD_WIDTH_MAX)
        {
            bHasKneadWidthMinPulse = FALSE ;
            bHasKneadWidthMedPulse = FALSE ;
            bHasKneadWidthMaxPulse = TRUE ;
            bDisplayKneadTrackMin = FALSE ;
            bDisplayKneadTrackMed = FALSE ;
            bDisplayKneadTrackMax = TRUE ;
            bDisplayKneadWidthMin = FALSE ;
            bDisplayKneadWidthMed = FALSE ;
            bDisplayKneadWidthMax = TRUE ;
            nLastKneadPosition = KNEAD_WIDTH_MAX ;
        }
    }
}

void main_50ms_int(void)
{
    bMasterSendPacket = TRUE;
}

void main_200ms_int(void)
{
  bBlueToothMasterSendPacket = TRUE;
}

void main_10ms_int(void)
{
    bTimer10MS = TRUE ;
    engineeringTime_10msFlag = 1;
}


//电动升降的显示
/*
void Display_BackLegStatus(void)
{
  switch(nCurBackPadMotorState)
  {
  case STATE_RUN_ANTICLOCK:
    switch(nBackLegPadFlashCount)
    {
    case 0:
      VfdLedBuzzer.bBackS2 = 1;
      break ;
    case 1:
      VfdLedBuzzer.bBackS2 = 1;
      VfdLedBuzzer.bBackS3 = 1;
      break ;
    case 2:
      VfdLedBuzzer.bBackS2 = 1;
      VfdLedBuzzer.bBackS3 = 1;
      break ;
    case 3:
      break ;
    }
    break ;
  case STATE_RUN_CLOCK:
    switch(nBackLegPadFlashCount)
    {
    case 0:
      VfdLedBuzzer.bBackS2 = 1;
      break ;
    case 1:
      VfdLedBuzzer.bBackS1 = 1;
      VfdLedBuzzer.bBackS2 = 1;
      break ;
    case 2:
      VfdLedBuzzer.bBackS1 = 1;
      VfdLedBuzzer.bBackS2 = 1;
      break ;
    case 3:
      break ;
    }
    break ;
  default:
    break ;
  }

  switch(nCurLegPadMotorState)
  {
  case STATE_RUN_ANTICLOCK:
    switch(nBackLegPadFlashCount)
    {
    case 0:
      VfdLedBuzzer.bBackS6 = 1;
      break ;
    case 1:
      VfdLedBuzzer.bBackS5 = 1;
      VfdLedBuzzer.bBackS6 = 1;
      break ;
    case 2:
      VfdLedBuzzer.bBackS5 = 1;
      VfdLedBuzzer.bBackS6 = 1;
      VfdLedBuzzer.bMassagerChairIcon = 1 ;
      break ;
    case 3:
      break ;
    }
    break ;
  case STATE_RUN_CLOCK:
    switch(nBackLegPadFlashCount)
    {
    case 0:
      VfdLedBuzzer.bBackS6 = 1;
      break ;
    case 1:
      VfdLedBuzzer.bBackS7 = 1;
      VfdLedBuzzer.bBackS6 = 1;
      break ;
    case 2:
      VfdLedBuzzer.bBackS7 = 1;
      VfdLedBuzzer.bBackS6 = 1;
      break ;
    case 3:
      break ;
    }
    break ;
  default:
    break ;
  }
}
*/
/*
void Display_WaitCommand(bool bFlash)
{
  VfdLedBuzzer.nVfdLedBuzzer[0] &= 0x40;
  VfdLedBuzzer.nVfdLedBuzzer[1] &= 0x10;
  VfdLedBuzzer.nVfdLedBuzzer[2] &= 0xc4;
  VfdLedBuzzer.nVfdLedBuzzer[5] &= 0x80;
  VfdLedBuzzer.nVfdLedBuzzer[7] &= 0x80;
  VfdLedBuzzer.nVfdLedBuzzer[8] &= 0x80;
  VfdLedBuzzer.nVfdLedBuzzer[9] &= 0x01;
  VfdLedBuzzer.nVfdLedBuzzer[10] &= 0x01;
  VfdLedBuzzer.nVfdLedBuzzer[11] &= 0x80;
  VfdLedBuzzer.nVfdLedBuzzer[12] &= 0x1f;
  VfdLedBuzzer.nVfdLedBuzzer[14] &= 0x80;

  RunTimeRefresh(0,0) ;
  VfdLedBuzzer.bMassagerChairIcon = 1 ;
  VfdLedBuzzer.bBodyIcon = 1 ;
  VfdLedBuzzer.bSeg82 = 1 ;
  VfdLedBuzzer.bLedPowerSwitch = bFlash ;
  bLedFullBackAutoMode0 = bFlash ;
  bLedFullBackAutoMode1 = bFlash ;
  bLedFullBackAutoMode2 = bFlash ;
  bLedFullBackAutoMode3 = bFlash ;
  VfdLedBuzzer.bLedBackManualMode = bFlash ;
  VfdLedBuzzer.bLedAirBagAuto = bFlash ;
  VfdLedBuzzer.bLedRoller = bFlash ;
}
*/
/*
void Display_Engineering(bool bFlash)
{

  VfdLedBuzzer.nVfdLedBuzzer[0] &= 0x40;
  VfdLedBuzzer.nVfdLedBuzzer[1] &= 0x10;
  VfdLedBuzzer.nVfdLedBuzzer[2] &= 0xc4;
  VfdLedBuzzer.nVfdLedBuzzer[5] &= 0x80;
  VfdLedBuzzer.nVfdLedBuzzer[7] &= 0x80;
  VfdLedBuzzer.nVfdLedBuzzer[8] &= 0x80;
  VfdLedBuzzer.nVfdLedBuzzer[9] &= 0x01;
  VfdLedBuzzer.nVfdLedBuzzer[10] &= 0x01;
  VfdLedBuzzer.nVfdLedBuzzer[11] &= 0x80;
  VfdLedBuzzer.nVfdLedBuzzer[12] &= 0x1f;
  VfdLedBuzzer.nVfdLedBuzzer[14] &= 0x80;

  //VfdLedBuzzer.bMassagerChairIcon = 1 ;
  VfdLedBuzzer.bBodyIcon = 1 ;
  VfdLedBuzzer.bSeg82 = 1 ;
  VfdLedBuzzer.bLedPowerSwitch = bFlash ;
  //bLedFullBackAutoMode0 = bFlash ;
  //bLedFullBackAutoMode1 = bFlash ;
  //bLedFullBackAutoMode2 = bFlash ;
  //bLedFullBackAutoMode3 = bFlash ;
  //VfdLedBuzzer.bLedBackManualMode = bFlash ;
  //VfdLedBuzzer.bLedAirBagAuto = bFlash ;
  //VfdLedBuzzer.bLedRoller = bFlash ;
}
*/
/*
void Display_FootRoller(void)
{
  if(bRollerEnable)
  {
    if(nKeyAirBagLocate == AIRBAG_LOCATE_AUTO)
    {
      VfdLedBuzzer.bLedRoller = LED_ON;
    }
    else
    {
      if(nRollerPWM == 1)
      {
        VfdLedBuzzer.bLedRoller = bSlowDisplayFlash;
      }
      if(nRollerPWM == 2)
      {
        VfdLedBuzzer.bLedRoller = bFastDisplayFlash;
      }
      if(nRollerPWM == 3)
      {
        VfdLedBuzzer.bLedRoller = LED_ON;
      }
    }
  }
  else
  {
    VfdLedBuzzer.bLedRoller = LED_OFF;
  }
}
*/
//10ms处理程序
void Main_100ms_Proce(void)
{

    if(w_KeyWalkHoldTimer != 0)
    {
        w_KeyWalkHoldTimer++;
        if((nChairRunState == CHAIR_STATE_WAIT_COMMAND) && (w_KeyWalkHoldTimer > 50))
        {
            //进入工程模式
            bDemoRun = 1;
        }
    }

    unsigned short adc24, adcVcc, adc24_1;
    st_Stretch.timer++;
    if(Power_Get() == 1)
    {
        ADC_Get_Voltage(ADC_V24, &adc24);
        ADC_Get_Voltage(ADC_V24_1, &adc24_1);

        if(adc24 > 3600)
        {
            nPowerMotorHighTime++;
        }
        else
        {
            nPowerMotorHighTime = 0;
        }

        if(adc24 < 1000)
        {
            nPowerMotorLowTime++;
        }
        else
        {
            nPowerMotorLowTime = 0;
        }

        if(adc24_1 > 3600)
        {
            nPowerValveHighTime++;
        }
        else
        {
            nPowerValveHighTime = 0;
        }

        if(adc24_1 < 1000)
        {
            nPowerValveLowTime++;
        }
        else
        {
            nPowerValveLowTime = 0;
        }
    }
    else
    {
        nPowerMotorHighTime = 0;
        nPowerMotorLowTime = 0;
        nPowerValveHighTime = 0;
        nPowerValveLowTime = 0;
    }

    ADC_Get_Voltage(ADC_VCC, &adcVcc);
    if(adcVcc > 550)
    {
        nPowerVCCHighTime++;
    }
    else
    {
        nPowerVCCHighTime = 0;
    }

    if(adcVcc < 450)
    {
        nPowerVCCLowTime++;
    }
    else
    {
        nPowerVCCLowTime = 0;
    }
    if(KneadMotor_IsRun())
    {
        nWidthOverTime++;
    }
    else
    {
        nWidthOverTime = 0;
    }
    if(WalkPower_Get() == WALK_MOTOR_POWER_ON)
    {
        if(Input_GetWalkChange())
        {
            nWalkOverTime = 0;
            Input_ClearWalkChange();
        }
        else
        {
            nWalkOverTime++;
        }
    }
    else
    {
        nWalkOverTime = 0;
    }
    if(BackMotor_GetPower() == BACK_MOTOR_POWER_ON)
    {
        nBackOverTime++;
    }
    else
    {
        nBackOverTime = 0;
    }

    if(FlexPower_Get() == FLEX_POWER_ON)
    {
        nFlexOverTime++;
    }
    else
    {
        nFlexOverTime = 0;
    }
    /*
    if(SlideMotor_GetPower() == SLIDE_MOTOR_POWER_ON)
    {
        nZeroOverTime++;
    }
    else
    {
        nZeroOverTime = 0;
    }
    */


    /********************************************/
    //按摩椅状态变换(不用频繁判断)
    if((nChairRunState == CHAIR_STATE_SETTLE) &&
            (bBackLegPadSettle != TRUE) &&
            (nBackMainRunMode != BACK_MAIN_MODE_SETTLE))
    {
      nChairRunState = CHAIR_STATE_IDLE ;
      nIndicateTimer = IDLE_INDICATE_TIME;
      //140623
      nCurBackMainRunModeStore = BACK_MAIN_MODE_IDLE ;
      nCurBackSubRunModeStore = BACK_SUB_MODE_NO_ACTION ;
      //nCurMaunalSubModeStore =  ;
      // Power_On();
    }
    //所有的功能被全部关闭后的状态转换
    if((nChairRunState == CHAIR_STATE_RUN) &&
            (nBackMainRunMode == BACK_MAIN_MODE_IDLE) &&
            (nKeyAirBagLocate == AIRBAG_LOCATE_NONE) &&
            (bKeyWaistHeat == FALSE) &&
            /*(bKeySeatVibrate == FALSE) &&*/
            bRollerEnable == FALSE)
    {
      nChairRunState = CHAIR_STATE_WAIT_COMMAND ;
      nChairStateCount = 0 ;
      nIndicateTimer = WAIT_INDICATE_TIME;
      //140530
      //w_PresetTimeStore = Data_Get_TimeSecond() ;
      //140623
      bKeyWaistHeatStore = FALSE ;
      nKeyAirBagLocateStore = AIRBAG_LOCATE_NONE ;
      
      Data_Set_Start(0, 0);
      // Power_On();
    }
    /********************fww**********************/
    if((nChairRunState == CHAIR_STATE_SETTLE)&&(nChairStateCount2 >= 240))
    {
          Power_All_Off();
          Main_Initial_IO();
          Main_Initial_Data();
          bBackLegPadSettle = FALSE;
          nChairRunState = CHAIR_STATE_IDLE;
    }
    /********************fww**********************/
    //WAIT_COMMAND超过给定的时间
    if((nChairRunState == CHAIR_STATE_WAIT_COMMAND/* || nChairRunState == CHAIR_STATE_SETTLE_1ST*/) &&
            (nChairStateCount >= MAX_WAIT_COMMAND_TIME))
    {
      /*if(nChairRunState == CHAIR_STATE_SETTLE_1ST)
      {
        bResetStateORchangeModeDis = ExitDisReset ;
      }*/
      nChairRunState = CHAIR_STATE_IDLE ;
      bKeyPowerSwitch = FALSE ;
      nIndicateTimer = IDLE_INDICATE_TIME;
    }
    //100ms变量更新
    //电机过流计数器
#ifdef SN_CHECK_EN
    nSendSNStepCount++ ;
#endif
    //动作持续时间
    nCurActionStepCounter++ ;
    nCurShoulderAdjustCounter++ ;
    nCurKnockRunStopCounter++ ;
    //振动记数器
    //nCurVibTime++ ;
    //气囊记数器
    st_AirBagAuto0.nAirBagCounter++;
    st_AirBagAuto1.nAirBagCounter++ ;
    st_AirBagAuto2.nAirBagCounter++ ;
    st_AirBagAuto3.nAirBagCounter++ ;
}

void Main_10ms_Proce(void)
{
    //揉捏电机状态计数器
    nCurKneadMotorStateCounter++ ;

    if((Data_Get_Time() == 0) && (nChairRunState == CHAIR_STATE_RUN))
    {
        bRunOverFlag = TRUE;
    }
    if(bRunOverFlag == TRUE)
    {
        bRunOverFlag = FALSE;
        RunOverStop();
        if(bErrorOverFlag == TRUE)
        {
            bErrorOverFlag = FALSE;
        }
    }
    //140530
    /*if((nChairRunState == CHAIR_STATE_SETTLE_1ST) && (nBuzzer100msFlag == FALSE))
    {
      nSendBuzzerTimes++ ;
      if(nSendBuzzerTimes <= 3)
      {
        nBuzzer100msFlag = TRUE ;
        nSendBuzzerTimes = 3 ;
      }
      else
      {
        nBuzzer100msFlag = FALSE ;
      }
    }
    else
    {
      nSendBuzzerTimes = 0 ;
    }*/
    
}

void  Main_500ms_Proce(void)
{
  ZeroNewCount = ~ZeroNewCount;
  nBackLegPadFlashCount++ ;
  if(nBackLegPadFlashCount >= 4)
  {
    nBackLegPadFlashCount = 0 ;
  }
  bDisplayFlash = ~bDisplayFlash ;
  nChairStateCount++ ;
  nChairStateCount2++;
  nChairRunDemoCount++;
  
  //140529
  /*************************fww****************************
  if((bResetStateORchangeModeDis == EnterDisReset) && 
     (bSelectKeyValue == KEYCONFIRM))
  {
    nWaitCustomActionTime++ ;
    if(nWaitCustomActionTime > 11)
    {
      nWaitCustomActionTime = 11 ;
    }
  }
  else 
  {
    nWaitCustomActionTime = 0 ;
  }
  ***********************fww**********************/
  //141116
  if((bKnockMotorPowerFlag == FALSE) && \
    (TIMER_CompareBufGet(KNOCK_MOTOR_TIMER, KNOCK_MOTOR_TIMER_CHANNEL) <= KNOCK_SPEED1_PWM))
  {
    nKneadBalanceCounter++;
    if(nKneadBalanceCounter > MAX_KNEADBALANCE_TIME_1MIN)//避免该变量溢出之后，给人感觉是无缘无故的重新开始检测该位置
    {
      nKneadBalanceCounter = MAX_KNEADBALANCE_TIME_1MIN;
    }
  }
  else
  {
    nKneadBalanceCounter = 0;
  }
}

void focredFinisih(void)
{
	bWalkMotorInProcess = FALSE;
	bKneadMotorInProcess = FALSE;
	bKnockMotorInProcess = FALSE;
}
#define STRETCH_GO_DOWN		0
#define STRETCH_GO_OUT		1

StretchProgramStruct const stretchProgram_30[] =
{
  {29,3,STRETCH_GO_OUT},
  {26,3,STRETCH_GO_DOWN},
  {23,3,STRETCH_GO_OUT},
  {20,3,STRETCH_GO_DOWN},
  {17,3,STRETCH_GO_OUT},
  {14,3,STRETCH_GO_DOWN},
  {11,3,STRETCH_GO_OUT},
  {8,3,STRETCH_GO_DOWN},
  {5,3,STRETCH_GO_OUT},
  {3,2,STRETCH_GO_DOWN},
};
StretchProgramStruct const stretchProgram_20[] =
{
  {19,3,STRETCH_GO_OUT},
  {16,3,STRETCH_GO_DOWN},
  {13,3,STRETCH_GO_OUT},
  {10,3,STRETCH_GO_DOWN},
  {7,3,STRETCH_GO_OUT},
  {5,3,STRETCH_GO_DOWN},
  {3,3,STRETCH_GO_OUT},
};
StretchProgramStruct const stretchProgram_10[] =
{
  {9,4,STRETCH_GO_OUT},
  {6,3,STRETCH_GO_DOWN},
  {3,3,STRETCH_GO_OUT},
};

void Valve_StretchControlProce_US047(void)
{
	bool bStatus;
	int legFlag,BackFlag/*,FlexFlag*/;
  
	if(!st_Stretch.active) 
	{
		unsigned int RunTime = Data_Get_TimeSecond();
		unsigned int Minutes,i;
		StretchProgramStruct const *p;
		unsigned int totalTimes;
    
		if(RunTime%60 != 0)  return; //0秒开始拉退
        
		if(w_PresetTime == RUN_TIME_10) 
		{
			p = stretchProgram_10;
			totalTimes = sizeof(stretchProgram_10)/sizeof(StretchProgramStruct);
		}
		else if(w_PresetTime == RUN_TIME_30) 
		{
			p = stretchProgram_30;
			totalTimes = sizeof(stretchProgram_30)/sizeof(StretchProgramStruct);
		}
		else
		{
			p = stretchProgram_20;
			totalTimes = sizeof(stretchProgram_20)/sizeof(StretchProgramStruct);
		} 
		Minutes = RunTime/60; //获取当前分钟数
   
		if(Minutes == 0) 
		{
			st_Stretch.times = 0;
			return; //最后一分钟停止拉退
		}
    
		for(i=0;i<totalTimes;i++)
		{
			if(Minutes == (p+i)->time) 
			{
				st_Stretch.active = TRUE;
				st_Stretch.init = TRUE; 
				stretchMode = (p+i)->mode;
				st_Stretch.times = (p+i)->times;
				break;
			}
		}
		if(!st_Stretch.active)  return;
	}
	if(st_Stretch.init)
	{
		//printf("init\n"); 
		st_Stretch.step = 0;
		st_Stretch.timer = 0;
		//st_Stretch.times = 3;
		st_Stretch.init = FALSE;
	}
	if(st_Stretch.times > 0)
	{
		switch(st_Stretch.step)
		{
			case  0:  //按摩椅到最大位置
				Valve_SetStretchUp();  
				//SlideFlag = SlideMotorControl(STATE_RUN_SLIDE_FORWARD); 
				//if(SlideFlag) //前滑电动缸必须先滑到前面
				{
					//clear accident flag 140707
					//Clear_Accident_flag();
					legFlag = LegMotor_Control(STATE_RUN_LEG_UP);
					BackFlag = BackMotor_Control(STATE_RUN_BACK_DOWN);
					//FlexFlag = FlexMotor_Control(STATE_RUN_FLEX_RESET,FLEX_SPEED_FAST,FLEX_CURRENT_3A);
					if(legFlag && BackFlag /*&& FlexFlag == FLEX_STOP_AT_IN*/)
					{
						st_Stretch.step++;
						st_Stretch.timer = 0; 
					}
				}
				break;
			case 1:  //调整电动小腿位置
				if(stretchMode == STRETCH_GO_OUT)
				{
					st_Stretch.step++;
					break;
				}
				//FlexMotorSetEnable(); //fww
				if(st_Stretch.timer > 10) 
					st_Stretch.step++;
				break;
			case  2: 
				//靠背不动，小腿不动，腿侧和足侧充气 持续时间为一直到小腿下降到最低点 
				nStretchStep = 1;
			//	focredFinisih();
				Valve_SetStretchCharge(1); 
				if(bRollerEnable)
				{
					RollerMotor_Control(ROLLER_SPEED_SLOW,0);
				}
				st_Stretch.step++;
				st_Stretch.timer = 0;
				break;
			case  3: 
				Valve_SetStretchCharge(0); 
				if(st_Stretch.timer >= C_STRETCH_CHARGE_TIME)
				{	//判断是否已达充气时间
					st_Stretch.step++;
					st_Stretch.timer = 0;
					nStretchStep = 2;
				//	focredFinisih();
				}
				break;
			case 4: 
				Valve_SetStretchCharge(0);
				if(stretchMode == STRETCH_GO_DOWN)
				{
					LegMotor_Control(STATE_RUN_LEG_DOWN);
					//靠行程开关控制
					if(Input_GetLegDownSwitch())
					{
						bStatus = 1;
					}
					else
					{
						bStatus = 0;
					}
				}
				else
				{
					//FlexFlag = FlexMotor_Control(STATE_RUN_FLEX_MANUAL_OUT,FLEX_SPEED_MID,FLEX_CURRENT_DRAG);
					if(/*FlexFlag ||*/ st_Stretch.timer > 100)
					{
						bStatus = 1;
					}
					else
					{
						bStatus = 0;
					}
				}
      
				if(bStatus)
				{  
					if(bRollerEnable)
					{
						RollerMotor_Control(ROLLER_SPEED_SLOW,0);
					}
					//FlexMotor_Control(STATE_FLEX_IDLE,FLEX_SPEED_FAST,FLEX_CURRENT_3A);
					//FlexMotor_ResetTime = 0;
					st_Stretch.step++;
					st_Stretch.timer = 0;
					nStretchStep = 3;
				//	focredFinisih();
				}
				break;
			case 5:    //加压时间
				Valve_SetStretchHold();
				st_Stretch.step++;
				st_Stretch.timer = 0;
				break;
			case 6:
				if(st_Stretch.timer >= (Valve_GetAirBagStrength()*10))
				{  //判断是否已达加压时间
					st_Stretch.step++;
					st_Stretch.timer = 0;
					RollerMotor_Control(ROLLER_SPEED_STOP,0);
				}
				break;
			case 7:
				st_Stretch.step = 0;
				st_Stretch.timer = 0;
				st_Stretch.times--;
				Valve_SetStretchUp();  
				if(st_Stretch.times == 0)
				{
					// nZLB_RunState = 1;            //拉退动作完成强制回到第一个零重力点
					nTargetMassagePosition = MASSAGE_OPTIMAL_POSITION; 
					bMassagePositionUpdate = TRUE;
					//bZLBMotorRunFlag = TRUE;
					st_Stretch.bBackLegFlag = FALSE;
					st_Stretch.timer = 0;
					st_Stretch.active = FALSE;
				}
				break;
			default:
				break;
		}
	}   
}

void Valve_StretchControlProce(void)
{
	static int mode = STRETCH_GO_OUT;
	bool bStatus;
	int legFlag, BackFlag/*, FlexFlag*/;
	if(!st_Stretch.active)
	{
		unsigned int RunTime = Data_Get_TimeSecond();
		unsigned int Minutes;
		if(RunTime % 60 == 1)
		{
			Minutes = RunTime / 60;
			if(Minutes <= 3) return; //最后三分钟不拉退
			// if(Minutes == w_Run_Time) return; //最后三分钟不拉退
			if((Minutes % 3) == 1)
			{
				st_Stretch.active = TRUE;
				st_Stretch.init = TRUE;
				//141010
				mode = STRETCH_GO_DOWN;
				focredFinisih();
			}
		}
		return;
	}
	if(st_Stretch.init)
	{
		//printf("init\n");
		st_Stretch.step = 0;
		st_Stretch.timer = 0;
		st_Stretch.times = 3;
		st_Stretch.init = FALSE;
		nTargetMassagePosition = MASSAGE_UNKNOW_POSITION;
	}
	if(st_Stretch.times > 0)
	{
		switch(st_Stretch.step)
		{
			case  0:  //按摩椅到最大位置
				nStretchStep = 0;//fww
				Valve_SetStretchUp();
				//SlideFlag = SlideMotorControl(STATE_RUN_SLIDE_FORWARD);
				//if(SlideFlag) //前滑电动缸必须先滑到前面
				{
					//140704
					//Clear_Accident_flag() ;
					legFlag = LegMotor_Control(STATE_RUN_LEG_UP);
					BackFlag = BackMotor_Control(STATE_RUN_BACK_DOWN);
					//FlexFlag = FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
					if(legFlag && BackFlag /*&& FlexFlag == FLEX_STOP_AT_IN*/)
					{
						st_Stretch.step ++;
						st_Stretch.timer = 0;
					}
				}
				break;
          
			case 1:  //调整电动小腿位置
				if(mode == STRETCH_GO_OUT)
				{
					st_Stretch.step++;
					break;
				}
				nStretchStep = 0;//fww
				// FlexMotorSetEnable();//fww
				if(st_Stretch.timer > 10)
					st_Stretch.step++;
				break;
			case  2:
				//靠背不动，小腿不动，腿侧和足侧充气 持续时间为一直到小腿下降到最低点
				nStretchStep = 0;//fww
			//	focredFinisih();
				Valve_SetStretchCharge(1);
				if(bRollerEnable)
				{
					RollerMotor_Control(ROLLER_SPEED_SLOW, 0);
				}
				st_Stretch.step++;
				st_Stretch.timer = 0;
				break;
			case  3:
				Valve_SetStretchCharge(0);
				if(mode == STRETCH_GO_OUT)
				{
					if(st_Stretch.timer >= C_STRETCH_CHARGE_TIME)
					{
						//判断是否已达充气时间
						st_Stretch.step++;
						st_Stretch.timer = 0;
						nStretchStep = 2;
					//	focredFinisih();
					}
				}
				else
				{
					switch(st_Stretch.times)
					{
						case 1:
							if(st_Stretch.timer >= C_STRETCH_CHARGE_TIME_NORMAL)
							{
								//判断是否已达充气时间
								st_Stretch.step++;
								st_Stretch.timer = 0;
								nStretchStep = 2;
								//focredFinisih();
							}
							break;
						case 2:
							if(st_Stretch.timer >= C_STRETCH_CHARGE_TIME_NORMAL + 20)
							{
								//判断是否已达充气时间
								st_Stretch.step++;
								st_Stretch.timer = 0;
								nStretchStep = 2;
								//focredFinisih();
							}
							break;
						case 3:
							if(st_Stretch.timer >= C_STRETCH_CHARGE_TIME_NORMAL + 40)
							{
								//判断是否已达充气时间
								st_Stretch.step++;
								st_Stretch.timer = 0;
								nStretchStep = 2;
								//focredFinisih();
							}
							break;
						default:
							if(st_Stretch.timer >= C_STRETCH_CHARGE_TIME_NORMAL)
							{
								//判断是否已达充气时间
								st_Stretch.step++;
								st_Stretch.timer = 0;
								nStretchStep = 2;
								//focredFinisih();
							}
							break;
					}
				}
				break;
			case 4:
				Valve_SetStretchCharge(0);
          
				if(mode == STRETCH_GO_DOWN)
				{
					LegMotor_Control(STATE_RUN_LEG_DOWN);
					//靠行程开关控制
					if(Input_GetLegDownSwitch())
					{
						bStatus = 1;
					}
					else
					{
						bStatus = 0;
					}
				}
				else
				{
					//FlexFlag = FlexMotor_Control(STATE_RUN_FLEX_MANUAL_OUT, FLEX_SPEED_MID, FLEX_CURRENT_DRAG);
					if(/*FlexFlag ||*/ st_Stretch.timer > 100)
					{
						bStatus = 1;
					}
					else
					{
						bStatus = 0;
					}
				}
          
				if(bStatus)
				{
					if(bRollerEnable)
					{
						RollerMotor_Control(ROLLER_SPEED_SLOW, 0);
					}
					//FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
					//FlexMotor_ResetTime = 0;
					st_Stretch.step++;
					st_Stretch.timer = 0;
					nStretchStep = 3;
					// focredFinisih();
				}
				break;
			case 5:    //加压时间
				Valve_SetStretchHold();
				st_Stretch.step++;
				st_Stretch.timer = 0;
				break;
			case 6:
				if(st_Stretch.timer >= (Valve_GetAirBagStrength() * 25))
				{
					//判断是否已达加压时间
					st_Stretch.step++;
					st_Stretch.timer = 0;
					RollerMotor_Control(ROLLER_SPEED_STOP, 0);
				}
				break;
			case 7:
				st_Stretch.step = 0;
				st_Stretch.timer = 0;
				st_Stretch.times--;
				Valve_SetStretchUp();
				if(st_Stretch.times == 0)
				{
 					// nZLB_RunState = 1;            //拉退动作完成强制回到第一个零重力点
					nTargetMassagePosition = MASSAGE_OPTIMAL2_POSITION;
					bMassagePositionUpdate = TRUE;
					//bZLBMotorRunFlag = TRUE;
					st_Stretch.bBackLegFlag = FALSE;
					st_Stretch.timer = 0;
					st_Stretch.active = FALSE;
				}
				break;
			default:
				break;
		}
	}
	// else
	//    if((st_Stretch.timer >= C_STRETCH_RESET_TIME) || Input_GetBackUpSwitch())
	// if(bZLBMotorRunFlag == FALSE)
	/*
	{    //判断是否已达复位时间
	st_Stretch.step++;
	st_Stretch.timer = 0;
	st_Stretch.active = FALSE;
	st_Stretch.bBackLegFlag = FALSE;
	//printf("step8\n");
	}
	*/
}

void Main_Valve_Proce(void)
{
	//140529
	//if(bResetStateORchangeModeDis == EnterDisReset) return ;
    
	if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
	{
		Valve_SetBackMode(1);
	}
	else
	{
		Valve_SetBackMode(0);
	}

	unsigned char by_EngineeringAirBag = ReadEEByte(AIRBAG_STRETCH_ADDRESS + USER_DATA_BASE);

	if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
	{
		if((Data_Get_ProgramExecTime() > VALVE_START_TIME) || !bMassagePositionUpdate )  //判断气阀开始时间是否到达
		{
			goto VALVE_START;  //如果按摩椅已经到达最佳位置
		}
		// Valve_Control(VALVE_DISABLE,&st_AirBagAuto,by_EngineeringAirBag);

		Valve_Control(VALVE_DISABLE, &st_AirBagAuto0, by_EngineeringAirBag);
		Valve_Control(VALVE_DISABLE, &st_AirBagAuto1, by_EngineeringAirBag);
		Valve_Control(VALVE_DISABLE, &st_AirBagAuto2, by_EngineeringAirBag);
		Valve_Control(VALVE_DISABLE, &st_AirBagAuto3, by_EngineeringAirBag);
		Valve_LegFootAirPumpACPowerOff();
		Valve_FootRollerProce(0, 0, &st_AirBagAuto0);
		return;
	}

	VALVE_START:

	if(nBackMainRunMode == BACK_MAIN_MODE_AUTO)
	{
		Valve_SetEnableSholder(0);
	}
	else
	{
		Valve_SetEnableSholder(1);
	}

    /*
     if(!st_Stretch.active)
     {
       st_Stretch.bBackLegFlag = FALSE;
     }
    */
	if(nKeyAirBagLocate == AIRBAG_LOCATE_AUTO)
	{
		if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) && ( nBackSubRunMode == BACK_SUB_MODE_AUTO_1))
		{
          /**************************fww********************************
            bool us047_enable = ReadEEByte(USER_DATA_BASE + STRETCH_OUT_ADDRESS);
            if(us047_enable)
            { 
              Valve_StretchControlProce_US047();  //执行拉退，气囊由算法控制
            }
            else
            { 
              Valve_StretchControlProce();  //执行拉退，气囊由算法控制
            }
         **************************fww************************/
			Valve_StretchControlProce();  //执行拉退，气囊由算法控制  //fww
			if(st_Stretch.active == TRUE)
			{
				//Valve_Control(VALVE_ENABLE, &st_AirBagArm, 1); //在拉退时刻，由数组控制手臂气囊程序  //FWW
			}
			else
			{
				Valve_Control(VALVE_ENABLE, &st_AirBagAuto0, by_EngineeringAirBag);
				Valve_FootRollerProce(bRollerEnable, 1, &st_AirBagAuto0);
			}
		}
		else
		{
			if(st_Stretch.active)
			{
 				st_Stretch.active = FALSE;
				st_Stretch.init = FALSE;
				bKeyLegPadUp = FALSE ;
				bKeyLegPadDown = FALSE ;
				bLegPadLinkage = FALSE ;
				bKeyBackPadUp = FALSE ;
				bKeyBackPadDown = FALSE ;
			}
			Valve_Control(VALVE_ENABLE, &st_AirBagAuto0, by_EngineeringAirBag);
			Valve_FootRollerProce(bRollerEnable, 1, &st_AirBagAuto0);
		}
	}
	else
	{
		if(st_Stretch.active)
		{
			st_Stretch.active = FALSE;
			st_Stretch.init = FALSE;
			bKeyLegPadUp = FALSE ;
			bKeyLegPadDown = FALSE ;
			bLegPadLinkage = FALSE ;
			bKeyBackPadUp = FALSE ;
			bKeyBackPadDown = FALSE ;
		}

		Valve_FootRollerProce(bRollerEnable, 0, &st_AirBagAuto3);
		switch(nKeyAirBagLocate)
		{
			case  AIRBAG_LOCATE_NONE:
				Valve_Control(VALVE_DISABLE, &st_AirBagAuto0, by_EngineeringAirBag);
				Valve_Control(VALVE_DISABLE, &st_AirBagAuto1, by_EngineeringAirBag);
				Valve_Control(VALVE_DISABLE, &st_AirBagAuto2, by_EngineeringAirBag);
				Valve_Control(VALVE_DISABLE, &st_AirBagAuto3, by_EngineeringAirBag);
				Valve_LegFootAirPumpACPowerOff();
				break;
			case  AIRBAG_LOCATE_LEG_FOOT:
				//Valve_Control(VALVE_DISABLE, &st_AirBagAuto0, by_EngineeringAirBag);
				//Valve_Control(VALVE_DISABLE, &st_AirBagAuto1, by_EngineeringAirBag);
				//Valve_Control(VALVE_DISABLE, &st_AirBagAuto2, by_EngineeringAirBag);
				Valve_Control(VALVE_ENABLE,  &st_AirBagAuto3, by_EngineeringAirBag);
				break;
			case AIRBAG_LOCATE_SEAT:
				//Valve_Control(VALVE_DISABLE, &st_AirBagAuto0, by_EngineeringAirBag);
				//Valve_Control(VALVE_DISABLE, &st_AirBagAuto1, by_EngineeringAirBag);
				//Valve_Control(VALVE_DISABLE, &st_AirBagAuto3, by_EngineeringAirBag);
				Valve_Control(VALVE_ENABLE,  &st_AirBagAuto2, by_EngineeringAirBag);
				break;
			case AIRBAG_LOCATE_ARM_SHOLDER:
				//Valve_Control(VALVE_DISABLE, &st_AirBagAuto0, by_EngineeringAirBag);
				//Valve_Control(VALVE_DISABLE, &st_AirBagAuto2, by_EngineeringAirBag);
				//Valve_Control(VALVE_DISABLE, &st_AirBagAuto3, by_EngineeringAirBag);
				Valve_Control(VALVE_ENABLE,  &st_AirBagAuto1, by_EngineeringAirBag);
				break;
		}
	}
}

void startBodyDetect(void)
{
    //往上走
    nFinalWalkMotorLocate = TOP_POSITION;
    //肩部检测清空
    bodyDetectSuccess = 0;
    //检测步骤清零
    shoulderPositionScanStep = 0;
    //20181019
    bDisplayDetect = 1;
}

void Main_BackProce(void)
{
    //140529
    //if(bResetStateORchangeModeDis == EnterDisReset) return ;
  
    switch(nBackMainRunMode)
    {
    case BACK_MAIN_MODE_SETTLE://每次停机复位时只需要将nBackSettleStep置0即可
        if((nBackSettleReason == PARK_KEY_STOP) || (nBackSettleReason == PARK_MANUAL_OVER))
        {
            switch(nBackSettleStep)
            {
            case 0:
                nKneadMotorControlParam1 = KNEAD_STOP_AT_MAX ;
                nKneadMotorControlParam2 = 0 ;
                bKneadMotorInProcess = TRUE ;
                nKnockMotorControlParam1 = KNOCK_STOP ;
                nKnockMotorControlParam2 = 0 ;
                bKnockMotorInProcess = TRUE ;
                nWalkMotorControlParam1	= WALK_LOCATE_PARK ;
                nWalkMotorControlParam2 = 0 ;
                bWalkMotorInProcess = TRUE ;
                bUpdateLocate = TRUE ;
                nBackSettleStep = 1 ;
                break ;
            case 1:
                if((bKneadMotorInProcess == FALSE) && (bKnockMotorInProcess == FALSE))
                {
                    nWalkMotorControlParam1	= WALK_LOCATE_TOP ;
                    nWalkMotorControlParam2 = 0 ;
                    bWalkMotorInProcess = TRUE ;
                    bUpdateLocate = TRUE ;
                    nBackSettleStep = 2 ;
                }
                break ;
            case 2:
                if(bWalkMotorInProcess == FALSE)
                {
                    nWalkMotorControlParam1	= WALK_LOCATE_ABSULATE ;
#ifdef SINGLE_HEADER
                    nWalkMotorControlParam2 = Input_GetWalkMotorPosition() - 10 ;
#else
                    nWalkMotorControlParam2 = Input_GetWalkMotorPosition() - 25 ;
#endif
                    bWalkMotorInProcess = TRUE ;
                    bUpdateLocate = TRUE ;
                    nBackSettleStep = 3 ;
                }
                break ;
            case 3:
                if(bWalkMotorInProcess == FALSE)
                {
                    nWalkMotorControlParam1	= WALK_LOCATE_PARK ;
                    nWalkMotorControlParam2 = 0 ;
                    bWalkMotorInProcess = TRUE ;
                    bUpdateLocate = TRUE ;
                    nBackSettleStep = 4 ;
                }
                break ;
            case 4:
                if(bWalkMotorInProcess == FALSE)
                {
                    nBackMainRunMode = BACK_MAIN_MODE_IDLE ;
                }
                break ;
            default:
                nBackMainRunMode = BACK_MAIN_MODE_IDLE ;
                break ;
            }
        }
        else
        {
            switch(nBackSettleStep)
            {
            case 0:
                nCurActionStepCounter = 0 ;//当前动作时间计数（行走或敲击时间计数）
                nCurKneadMotorCycles = 0 ;//揉捏圈数计数（揉捏）
                nKneadMotorControlParam1 = KNEAD_STOP_AT_MIN ;
                nKneadMotorControlParam2 = 0 ;
                bKneadMotorInProcess = TRUE ;
                nKnockMotorControlParam1 = KNOCK_STOP ;
                nKnockMotorControlParam2 = 0 ;
                bKnockMotorInProcess = TRUE ;
                nWalkMotorControlParam1	= WALK_LOCATE_TOP ;
                nWalkMotorControlParam2 = 0 ;
                bWalkMotorInProcess = TRUE ;
                bUpdateLocate = TRUE ;
                nBackSettleStep = 1 ;
                break ;
            case 1:
                if((bWalkMotorInProcess == FALSE) &&
                        (bKneadMotorInProcess == FALSE) &&
                        (bKnockMotorInProcess == FALSE))
                {
                    nCurActionStepCounter = 0 ;//当前动作时间计数（行走或敲击时间计数）
                    nCurKneadMotorCycles = 0 ;//揉捏圈数计数（揉捏）
                    nKneadMotorControlParam1 = KNEAD_STOP ;
                    nKneadMotorControlParam2 = 0 ;
                    bKneadMotorInProcess = TRUE ;
                    nKnockMotorControlParam1 = KNOCK_STOP ;
                    nKnockMotorControlParam2 = 0 ;
                    bKnockMotorInProcess = TRUE ;
                    nWalkMotorControlParam1	= WALK_LOCATE_ABSULATE ;
#ifdef SINGLE_HEADER
                    nWalkMotorControlParam2 = Input_GetWalkMotorPosition() - 10 ;
#else
                    nWalkMotorControlParam2 = Input_GetWalkMotorPosition() - 25 ;
#endif
                    bWalkMotorInProcess = TRUE ;
                    bUpdateLocate = TRUE ;
                    nBackSettleStep = 2 ;
                }
                break ;
            case 2:
                if((bWalkMotorInProcess == FALSE) &&
                        (bKneadMotorInProcess == FALSE) &&
                        (bKnockMotorInProcess == FALSE))
                {
                    nCurActionStepCounter = 0 ;//当前动作时间计数（行走或敲击时间计数）
                    nCurKneadMotorCycles = 0 ;//揉捏圈数计数（揉捏）
                    nKneadMotorControlParam1 = KNEAD_STOP ;
                    nKneadMotorControlParam2 = 0 ;
                    bKneadMotorInProcess = TRUE ;
                    nKnockMotorControlParam1 = KNOCK_STOP ;
                    nKnockMotorControlParam2 = 0 ;
                    bKnockMotorInProcess = TRUE ;
                    nWalkMotorControlParam1	= WALK_LOCATE_TOP ;
                    nWalkMotorControlParam2 = 0 ;
                    bWalkMotorInProcess = TRUE ;
                    bUpdateLocate = TRUE ;
                    nBackSettleStep = 3 ;
                }
                break ;
            case 3:
                if((bWalkMotorInProcess == FALSE) &&
                        (bKneadMotorInProcess == FALSE) &&
                        (bKnockMotorInProcess == FALSE))
                {
                    nCurActionStepCounter = 0 ;//当前动作时间计数（行走或敲击时间计数）
                    nCurKneadMotorCycles = 0 ;//揉捏圈数计数（揉捏）
                    nKneadMotorControlParam1 = KNEAD_STOP_AT_MAX ;
                    nKneadMotorControlParam2 = 0 ;
                    bKneadMotorInProcess = TRUE ;
                    nKnockMotorControlParam1 = KNOCK_STOP ;
                    nKnockMotorControlParam2 = 0 ;
                    bKnockMotorInProcess = TRUE ;
                    nWalkMotorControlParam1	= WALK_LOCATE_PARK ;
                    nWalkMotorControlParam2 = 0;
                    bWalkMotorInProcess = TRUE ;
                    bUpdateLocate = TRUE ;
                    nBackSettleStep = 4 ;
                }
                break ;
            case 4:
                if((bWalkMotorInProcess == FALSE) &&
                        (bKneadMotorInProcess == FALSE) &&
                        (bKnockMotorInProcess == FALSE))
                {
                    nCurActionStepCounter = 0 ;//当前动作时间计数（行走或敲击时间计数）
                    nCurKneadMotorCycles = 0 ;//揉捏圈数计数（揉捏）
                    nWalkMotorControlParam1	= WALK_LOCATE_ABSULATE ;
#ifdef SINGLE_HEADER
                    nWalkMotorControlParam2 = Input_GetWalkMotorPosition() - 10 ;
#else
                    nWalkMotorControlParam2 = Input_GetWalkMotorPosition() - 25 ;
#endif
                    bWalkMotorInProcess = TRUE ;
                    bUpdateLocate = TRUE ;
                    nBackSettleStep = 5 ;
                }
                break ;
            case 5:
                if(bWalkMotorInProcess == FALSE)
                {
                    nWalkMotorControlParam1	= WALK_LOCATE_PARK ;
                    nWalkMotorControlParam2 = 0 ;
                    bWalkMotorInProcess = TRUE ;
                    bUpdateLocate = TRUE ;
                    nBackSettleStep = 6 ;
                }
                break ;
            case 6:
                if(bWalkMotorInProcess == FALSE)
                {
                    nBackSettleStep = 7 ;
                }
                break ;
            case 7:
                nBackMainRunMode = BACK_MAIN_MODE_IDLE ;
                break ;
            }
        }
        break ;
		case BACK_MAIN_MODE_AUTO:
			if(bBackAutoModeInit == TRUE)
			{
				bBackAutoModeInit = FALSE ;
				nMaxActionStep = BACK_AUTO_STEPS[nBackSubRunMode] ;
				nStartActionStep = BACK_AUTO_START_STEP[nBackSubRunMode] ;
				bGetNextActionStep = TRUE ;
				nCurActionStep = 0 ;
				nDetectStep = 0;
				nStretchStep = 0;
				//如果不等当前的动作完成，而强行将InProcess置成FALSE，会造成冲顶
			}
			else
			{
            /*
            if(nCurSubFunction == BACK_SUB_MODE_BODY_DETECT_0
               &&(bKeyWalkUp == TRUE||bKeyWalkDown == TRUE))
            {
              //切换手动调整
              AutoDirector = ManualShoulderAdjust[0];
              //printf("m\n");
              refreshAutoDirector();
            }
            //肩部微调结束(Finish)：记录当前肩部位置的准确数据
            else
            */
				if(nCurSubFunction == BACK_SUB_MODE_BODY_DETECT_1)
				{

					if(bKeyWalkUp == TRUE || bKeyWalkDown == TRUE)
					{
						nCurShoulderAdjustCounter = 0;
					}

                /*
                if(bodyDetectSuccess){
                  //printf("B\n");
                  //体型检测成功
                  bBodyDetectSuccess = TRUE ;
                  //继续下个动作
                  bGetNextActionStep = TRUE ;
                }
                else
                */
					if(nCurShoulderAdjustCounter >= MAX_SHOULDER_ADJUST_TIME)
					{
						//肩部微调结束时，记录新肩部位置
						nShoulderPosition = Input_GetWalkMotorPosition() ;
						if(nFinalWalkMotorLocate < LIMIT_POSITION)
						{
							nFinalWalkMotorLocate = LIMIT_POSITION ;
							bodyDetectSuccess = 0;
						}
						else if(nFinalWalkMotorLocate > TOP_POSITION )
						{
							nFinalWalkMotorLocate = DEFAULT_SHOULDER_POSITION;
							bodyDetectSuccess = 0;
						}
						bFail = ~bodyDetectSuccess;
						//更新肩部微调相关数据
						BodyDataRefresh() ;
						//结束蜂鸣器
						if(bKeyWalkUp == TRUE)
						{
							bKeyWalkUp = FALSE ;
							nBuzzerMode = BUZZER_MODE_OFF ;
							bSendBuzzerMode = TRUE ;
							bBlueToothSendBuzzerMode = TRUE;
						}
						if(bKeyWalkDown == TRUE)
						{
							bKeyWalkDown = FALSE ;
							nBuzzerMode = BUZZER_MODE_OFF ;
							bSendBuzzerMode = TRUE ;
							bBlueToothSendBuzzerMode = TRUE;
						}
						//体型检测成功
						bBodyDetectSuccess = TRUE ;
                                                //20181019
                                                bDisplayDetect = 0;
						//继续下个动作
                    /*
                    nCurActionStep++ ;
                    if(nCurActionStep >= nMaxActionStep)
                    {
                    nCurActionStep = nStartActionStep ;
                    }
                    */
						bGetNextActionStep = TRUE ;
					}
				}
				else if((bWalkMotorInProcess == FALSE) && (bKneadMotorInProcess == FALSE) && (bKnockMotorInProcess == FALSE))
				{
					//printf("[%d]\n",nCurActionStep);
					nCurActionStep++ ; //自动程序步骤增加
					if(nCurActionStep >= nMaxActionStep)
					{
							nCurActionStep = nStartActionStep ;
					}
					bGetNextActionStep = TRUE ;
				}
			}
			if(bGetNextActionStep == TRUE)
			{
				bGetNextActionStep = FALSE ;
				// if(nReworkShoulderPosition == 2)   //开始检测
				if(nReworkShoulderPosition == 2 && nBackSubRunMode != BACK_SUB_MODE_AUTO_5)   //开始检测
				{
					//初始化检测参数检测
					if(nDetectStep == 0)   startBodyDetect();
					AutoDirector = AutoFunctionDetect[nDetectStep] ;
					nDetectStep++;
					// if(nDetectStep>2)
					if(nDetectStep >= (sizeof(AutoFunctionDetect) / sizeof(struct Walk_Knead_Knock_Motor_Struct_Auto)))
					{
						nDetectStep = 0;
						nReworkShoulderPosition = 0;
					}
				}
				else if(st_Stretch.active)
				{
					AutoDirector = AutoFunctionStretch[nStretchStep] ;
				}
				else
				{
					do
					{
						switch(nBackSubRunMode)
						{
							case BACK_SUB_MODE_AUTO_0:
								AutoDirector = AutoFunction0[nCurActionStep] ;
								break ;
							case BACK_SUB_MODE_AUTO_1:
								AutoDirector = AutoFunction1[nCurActionStep] ;
								break ;
							case BACK_SUB_MODE_AUTO_2:
								AutoDirector = AutoFunction2[nCurActionStep] ;
								break ;
							case BACK_SUB_MODE_AUTO_3:
								AutoDirector = AutoFunction3[nCurActionStep] ;
								break ;
							case BACK_SUB_MODE_AUTO_4:
								AutoDirector = AutoFunction4[nCurActionStep] ;
								break ;
							case BACK_SUB_MODE_AUTO_5:
								AutoDirector = AutoFunction5[nCurActionStep] ;
								break ;
						}
						if(((AutoDirector.nSubFunction == BACK_SUB_MODE_BODY_DETECT_0) ||
							(AutoDirector.nSubFunction == BACK_SUB_MODE_BODY_DETECT_1)) &&
							(bBodyDetectSuccess == TRUE)) //跳过肩膀位置检测程序
						{
							nCurActionStep++ ;
							if(nCurActionStep >= nMaxActionStep)
							{
								nCurActionStep = nStartActionStep ;
							}
						}
						else
						{
							break ;
						}
					}
					while(TRUE) ;
				}
				//每次更换动作需要更新的变量
				nCurActionStepCounter = 0 ;//当前动作时间计数（行走或敲击时间计数）
				nCurShoulderAdjustCounter = 0 ;
				if(!((nCurSubFunction == BACK_SUB_MODE_SOFT_KNOCK) && (AutoDirector.nSubFunction == BACK_SUB_MODE_SOFT_KNOCK)))
				{
					nCurKnockRunStopCounter = 0 ;//叩击动作记数器
				}
				nCurKneadMotorCycles = 0 ;//揉捏圈数计数（揉捏）
				refreshAutoDirector();
			}
			break ;
    case BACK_MAIN_MODE_MANUAL:
        if(bBackManualModeInit == TRUE)
        {
            bBackManualModeInit = FALSE ;
            bGetNextActionStep = TRUE ;
            nCurActionStep = 0 ;
        }
        else
        {
            if((bWalkMotorInProcess == FALSE) &&
                    (bKneadMotorInProcess == FALSE) &&
                    (bKnockMotorInProcess == FALSE))
            {
                nCurActionStep++ ;
                if(nCurActionStep >= nMaxActionStep)
                {
                    nCurActionStep = nStartActionStep ;
                }
                bGetNextActionStep = TRUE ;
                //bCurActionStepChange = TRUE ;//ONLY FOR TEST
            }
        }
        if(bGetNextActionStep == TRUE)
        {
            bGetNextActionStep = FALSE ;
            //每次更换动作需要更新的变量
            nCurActionStepCounter = 0 ;//当前动作时间计数（行走或敲击时间计数）
            nCurKnockRunStopCounter = 0 ;//叩击动作记数器
            nCurKneadMotorCycles = 0 ;//揉捏圈数计数（揉捏）
            nCurSubFunction = ManualDirector[nCurActionStep].nSubFunction ;
            nCurKneadKnockSpeed = ManualDirector[nCurActionStep].nKneadKnockSpeed ;
            //nKeyKneadKnockSpeed = ManualDirector[nCurActionStep].nKneadKnockSpeed ;
            //设置行走电机
            bWalkMotorInProcess = TRUE ;
            bUpdateLocate = TRUE ;
            nWalkMotorControlParam1 = ManualDirector[nCurActionStep].nWalkMotorLocateMethod ;
            nWalkMotorControlParam2 = ManualDirector[nCurActionStep].nWalkMotorLocateParam ;
            //设置揉捏电机
            bKneadMotorInProcess = TRUE ;
            nKneadMotorControlParam1 = ManualDirector[nCurActionStep].nKneadMotorState ;
            nKneadMotorControlParam2 = ManualDirector[nCurActionStep].nKneadMotorCycles ;
            //设置捶击电机
            bKnockMotorInProcess = TRUE ;
            nKnockMotorControlParam1 = ManualDirector[nCurActionStep].nKnockMotorState ;
            nKnockMotorControlParam2 = ManualDirector[nCurActionStep].nKnockMotorRunTime ;
            nKnockMotorControlParam3 = ManualDirector[nCurActionStep].nKnockMotorStopTime ;
        }
        break ;
    }
}


void Main_Walk_Beep_Proce(void)
{
  //if(bResetStateORchangeModeDis == EnterDisReset) return;
    if(bKeyWalkUp == TRUE)
    {
        if((nBackMainRunMode == BACK_MAIN_MODE_MANUAL) && (nKeyBackLocate == LOCATE_POINT))
        {
            //行走电机的行程信号只有在继电器完全稳定后才能检测
            //if(bReachWalkUpLimitFlag == TRUE)
            if(Input_GetWalkUpSwitch() == REACH_WALK_LIMIT)
            {
                //设置连续蜂鸣器声音
                nBuzzerMode = BUZZER_MODE_FAST ;
                bSendBuzzerMode = TRUE ;
                bBlueToothSendBuzzerMode = TRUE;
            }
            else
                //{
                //  if(nCurWalkMotorState == STATE_RUN_ANTICLOCK)
            {
                nBuzzerMode = BUZZER_MODE_SLOW ;
                bSendBuzzerMode = TRUE ;
                bBlueToothSendBuzzerMode = TRUE;
            }
            //}
        }
        else if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) && (nCurSubFunction == BACK_SUB_MODE_BODY_DETECT_1))
        {
            if(Input_GetWalkMotorPosition() >= nShoulderPositionTop - 3)
            {
                //设置连续蜂鸣器声音
                nBuzzerMode = BUZZER_MODE_FAST ;
                bSendBuzzerMode = TRUE;
                bBlueToothSendBuzzerMode = TRUE;
            }
            else
            {
                // if(nCurWalkMotorState == STATE_RUN_ANTICLOCK)
                {
                    nBuzzerMode = BUZZER_MODE_SLOW ;
                    bSendBuzzerMode = TRUE ;
                    bBlueToothSendBuzzerMode = TRUE;
                }
            }
        }
    }
    if(bKeyWalkDown == TRUE)
    {
        if((nBackMainRunMode == BACK_MAIN_MODE_MANUAL) && (nKeyBackLocate == LOCATE_POINT))
        {
            //设置连续蜂鸣器声音
            //  if(bReachWalkDownLimitFlag == TRUE)
            if(Input_GetWalkDownSwitch() == REACH_WALK_LIMIT)
            {
                //设置连续蜂鸣器声音
                nBuzzerMode = BUZZER_MODE_FAST ;
                bSendBuzzerMode = TRUE ;
                bBlueToothSendBuzzerMode = TRUE;
            }
            else
            {
                //if(nCurWalkMotorState == STATE_RUN_CLOCK)
                {
                    nBuzzerMode = BUZZER_MODE_SLOW ;
                    bSendBuzzerMode = TRUE ;
                    bBlueToothSendBuzzerMode = TRUE;
                }
            }
        }
        else if((nBackMainRunMode == BACK_MAIN_MODE_AUTO) && (nCurSubFunction == BACK_SUB_MODE_BODY_DETECT_1))
        {
            if(Input_GetWalkMotorPosition() <= nShoulderPositionBottom + 3)
            {
                //设置连续蜂鸣器声音
                nBuzzerMode = BUZZER_MODE_FAST ;
                bSendBuzzerMode = TRUE ;
                bBlueToothSendBuzzerMode = TRUE;
            }
            else
            {
                // if(nCurWalkMotorState == STATE_RUN_CLOCK)
                {
                    nBuzzerMode = BUZZER_MODE_SLOW ;
                    bSendBuzzerMode = TRUE ;
                    bBlueToothSendBuzzerMode = TRUE;
                }
            }
        }
    }
}
//engineer start
void engineering_stop_all(void)
{
    WaistHeat_Off();
    WalkMotor_Control(STATE_WALK_IDLE, 0);
    KneadMotor_Control(STATE_KNEAD_IDLE, KNEAD_SPEED0_PWM);
    LegMotor_Control(STATE_LEG_IDLE);
    BackMotor_Control(STATE_BACK_IDLE);
    SlideMotorControl(STATE_SLIDE_IDLE);
    //FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);//fww
    //FlexMotor_ResetTime = 0;
    KnockMotor_Set_Pwm_Data(0);
    RollerMotor_Control(0,0);
    LED_RGB_Set_All(0);
    //Valve_BodyUpAirPumpACPowerOff();
    //Valve_LegFootAirPumpACPowerOff();
}
//140531
void PowerOff_Pause_all(void)
{
    WaistHeat_Off();
    WalkMotor_Control(STATE_WALK_IDLE, 0);
    KneadMotor_Control(STATE_KNEAD_IDLE, KNEAD_SPEED0_PWM);
    LegMotor_Control(STATE_LEG_IDLE);
    BackMotor_Control(STATE_BACK_IDLE);
    SlideMotorControl(STATE_SLIDE_IDLE);
    //FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);//fww
    //FlexMotor_ResetTime = 0;
    KnockMotor_Set_Pwm_Data(0);
    KnockMotor_Break();
    
    RollerMotor_Control(0,0);
    //LED_RGB_Set_All(0);
    Valve_LegFootAirPumpACPowerOff();
}

BITS engineerData1old;
#define walk_up_old         engineerData1old.bD0
#define walk_down_old       engineerData1old.bD1
#define shoulder_detect_old engineerData1old.bD2
#define knead_width_min_old engineerData1old.bD3
#define knead_width_mid_old engineerData1old.bD4
#define knead_width_max_old engineerData1old.bD5
#define back_up_old         engineerData1old.bD6
#define back_down_old       engineerData1old.bD7
BITS engineerData2old;
#define leg_up_old          engineerData2old.bD0
#define leg_down_old        engineerData2old.bD1
#define test_finish         engineerData2old.bD2
#define leg_angle_old       engineerData2old.bD3
#define leg_ground_old      engineerData2old.bD4
#define foot_Switch_old     engineerData2old.bD5

//#define test_finish     engineerData2old.bD2
BITS engineerData1;
#define walk_up         engineerData1.bD0
#define walk_down       engineerData1.bD1
#define shoulder_detect engineerData1.bD2
#define knead_width_min engineerData1.bD3
#define knead_width_mid engineerData1.bD4
#define knead_width_max engineerData1.bD5
#define back_up         engineerData1.bD6
#define back_down       engineerData1.bD7
BITS engineerData2;
#define leg_up          engineerData2.bD0
#define leg_down        engineerData2.bD1
#define has_leg         engineerData2.bD2
#define knock           engineerData2.bD3
#define roller          engineerData2.bD4
#define heat            engineerData2.bD5
#define has_heat        engineerData2.bD6
#define air_bag         engineerData2.bD7

BITS engineerData5;
#define slide_backward  engineerData5.bD0
#define slide_forward   engineerData5.bD1
#define flex_up         engineerData5.bD2
#define flex_down       engineerData5.bD3
#define foot_Switch     engineerData5.bD4
#define leg_angle       engineerData5.bD5
#define leg_ground      engineerData5.bD6
#define knead_phase     engineerData5.bD7

typedef union
{
    struct
    {
        unsigned bD0: 2 ;
        unsigned bD1: 2 ;
        unsigned bD2: 2 ;
        unsigned bD3: 2 ;
    } ;
    unsigned char nByte ;
} BITS2 ;
BITS2 engineerData3;
#define walk_check_count     engineerData3.bD0
#define shoulder_check_count engineerData3.bD1
#define knead_check_count    engineerData3.bD2
#define back_check_count     engineerData3.bD3
BITS2 engineerData4;
#define leg_check_count      engineerData4.bD0

#define TIME_COUNT      100



//此函数执行完毕会引起CPU复位
void Main_Engineering(void)
{
   unsigned int engineerTimeCount = 0, air_bagTimeCount = 0;
   int leg_flex_step = 0;
   int slide_step = 0;
   has_heat = 1;
    has_leg = 1;
    heat = 1;
    knock = 1;
    roller = 1;
    bool strengthMode,sleepMode;
    unsigned int back_position, walk_position;
    unsigned char oneKeyStep = 0, oneKeyStepLength = 4, enAirbagStep;
    unsigned char oneKeyLegCountDown = 0;
    engineering_stop_all();
    unsigned short adc24, adcVcc, adc24_1, tempture;
    int engStatus = LINGO_ENG;
    unsigned int overCounter = 0;
    bool status = true;
    bool bProgram = false;
    char lingo;
    bool bHeat = false;
    char command;
    unsigned char PWM = 0;
    char airbagIndex = 1, airpumpIndex = 0;//Fungares
    unsigned int airbag;
    unsigned int pw_Information[5];
    unsigned char strength;
    unsigned char rollerSpeed = 0;
    unsigned char rollerPhase = 0;
    unsigned char color;
    unsigned char kneadSpeed = 0;
    unsigned char kneadPhase = 0;
    bool bUpKey = false;
    bool bDownKey = false;
    IndicateLED_On();
    ADC_Get_Voltage(ADC_VCC, &adcVcc);
    ADC_Get_Voltage(ADC_V24, &adc24);
    ADC_Get_Voltage(ADC_V24_1, &adc24_1);
    tempture = ADC_Get_Inttemp();
    memset(pw_Information, 0, sizeof(pw_Information));
    PBYTE pInformation = (PBYTE)pw_Information;
    MEM_Read_Memory(pw_Information, MEMORY_LENGTH_OF_BYTES);
// Power_On();
    while(status)
    {
        WDOG_Feed();
        lingo = Main_GetKey();
        switch(lingo)
        {
        case LINGO_AIRBAG:
        {
            engStatus = LINGO_AIRBAG;
            command = DMAUart_GetExternKey();
            switch(command)
            {
            case SYS_KEY_UP:
                airbagIndex &= 0x7f;
                airbagIndex++;
                airbagIndex %= 16;//Fungares
                break;
            case SYS_KEY_DOWN:
                airbagIndex &= 0x7f;
                airbagIndex--;
                if(airbagIndex > 16)
                    airbagIndex = 15;
                break;
            case SYS_KEY_LEFT:
                //airpumpIndex++;
                //airpumpIndex &= 0x03;
               airpumpIndex = 0x03;
                break;
            case SYS_KEY_RIGHT:
                //airpumpIndex--;
                //airpumpIndex &= 0x03;
               airpumpIndex = 0x00;
                break;
            case SYS_KEY_ENTER:
                airbagIndex |= 0x80;
                break;
            }
        }
        break;
        case LINGO_ROLLER_TEST:
        {
            engStatus = LINGO_ROLLER_TEST;
            command = DMAUart_GetExternKey();
            switch(command)
            {
            case SYS_KEY_UP:
                if(rollerSpeed < 3)
                    rollerSpeed++;
                break;
            case SYS_KEY_DOWN:
                if(rollerSpeed > 0)
                    rollerSpeed--;
                break;
            case SYS_KEY_LEFT:
            case SYS_KEY_RIGHT:
                if(rollerPhase == 0)
                    rollerPhase = 1;
                else
                    rollerPhase = 0;
                break;
            case SYS_KEY_ENTER:
                break;
            }
        }
        break;
        case LINGO_SLIDE_TEST:
        {
            engStatus = LINGO_SLIDE_TEST;
            command = DMAUart_GetExternKey();
            switch(command)
            {
            case SYS_KEY_UP:
                bUpKey = true;
                bDownKey = false;
                break;
            case SYS_KEY_UP_RELEASE:
            case SYS_KEY_DOWN_RELEASE:
                bUpKey = false;
                bDownKey = false;
                break;
            case SYS_KEY_DOWN:
                bUpKey = false;
                bDownKey = true;
                break;
            }
        }
        break;
        case LINGO_BACK_TEST:
        {
            engStatus = LINGO_BACK_TEST;
            command = DMAUart_GetExternKey();
            switch(command)
            {
            case SYS_KEY_UP:
                bUpKey = true;
                bDownKey = false;
                break;
            case SYS_KEY_UP_RELEASE:
            case SYS_KEY_DOWN_RELEASE:
                bUpKey = false;
                bDownKey = false;
                break;
            case SYS_KEY_DOWN:
                bUpKey = false;
                bDownKey = true;
                break;
            }
        }
        break;
        case LINGO_LEG_TEST:
        {
            engStatus = LINGO_LEG_TEST;
            command = DMAUart_GetExternKey();
            switch(command)
            {
            case SYS_KEY_UP:
                bUpKey = true;
                bDownKey = false;
                break;
            case SYS_KEY_UP_RELEASE:
            case SYS_KEY_DOWN_RELEASE:
                bUpKey = false;
                bDownKey = false;
                break;
            case SYS_KEY_DOWN:
                bUpKey = false;
                bDownKey = true;
                break;
            }
        }
        break;
        
        case LINGO_ONE_KEY_TEST:
        {
            engStatus = LINGO_ONE_KEY_TEST;
            command = DMAUart_GetExternKey();
            engineerTimeCount = 1, air_bagTimeCount = 1; //清时间，不设零防止跳到下一步
            switch(command)
            {
            case SYS_KEY_UP://上一步
                if(oneKeyStep > 1)oneKeyStep--;
                else oneKeyStep = oneKeyStepLength;
                if(test_finish && oneKeyStep == 0)oneKeyStep = oneKeyStepLength;
                break;
            case SYS_KEY_DOWN://下一步
                if(oneKeyStep < oneKeyStepLength)oneKeyStep++;
                else oneKeyStep = 0;
                break;
            case SYS_KEY_LEFT://开，并且下一步(或气囊：上一步)
                switch(oneKeyStep)
                {
                case 1:
                    if(has_heat)heat = 1;
                    if(oneKeyStep < oneKeyStepLength)oneKeyStep++;
                    else oneKeyStep = 0;
                    break;
                case 2:
                    knock = 1;
                    if(oneKeyStep < oneKeyStepLength)oneKeyStep++;
                    else oneKeyStep = 0;
                    break;
                case 3:
                    roller = 1;
                    if(oneKeyStep < oneKeyStepLength)oneKeyStep++;
                    else oneKeyStep = 0;
                    break;
                case 4:
                    air_bag = 1;
                    if(enAirbagStep > 0)
                    {
                        enAirbagStep--;
                    }
                    else
                    {
                        enAirbagStep = 24;
                    }
                    break;
                }
                break;
            case SYS_KEY_RIGHT://关，并且下一步(或气囊：下一步)
                switch(oneKeyStep)
                {
                case 1:
                    if(has_heat)heat = 0;
                    if(oneKeyStep < oneKeyStepLength)oneKeyStep++;
                    else oneKeyStep = 0;
                    break;
                case 2:
                    knock = 0;
                    if(oneKeyStep < oneKeyStepLength)oneKeyStep++;
                    else oneKeyStep = 0;
                    break;
                case 3:
                    roller = 0;
                    if(oneKeyStep < oneKeyStepLength)oneKeyStep++;
                    else oneKeyStep = 0;
                    break;
                case 4:
                    air_bag = 1;
                    if(enAirbagStep < 24)
                    {
                        enAirbagStep++;
                    }
                    else
                    {
                        enAirbagStep = 0;
                    }
                    break;
                }
                break;
            case 248://菜单中进入此界面
                //初始化
                engineerData1old.nByte = 0;
                engineerData2old.nByte = 0;
                engineerData3.nByte = 0;
                engineerData4.nByte = 0;
                oneKeyStep = 0;
                enAirbagStep = 0;
                test_finish = 0;
                heat = 1;
                knock = 1;
                roller = 1;
                air_bag = 1;
                walk_up = 0;
                walk_down = 0;
                shoulder_detect = 0;
                knead_width_min = 0;
                knead_width_mid = 0;
                knead_width_max = 0;
                leg_up = 0;
                leg_down = 0;
                back_up = 0;
                back_down = 0;
                back_position = 0;
                walk_position = 0;
                engineering_stop_all();
                engineerData5.nByte = 0;
                leg_flex_step = 0;
                slide_step = 0;
                
                leg_angle_old = Input_GetFlexAngleSwitch();      
                leg_ground_old = Input_GetFlexGroundSwitch();   
                foot_Switch_old = Input_GetFlexFootSwitch();
                shoulder_detect_old = Input_GetVout();
                
                break;
            case 15:
                engineering_stop_all();
                oneKeyStep = 0;
                break;
            default:
                break;
            }
        }
        break;
        case LINGO_HEAT_TEST:
        {
            engStatus = LINGO_HEAT_TEST;
            command = DMAUart_GetExternKey();
            switch(command)
            {
            case SYS_KEY_ENTER:
                if(bHeat)
                {
                    bHeat = 0;
                    WaistHeat_Off();
                }
                else
                {
                    bHeat = 1;
                    WaistHeat_On();
                }
                break;
            }
        }
        break;
        case LINGO_FLEX_TEST:
        {
            engStatus = LINGO_FLEX_TEST;
            command = DMAUart_GetExternKey();
            switch(command)
            {
            case SYS_KEY_UP:
                bUpKey = true;
                bDownKey = false;
                break;
            case SYS_KEY_UP_RELEASE:
            case SYS_KEY_DOWN_RELEASE:
                bUpKey = false;
                bDownKey = false;
                break;
            case SYS_KEY_DOWN:
                bUpKey = false;
                bDownKey = true;
                break;
            }
        }
        break;
        case LINGO_WALK_TEST:
        {
            engStatus = LINGO_WALK_TEST;
            command = DMAUart_GetExternKey();
            switch(command)
            {
            case SYS_KEY_UP:
                bUpKey = true;
                bDownKey = false;
                break;
            case SYS_KEY_UP_RELEASE:
            case SYS_KEY_DOWN_RELEASE:
                bUpKey = false;
                bDownKey = false;
                break;
            case SYS_KEY_DOWN:
                bUpKey = false;
                bDownKey = true;
                break;
            }
        }
        break;
        case LINGO_LED_TEST:
        {
            engStatus = LINGO_LED_TEST;
            command = DMAUart_GetExternKey();
            switch(command)
            {
            case SYS_KEY_UP:
                break;
            case SYS_KEY_DOWN:
                break;
            case SYS_KEY_LEFT:
            case SYS_KEY_RIGHT:
                break;
            case SYS_KEY_ENTER:
                color++;
                color %= 3;
                break;
            }
        }
        break;
        case LINGO_KNEAD_TEST:
        {
            engStatus = LINGO_KNEAD_TEST;
            command = DMAUart_GetExternKey();
            switch(command)
            {
            case SYS_KEY_UP:
                if(kneadSpeed < 6)
                    kneadSpeed++;
                break;
            case SYS_KEY_DOWN:
                if(kneadSpeed > 0)
                    kneadSpeed--;
                break;
            case SYS_KEY_LEFT:
            case SYS_KEY_RIGHT:
                if(kneadPhase == 0)
                    kneadPhase = 1;
                else
                    kneadPhase = 0;
                break;
            case SYS_KEY_ENTER:
                break;
            }
        }
        break;
        case LINGO_KNOCK_TEST:
        {
            engStatus = LINGO_KNOCK_TEST;
            command = DMAUart_GetExternKey();
            switch(command)
            {
            case SYS_KEY_UP:
                if(kneadSpeed < 6)
                    kneadSpeed++;
                break;
            case SYS_KEY_DOWN:
                if(kneadSpeed > 0)
                    kneadSpeed--;
                break;
            case SYS_KEY_LEFT:
            case SYS_KEY_RIGHT:
                if(kneadPhase == 0)
                    kneadPhase = 1;
                else
                    kneadPhase = 0;
                break;
            case SYS_KEY_ENTER:
                break;
            }
        }
        break;
        case LINGO_INPUT:
        {
            engStatus = LINGO_INPUT;
            command = DMAUart_GetExternKey();
            switch(command)
            {
            case SYS_KEY_UP:
                break;
            case SYS_KEY_DOWN:
                break;
            case SYS_KEY_LEFT:
                break;
            case SYS_KEY_RIGHT:
                break;
            case SYS_KEY_ENTER:
                break;
            }
        }
        break;
        case LINGO_MUSIC_TEST:
        {
            engStatus = LINGO_MUSIC_TEST;
            command = DMAUart_GetExternKey();
            switch(command)
            {
            case SYS_KEY_UP:
                break;
            case SYS_KEY_DOWN:
                break;
            case SYS_KEY_LEFT:
                break;
            case SYS_KEY_RIGHT:
                break;
            case SYS_KEY_ENTER:
                Power_AMP_Off();
                Timer_Counter_Clear(C_TIMER_TEMP);
                break;
            }
        }
        break;
        case LINGO_TOTAL_TIME:
        {
            engStatus = LINGO_TOTAL_TIME;
            command = DMAUart_GetExternKey();
            switch(command)
            {
            case SYS_KEY_UP:
                break;
            case SYS_KEY_DOWN:
                break;
            case SYS_KEY_LEFT:
                break;
            case SYS_KEY_RIGHT:
                break;
            case SYS_KEY_ENTER:
                break;
            }
        }
        break;
        
        
        
        case LINGO_PROGRAM:
            engStatus = LINGO_PROGRAM;
            if(*(pInformation + PROGRAM_ENABLE_ADDRESS) != PROGRAM_FLAG)
            {
                *(pInformation + PROGRAM_ENABLE_ADDRESS) = PROGRAM_FLAG; //写编程标志位
                MEM_Write_Memory(pw_Information, MEMORY_LENGTH_OF_BYTES);
            }
            bProgram = true;
            break;
        case LINGO_PROGRAM_BY_BLUETOOTH:
          /*
            engStatus = LINGO_PROGRAM;
            *(pInformation + PROGRAM_ENABLE_ADDRESS) = PROGRAM_BY_BLUETOOTH_FLAG; //写蓝牙编程标志位
            MEM_Write_Memory(pw_Information, MEMORY_LENGTH_OF_BYTES);
            bProgram = true;
          */
            break;
        case LINGO_BLUETOOTH_BR115200:
          /*
            while(!bMasterSendPacket);
            OutBuffer[0] = 'B' ;
            OutBuffer[1] = 'C' ;
            OutBuffer[2] = ':' ;
            OutBuffer[3] = 'B' ;
            OutBuffer[4] = 'R' ;
            OutBuffer[5] = '=' ;
            OutBuffer[6] = '0' ;
            OutBuffer[7] = 'C' ;
            OutBuffer[8] = '\r' ;
            OutBuffer[9] = '\n' ;
            nOutBufferCount = 10;
            DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
            bMasterSendPacket = FALSE;
          */
            break;
        case LINGO_ENG:
        {
            engStatus = LINGO_ENG;
            command = DMAUart_GetExternKey();
            switch(command)
            {
            case ENG_CMD_RESET:  //关机是否复位

                if(*(pInformation + SETTLE_ADDRESS))
                {
                    *(pInformation + SETTLE_ADDRESS) = 0;
                }
                else
                {
                    *(pInformation + SETTLE_ADDRESS) = 1;
                }
                MEM_Write_Memory(pw_Information, MEMORY_LENGTH_OF_BYTES);
                break;
            case ENG_CMD_DEC_STRENGTH:  //气囊力度减1
                strength = *(pInformation + AIRBAG_STRETCH_ADDRESS);
                if(strength == 0) break;
                strength--;
                strength %= 3;  //防止因为断电等原因导致数据错误
                *(pInformation + AIRBAG_STRETCH_ADDRESS) = strength;
                MEM_Write_Memory(pw_Information, MEMORY_LENGTH_OF_BYTES);
                break;
            case ENG_CMD_ADD_STRENGTH:  //气囊力度加1
                strength = *(pInformation + AIRBAG_STRETCH_ADDRESS);
                if(strength >= 2) break;
                strength++;
                *(pInformation + AIRBAG_STRETCH_ADDRESS) = strength;
                MEM_Write_Memory(pw_Information, MEMORY_LENGTH_OF_BYTES);
                break;
            case ENG_CMD_STRETCH_MODE:    //拉退程序切换
                strengthMode = *(pInformation + STRETCH_OUT_ADDRESS);
                if(strengthMode == 0) strengthMode = 1;
                else strengthMode = 0;
                *(pInformation + STRETCH_OUT_ADDRESS) = strengthMode;
                MEM_Write_Memory(pw_Information, MEMORY_LENGTH_OF_BYTES);
                break;    
            case ENG_CMD_SLEEP_PROGRAM:    //睡眠程序打开与关闭
                sleepMode = *(pInformation + SLEEP_PROGRAM_ADDRESS);
                if(sleepMode == 0) sleepMode = 1;
                else sleepMode = 0;
                *(pInformation + SLEEP_PROGRAM_ADDRESS) = sleepMode;
                MEM_Write_Memory(pw_Information, MEMORY_LENGTH_OF_BYTES);
                break;        
            case ENG_CMD_SLIDE:   //滑动使能禁止
                if(*(pInformation + SLIDE_MOTOR_ENABLE_ADDRESS))
                {
                    *(pInformation + SLIDE_MOTOR_ENABLE_ADDRESS) = 0;

                }
                else
                {
                    *(pInformation + SLIDE_MOTOR_ENABLE_ADDRESS) = 1;
                }
                MEM_Write_Memory(pw_Information, MEMORY_LENGTH_OF_BYTES);
                break;
            }
        }
        break;
        case LINGO_RESET:
            password = 0;
            NVIC_SystemReset();
            break; //复位CPU
        case LINGO_MENU:
            engStatus = LINGO_MENU;
            break; //复位CPU
        }
        /*******以下程序为气囊测试*************************/
        switch(engStatus)
        {
        case LINGO_ONE_KEY_TEST:
        {
            //检测信号 TODO,测试无信号时的情况
            Input_Proce();
            //使用中断标志
            if(engineeringTime_10msFlag)
            {
                engineerTimeCount++;
                engineerTimeCount %= 10 * TIME_COUNT; //10秒走一步
                air_bagTimeCount++;
                air_bagTimeCount %= 7 * TIME_COUNT; //10秒走一步
                if(oneKeyLegCountDown > 0)oneKeyLegCountDown--;
                //时间中断清零
                engineeringTime_10msFlag = 0;
            }
            //实现
            //参数
            //back_position = Input_GetBackMotorPosition();
            back_position = 0;
            walk_position = Input_GetWalkMotorPosition();
            //自动测试步骤
            if(oneKeyStep == 0)
            {
                //行走
                if(!walk_up)
                {
                    if(Input_GetWalkUpSwitch() == REACH_WALK_LIMIT)
                    {
                        if(walk_up_old == 0)
                        {
                            if(walk_check_count < 3)
                            {
                                walk_check_count++;
                            }
                            else
                            {
                                //上行程OK
                                walk_up = 1;
                                //清零
                                walk_check_count = 0;
                            }
                        }
                        WalkMotor_Control(STATE_RUN_WALK_DOWN, 0);
                    }
                    else
                    {
                        WalkMotor_Control(STATE_RUN_WALK_UP, 0);
                    }
                }//肩位
                else if(!walk_down)
                {
                    if(Input_GetWalkDownSwitch() == REACH_WALK_LIMIT)
                    {
                        Input_SetWalkMotorPosition(0);
                        if(walk_down_old == 0)
                        {
                            if(walk_check_count < 3)
                            {
                                walk_check_count++;
                            }
                            else
                            {
                                //下行程OK
                                walk_down = 1;
                                //清零
                                walk_check_count = 0;
                            }
                        }
                        WalkMotor_Control(STATE_RUN_WALK_UP, 0);
                    }
                    else
                    {
                        WalkMotor_Control(STATE_RUN_WALK_DOWN, 0);
                    }
                }
                else
                {
                    if(Input_GetWalkUpSwitch() != REACH_WALK_LIMIT)
                    {
                        WalkMotor_Control(STATE_RUN_WALK_UP, 0);
                    }
                    else
                    {
                        WalkMotor_Control(STATE_WALK_IDLE, 0);
                    }
                }
                //揉捏
                if(!knead_width_min)
                {
                    if(Input_GetKneadMin() == 0)
                    {
                        if(knead_width_min_old == 0)
                        {
                            if(knead_check_count < 3)
                            {
                                knead_check_count++;
                            }
                            else
                            {
                                //OK
                                knead_width_min = 1;
                                //清零
                                knead_check_count = 0;
                            }
                        }
                        KneadMotor_Control(STATE_KNEAD_CLOCK_RUN, KNEAD_SPEED1_PWM);
                    }
                    else
                    {
                        KneadMotor_Control(STATE_KNEAD_UNCLOCK_RUN, KNEAD_SPEED1_PWM);
                    }
                }
                else if(!knead_width_mid)
                {
                    if(Input_GetKneadMid() == 0)
                    {
                        if(knead_width_mid_old == 0)
                        {
                            if(knead_check_count < 3)
                            {
                                knead_check_count++;
                            }
                            else
                            {
                                //OK
                                knead_width_mid = 1;
                                //清零
                                knead_check_count = 0;
                            }
                        }
                        KneadMotor_Control(STATE_KNEAD_CLOCK_RUN, KNEAD_SPEED6_PWM);
                    }
                    else
                    {
                        KneadMotor_Control(STATE_KNEAD_UNCLOCK_RUN, KNEAD_SPEED6_PWM);
                    }
                }
                else if(!knead_width_max)
                {
                    if(Input_GetKneadMax() == 0)
                    {
                        if(knead_width_max_old == 0)
                        {
                            if(knead_check_count < 3)
                            {
                                knead_check_count++;
                            }
                            else
                            {
                                //OK
                                knead_width_max = 1;
                                //清零
                                knead_check_count = 0;
                            }
                        }
                        KneadMotor_Control(STATE_KNEAD_CLOCK_RUN, KNEAD_SPEED1_PWM);
                    }
                    else
                    {
                        KneadMotor_Control(STATE_KNEAD_UNCLOCK_RUN, KNEAD_SPEED1_PWM);
                    }
                }
                else KneadMotor_Control(STATE_KNEAD_IDLE, KNEAD_SPEED0_PWM);
                //小腿
                if(has_leg)
                {
                    if((!leg_up) &&(!leg_down)/*&&(!flex_up)&&(!flex_down)*/)  
                    {   //测试小腿上行程开关
                       //FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A);//fww
                       switch(leg_flex_step)
                       {
                        case 0:  //到达up位置
                                if(LegMotor_Control(STATE_RUN_LEG_UP)) 
                                {
                                  leg_flex_step++;
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                }
                                break;  
                         case 1:  //停在up位置0.5秒
                               LegMotor_Control(STATE_LEG_IDLE);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;  
                         case 2:  //离开up位置
                                LegMotor_Control(STATE_RUN_LEG_DOWN);
                                if(Input_GetLegUpSwitch() != REACH_LEG_LIMIT)
                                {
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                  leg_flex_step++;
                                }
                                break;
                          case 3:  //停0.5秒
                               LegMotor_Control(STATE_LEG_IDLE);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;
                          case 4:  //到达up位置
                                if(LegMotor_Control(STATE_RUN_LEG_UP)) 
                                {
                                  leg_flex_step = 0;
                                  leg_up = 1;
                                }
                                break;       
                       }
                    }
                    /******************************fww*******************************
                    if((leg_up) &&(!leg_down)&&(!flex_up)&&(!flex_down))  
                      {//测试电动伸缩小腿上（外）行程开关
                        LegMotor_Control(STATE_LEG_IDLE);
                        switch(leg_flex_step)
                       {
                        case 0:  //到达up位置
                                if(FlexMotor_Control(STATE_RUN_FLEX_MANUAL_OUT, FLEX_SPEED_FAST, FLEX_CURRENT_3A))
                                {
                                  leg_flex_step++;
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                }
                                break;  
                         case 1:  //停在up位置0.5秒
                               FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                               //FlexMotor_ResetTime = 0;
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;  
                         case 2:  //离开up位置
                                FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                                if(Input_GetFlexOutSwitch() != REACH_FLEX_LIMIT)
                                {
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                  leg_flex_step++;
                                }
                                break;
                          case 3:  //停0.5秒
                               FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
                               //FlexMotor_ResetTime = 0;
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;
                          case 4:  //到达up位置
                                if(FlexMotor_Control(STATE_RUN_FLEX_MANUAL_OUT, FLEX_SPEED_FAST, FLEX_CURRENT_3A))
                                {
                                  leg_flex_step = 0;
                                  flex_up = 1;
                                }
                                break;       
                       }
                      }
                    if((leg_up) &&(!leg_down)&&(flex_up)&&(!flex_down))  
                      {//测试电动伸缩小腿上（外）行程开关
                        LegMotor_Control(STATE_LEG_IDLE);
                        switch(leg_flex_step)
                       {
                        case 0:  //到达in位置
                                //if(FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A))//fww
                                {
                                  leg_flex_step++;
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                }
                                break;  
                         case 1:  //停在in位置0.5秒
                               //FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);//fww
                               //FlexMotor_ResetTime = 0;
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;  
                         case 2:  //离开in位置
                                //FlexMotor_Control(STATE_RUN_FLEX_MANUAL_OUT, FLEX_SPEED_FAST, FLEX_CURRENT_3A);//fww
                                if(Input_GetFlexInSwitch() != REACH_FLEX_LIMIT)
                                {
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                  leg_flex_step++;
                                }
                                break;
                          case 3:  //停0.5秒
                               //FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);//fww
                               //FlexMotor_ResetTime = 0;
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;
                          case 4:  //到达In位置
                                //if(FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A))//fww
                                {
                                  leg_flex_step = 0;
                                  flex_down = 1;
                                }
                                break;       
                       }
                      }
                    **********************************fww**************************/
                   if((leg_up) &&(!leg_down)/*&&(flex_up)&&(flex_down)*/)   
                   {   //测试小腿上行程开关
                       //FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A); //关闭电动伸缩小腿//fww
                       //FlexMotor_ResetTime = 0;
                       switch(leg_flex_step)
                       {
                        case 0:  //到达down位置
                                if(LegMotor_Control(STATE_RUN_LEG_DOWN)) 
                                {
                                  leg_flex_step++;
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                }
                                break;  
                         case 1:  //停在DOWN位置0.5秒
                               LegMotor_Control(STATE_LEG_IDLE);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;  
                         case 2:  //离开DOWN位置
                                LegMotor_Control(STATE_RUN_LEG_UP) ;
                                if(Input_GetLegDownSwitch() != REACH_LEG_LIMIT);
                                {
                                  Timer_Counter_Clear(C_TIMER_ENG1);
                                  leg_flex_step++;
                                }
                                break;
                          case 3:  //停0.5秒
                               LegMotor_Control(STATE_LEG_IDLE);
                               if(Timer_Counter(C_TIMER_ENG1,5))
                                {
                                  leg_flex_step++;
                                }
                               break;
                          case 4:  //到达Down位置
                                if(LegMotor_Control(STATE_RUN_LEG_DOWN)) 
                                {
                                  leg_flex_step = 0;
                                  leg_down = 1;
                                }
                                break;       
                       }
                    }
                }//has_leg
                //靠背
              if(leg_angle_old != Input_GetFlexAngleSwitch())      
              {
                leg_angle = 1;
              }
              if(leg_ground_old != Input_GetFlexGroundSwitch())
              {
                leg_ground = 1;
              } 
              if(foot_Switch_old != Input_GetFlexFootSwitch())
              {
                foot_Switch = 1;
              }
              
              if(shoulder_detect_old != Input_GetVout())
              {
                shoulder_detect = 1;
              }
              
                if((!slide_backward) && (!slide_forward))
                {   //测试前滑前行程开关
                       switch(slide_step)
                       {
                        case 0:  //到达最前位置
                          if(SlideMotorControl(STATE_RUN_SLIDE_FORWARD))
                            {
                               slide_step++;
                               Timer_Counter_Clear(C_TIMER_ENG2);
                            }
                            break;  
                          //SlideMotorControl(STATE_RUN_SLIDE_BACKWARD);
                          //SlideMotorControl(STATE_SLIDE_IDLE);
                         case 1:  //停在最前位置0.5秒
                               SlideMotorControl(STATE_SLIDE_IDLE);
                               if(Timer_Counter(C_TIMER_ENG2,5))
                                {
                                  slide_step++;
                                }
                               break;  
                         case 2: //离开最前位置
                                SlideMotorControl(STATE_RUN_SLIDE_BACKWARD);
                                if(Input_GetSlideForwardSwitch() != REACH_SLIDE_LIMIT)
                                {
                                  Timer_Counter_Clear(C_TIMER_ENG2);
                                  slide_step++;
                                }
                                break;
                          case 3:  //停0.5秒
                               SlideMotorControl(STATE_SLIDE_IDLE);
                               if(Timer_Counter(C_TIMER_ENG2,5))
                                {
                                  slide_step++;
                                }
                               break;
                          case 4:  //到达最前位置
                            if(SlideMotorControl(STATE_RUN_SLIDE_FORWARD))
                            {
                                  slide_step = 0;
                                  slide_forward = 1;
                            }
                            break;
                       } //end switch
                    }
                
                if((!slide_backward) && (slide_forward))
                {   //测试前滑后行程开关
                       switch(slide_step)
                       {
                        case 0:  //到达最后位置
                          if(SlideMotorControl(STATE_RUN_SLIDE_BACKWARD))
                            {
                               slide_step++;
                               Timer_Counter_Clear(C_TIMER_ENG2);
                            }
                            break;  
                         case 1:  //停在最后位置0.5秒
                               SlideMotorControl(STATE_SLIDE_IDLE);
                               if(Timer_Counter(C_TIMER_ENG2,5))
                                {
                                  slide_step++;
                                }
                               break;  
                         case 2: //离开最后位置
                                SlideMotorControl(STATE_RUN_SLIDE_FORWARD);
                                if(Input_GetSlideBackwardSwitch() != REACH_SLIDE_LIMIT)
                                {
                                  Timer_Counter_Clear(C_TIMER_ENG2);
                                  slide_step++;
                                }
                                break;
                          case 3:  //停0.5秒
                               SlideMotorControl(STATE_SLIDE_IDLE);
                               if(Timer_Counter(C_TIMER_ENG2,5))
                                {
                                  slide_step++;
                                }
                               break;
                          case 4:  //到达最后位置
                            if(SlideMotorControl(STATE_RUN_SLIDE_BACKWARD))
                            {
                                  slide_step = 0;
                                  slide_backward = 1;
                            }
                            break;
                       } //end switch
                    }
                if(!back_up)
                {
                    if(Input_GetBackUpSwitch() == REACH_BACK_LIMIT)
                    {
                        if(back_up_old == 0)
                        {
                            if(back_check_count < 3)
                            {
                                back_check_count++;
                            }
                            else
                            {
                                //OK
                                back_up = 1;
                                //清零
                                back_check_count = 0;
                            }
                        }
                        BackMotor_Control(STATE_RUN_BACK_DOWN);
                    }
                    else
                    {
                        BackMotor_Control(STATE_RUN_BACK_UP);
                    }
                }
                else if(!back_down)
                {
                    if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT)
                    {
                        if(back_down_old == 0)
                        {
                            if(back_check_count < 3)
                            {
                                back_check_count++;
                            }
                            else
                            {
                                //OK
                                back_down = 1;
                                //清零
                                back_check_count = 0;
                            }
                        }
                        BackMotor_Control(STATE_RUN_BACK_UP);
                    }
                    else
                    {
                        BackMotor_Control(STATE_RUN_BACK_DOWN);
                    }
                }
                else
                {
                    if(Input_GetBackUpSwitch() != REACH_BACK_LIMIT)
                    {
                        BackMotor_Control(STATE_RUN_BACK_UP);
                    }
                    else
                    {
                        BackMotor_Control(STATE_BACK_IDLE);
                    }
                }

                if(walk_up && walk_down && shoulder_detect && knead_width_min && knead_width_mid
                        && knead_width_max && ((!has_leg) || (leg_up && leg_down)) && back_up && back_down 
                          && slide_backward && slide_forward && flex_up && flex_down && foot_Switch 
                            && leg_angle && leg_ground)
                {
                    if(oneKeyStep == 0)
                    {
                        oneKeyStep++;
                        engineerTimeCount = 1;
                    }
                }
            }
            else
            {
                if(Input_GetWalkUpSwitch() != REACH_WALK_LIMIT && walk_up)
                {
                    WalkMotor_Control(STATE_RUN_WALK_UP, 0);
                }
                else
                {
                    WalkMotor_Control(STATE_WALK_IDLE, 0);
                }
                if(Input_GetLegDownSwitch() != REACH_LEG_LIMIT && has_leg && leg_down)
                {
                    LegMotor_Control(STATE_RUN_LEG_DOWN);
                }
                else
                {
                    LegMotor_Control(STATE_LEG_IDLE);
                }
                if(Input_GetBackUpSwitch() != REACH_BACK_LIMIT && back_up)
                {
                    BackMotor_Control(STATE_RUN_BACK_UP);
                }
                else
                {
                    BackMotor_Control(STATE_BACK_IDLE);
                }
            }
            //需手动配合的部分
            //加热
            if(has_heat)
            {
                if(heat)WaistHeat_On();
                else WaistHeat_Off();
            }
            //敲击
            if(knock == 1)
            {
                if(engineerTimeCount < 9 * TIME_COUNT)
                {
                    KnockMotor_ClockRun();
                    KnockMotor_Set_Pwm_Data(KNOCK_SPEED1_PWM);
                }
                else
                {
                    KnockMotor_UnClockRun();
                    KnockMotor_Set_Pwm_Data(KNOCK_SPEED6_PWM);
                }
            }
            else
            {
                KnockMotor_Set_Pwm_Data(KNOCK_SPEED0_PWM);
                KnockMotor_Break();
            }
            //滚轮
            if(roller)
            {
                if(engineerTimeCount < 5 * TIME_COUNT)
                {
                    RollerMotor_Control(ROLLER_SPEED_SLOW, 0);
                }
                else
                {
                    RollerMotor_Control(ROLLER_SPEED_FAST, 1);
                }
            }
            else
            {
                RollerMotor_Control(ROLLER_SPEED_STOP, 0);
            }
            //气囊
            if(air_bag)
            {
                Valve_LegFootAirPumpACPowerOn(); //小腿和臀部

                BITS_ValveData[0].nByte = 0;
                BITS_ValveData[1].nByte = 0;
                if(enAirbagStep > 8)BITS_ValveData[1].nByte = (1 << (enAirbagStep - 9)) & 0xff;
                else BITS_ValveData[0].nByte = (1 << (enAirbagStep - 1)) & 0xff;
                //10秒后自动下一步
                if(air_bagTimeCount == 0)
                {
                    air_bagTimeCount++;//防止循环内重复调用
                    enAirbagStep++;
                }
                //测试结束
                if(enAirbagStep > 24)
                {
                    enAirbagStep = 0;//清零
                    
                        if(walk_up && walk_down && shoulder_detect && knead_width_min && knead_width_mid
                        && knead_width_max && ((!has_leg) || (leg_up && leg_down)) && back_up && back_down 
                          && slide_backward && slide_forward && flex_up && flex_down && foot_Switch 
                            && leg_angle && leg_ground)
                    {
                        test_finish = 1;
                        air_bag = 0;
                    }
                }
            }
            else
            {
               Valve_LegFootAirPumpACPowerOff(); //小腿和臀部
                BITS_ValveData[0].nByte = 0;
                BITS_ValveData[1].nByte = 0;
            }
            Valve_Send_Data();
            //手动配合步骤
            //1：加热，2：敲击，3：滚轮，4：气囊,加热时不自动增加
            if(oneKeyStep > 0 && oneKeyStep < oneKeyStepLength)
            {
                //10秒后自动下一步
                if(engineerTimeCount == 0)
                {
                    engineerTimeCount++;//防止循环内重复调用
                    switch(oneKeyStep)
                    {
                    case 2:
                        knock = 0;
                        break;
                    case 3:
                        roller = 0;
                        break;
                    }
                    oneKeyStep++;
                }
            }
            if(bMasterSendPacket)
            {
                OutBuffer[0] = SOI;
                OutBuffer[1] = 0;
                OutBuffer[1] |= heat;
                OutBuffer[1] |= has_heat << 1;
                OutBuffer[1] |= walk_up << 2;
                OutBuffer[1] |= walk_down << 3;
                OutBuffer[1] |= shoulder_detect << 4;
                OutBuffer[1] |= knead_width_min << 5;
                OutBuffer[1] |= knead_width_mid << 6;
                OutBuffer[1] |= knead_width_max << 7;
                OutBuffer[2] = 0;
                OutBuffer[2] |= has_leg;
                OutBuffer[2] |= leg_up << 1;
                OutBuffer[2] |= leg_down << 2;
                OutBuffer[2] |= back_up << 3;
                OutBuffer[2] |= back_down << 4;
                OutBuffer[2] |= (back_position & 0x7) << 5;
                OutBuffer[3] = ((back_position >> 3) & 0x7f) | ((walk_position & 0x1) << 7);
                OutBuffer[4] = (walk_position >> 1) & 0xff;
                OutBuffer[5] = (enAirbagStep & 0x1f) | ((oneKeyStep & 0x7) << 5);
                OutBuffer[6] = (knock << 7) | (roller << 6) | (test_finish << 5);
                
                OutBuffer[7] = 0;
                OutBuffer[7] |= slide_backward;
                OutBuffer[7] |= slide_forward << 1;
                OutBuffer[7] |= flex_up << 2;
                OutBuffer[7] |= flex_down << 3;
                OutBuffer[7] |= foot_Switch << 4;
                OutBuffer[7] |= leg_angle << 5;
                OutBuffer[7] |= leg_ground << 6;
                
                OutBuffer[8] = EOI;
                nOutBufferCount = 9;
                DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
                bMasterSendPacket = FALSE;
            }
            walk_up_old = (Input_GetWalkUpSwitch() == REACH_WALK_LIMIT);
            walk_down_old = (Input_GetWalkDownSwitch() == REACH_WALK_LIMIT);
            //shoulder_detect_old = (Input_GetVout() == BODY_TOUCHED);
            knead_width_min_old = (Input_GetKneadMin() == 0);
            knead_width_mid_old = (Input_GetKneadMid() == 0);
            knead_width_max_old = (Input_GetKneadMax() == 0);
            back_up_old = (Input_GetBackUpSwitch() == REACH_BACK_LIMIT);
            back_down_old = (Input_GetBackDownSwitch() == REACH_BACK_LIMIT);
            leg_up_old   = (Input_GetLegUpSwitch() == REACH_BACK_LIMIT);
            leg_down_old  = (Input_GetLegDownSwitch() == REACH_BACK_LIMIT);
        }
        break;  
          
        case LINGO_HEAT_TEST:
        {
            if(bMasterSendPacket)
            {
                OutBuffer[0] = SOI ;
                OutBuffer[1] = (unsigned char)bHeat;
                OutBuffer[2] = EOI ;
                nOutBufferCount = 3;
                DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
                bMasterSendPacket = FALSE ;
            }
        }
        break;


        case LINGO_MUSIC_TEST:
        {
            if(Timer_Counter(C_TIMER_TEMP, 1))
            {
                Power_AMP_On();  //0.1秒后开启蓝牙
            }
            if(bMasterSendPacket)
            {
                OutBuffer[0] = SOI ;
                OutBuffer[1] = 0;
                OutBuffer[2] = EOI ;
                nOutBufferCount = 3;
                DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
                bMasterSendPacket = FALSE ;
            }
        }
        break;
        
        case LINGO_TOTAL_TIME:
          {
            if(bMasterSendPacket)
            {
              unsigned int totalTime;
              totalTime = MEM_Get_Total_Time();
              OutBuffer[0] = SOI ;
              OutBuffer[1] = (unsigned char)(totalTime >> 24);
              OutBuffer[2] = (unsigned char)(totalTime >> 16);
              OutBuffer[3] = (unsigned char)(totalTime >> 8);
              OutBuffer[4] = (unsigned char)(totalTime);
              OutBuffer[5] = EOI ;
              nOutBufferCount = 6;
              DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
              bMasterSendPacket = FALSE ;
            }
          }
        break;
        
        case LINGO_LED_TEST:
        {
            Valve_Send_Data();
            Input_Proce();

            if(color == 0)
            {
                LED_RGB_Set_Red_Data(0);
                LED_RGB_Set_Green_Data(100);
                LED_RGB_Set_Blue_Data(0);
            }
            if(color == 1)
            {
                LED_RGB_Set_Red_Data(100);
                LED_RGB_Set_Green_Data(0);
                LED_RGB_Set_Blue_Data(0);
            }
            if(color == 2)
            {
                LED_RGB_Set_Red_Data(0);
                LED_RGB_Set_Green_Data(0);
                LED_RGB_Set_Blue_Data(100);
            }

            if(bMasterSendPacket)
            {
                OutBuffer[0] = SOI ;
                OutBuffer[1] = color;
                OutBuffer[2] = EOI ;
                nOutBufferCount = 3;
                DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
                bMasterSendPacket = FALSE ;
            }
        }
        break;

        case LINGO_SLIDE_TEST:
        {
            Valve_Send_Data();
            Input_Proce();
            if(bUpKey) SlideMotorControl(STATE_RUN_SLIDE_FORWARD);
            if(bDownKey) SlideMotorControl(STATE_RUN_SLIDE_BACKWARD);
            if(!bUpKey && !bDownKey) SlideMotorControl(STATE_SLIDE_IDLE);
            if(bMasterSendPacket)
            {
                OutBuffer[0] = SOI ;
                OutBuffer[1] = 0;
                if(Input_GetSlideForwardSwitch() == REACH_SLIDE_LIMIT)  OutBuffer[1] |= 0x01;
                if(Input_GetSlideBackwardSwitch() == REACH_SLIDE_LIMIT) OutBuffer[1] |= 0x02;
                OutBuffer[2] = EOI ;
                nOutBufferCount = 3;
                DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
                bMasterSendPacket = FALSE ;
            }
        }
        break;
        case LINGO_BACK_TEST:
        {
            Valve_Send_Data();
            Input_Proce();

            if(bUpKey) BackMotor_Control(STATE_RUN_BACK_UP);
            if(bDownKey) BackMotor_Control(STATE_RUN_BACK_DOWN);
            if(!bUpKey && !bDownKey) BackMotor_Control(STATE_BACK_IDLE);

            if(bMasterSendPacket)
            {
                OutBuffer[0] = SOI ;
                OutBuffer[1] = 0;
                if(Input_GetBackUpSwitch() == REACH_BACK_LIMIT) OutBuffer[1] |= 0x01;
                if(Input_GetBackDownSwitch() == REACH_BACK_LIMIT) OutBuffer[1] |= 0x02;
                OutBuffer[2] = EOI ;
                nOutBufferCount = 3;
                DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
                bMasterSendPacket = FALSE ;
            }
        }
        break;
        case LINGO_LEG_TEST:
        {
            Valve_Send_Data();
            Input_Proce();
            if(bUpKey) LegMotor_Control(STATE_RUN_LEG_UP);
            if(bDownKey) LegMotor_Control(STATE_RUN_LEG_DOWN);
            if(!bUpKey && !bDownKey) LegMotor_Control(STATE_LEG_IDLE);

            if(bMasterSendPacket)
            {
                OutBuffer[0] = SOI ;
                OutBuffer[1] = 0;
                if(Input_GetLegUpSwitch() == REACH_BACK_LIMIT) OutBuffer[1] |= 0x01;
                if(Input_GetLegDownSwitch() == REACH_BACK_LIMIT) OutBuffer[1] |= 0x02;
                if(Input_GetFlexAngleSwitch() == LEGANGLE_SWITCH_ON) OutBuffer[1] |= 0x04; //小于15度
                if(Input_GetFlexGroundSwitch() == LEGGROUND_SWITCH_ON) OutBuffer[1] |= 0x08; //碰到地面了
                OutBuffer[2] = EOI ;
                nOutBufferCount = 3;
                DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
                bMasterSendPacket = FALSE ;
            }
        }
        break;
        case LINGO_FLEX_TEST:
        {
            Valve_Send_Data();
            Input_Proce();
            if(bUpKey) FlexMotor_Control(STATE_RUN_FLEX_TEST_OUT, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
            if(bDownKey) FlexMotor_Control(STATE_RUN_FLEX_RESET, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
            if(!bUpKey && !bDownKey) 
            {
              FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A);
              //FlexMotor_ResetTime = 0;
            }
            if(bMasterSendPacket)
            {
                OutBuffer[0] = SOI ;
                OutBuffer[1] = 0;
                
                if(Input_GetFlexOutSwitch() == REACH_FLEX_LIMIT ) OutBuffer[1] |= 0x01;
                if(Input_GetFlexInSwitch() == REACH_FLEX_LIMIT ) OutBuffer[1] |= 0x02;
                if(Input_GetFlexAngleSwitch() == LEGANGLE_SWITCH_ON) OutBuffer[1] |= 0x04; //小于15度
                if(Input_GetFlexGroundSwitch() == LEGGROUND_SWITCH_ON) OutBuffer[1] |= 0x08; //碰到地面了
                if(Input_GetFlexFootSwitch() == FOOT_SWITCH_ON) OutBuffer[1] |= 0x10; //碰到脚了
                
                OutBuffer[2] = EOI ;
                nOutBufferCount = 3;
                DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
                bMasterSendPacket = FALSE ;
            }
        }
        break;
        case LINGO_WALK_TEST:
        {
            Valve_Send_Data();
            Input_Proce();
            if(bUpKey) WalkMotor_Control(STATE_RUN_WALK_UP, 0);
            if(bDownKey) WalkMotor_Control(STATE_RUN_WALK_DOWN, 0);
            if(!bUpKey && !bDownKey) WalkMotor_Control(STATE_WALK_IDLE, 0);
            if(bMasterSendPacket)
            {
                OutBuffer[0] = SOI ;
                OutBuffer[1] = 0;
                if(Input_GetWalkUpSwitch() == REACH_WALK_LIMIT) OutBuffer[1] |= 0x01;
                if(Input_GetWalkDownSwitch() == REACH_WALK_LIMIT) OutBuffer[1] |= 0x02;
                if(Input_GetVout() == BODY_TOUCHED) OutBuffer[1] |= 0x04;
                OutBuffer[2] = Input_GetWalkMotorPosition() >> 8;
                OutBuffer[3] = Input_GetWalkMotorPosition() ;
                OutBuffer[4] = EOI ;

                nOutBufferCount = 5;
                DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
                bMasterSendPacket = FALSE ;
            }
        }
        break;

        break;
        case LINGO_ROLLER_TEST:
        {
            if(rollerSpeed == 0)
            {
                RollerMotor_Control(ROLLER_SPEED_STOP, 0);
            }
            else
            {
                switch(rollerSpeed)
                {
                default:
                case 1:
                    PWM = ROLLER_SPEED_SLOW;
                    break ;
                case 2:
                    PWM = ROLLER_SPEED_MID;
                    break ;
                case 3:
                    PWM = ROLLER_SPEED_FAST;
                    break ;
                }
                if(rollerPhase == 0)
                    RollerMotor_Control(PWM, 0);
                else
                    RollerMotor_Control(PWM, 1);
            }
            if(bMasterSendPacket)
            {
                OutBuffer[0] = SOI ;
                OutBuffer[1] = rollerSpeed;
                OutBuffer[2] = rollerPhase;
                OutBuffer[3] = EOI ;
                nOutBufferCount = 4;
                DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
                bMasterSendPacket = FALSE ;
            }
        }
        break;
        case LINGO_KNOCK_TEST:
        {
            Valve_Send_Data();
            Input_Proce();
            if(kneadSpeed == 0)
            {
                KnockMotor_Break();
            }
            else
            {
                switch(kneadSpeed)
                {
                default:
                case 0:
                    PWM = KNOCK_SPEED0_PWM;
                    break ;
                case 1:
                    PWM = KNOCK_SPEED1_PWM;
                    break ;
                case 2:
                    PWM = KNOCK_SPEED2_PWM;
                    break ;
                case 3:
                    PWM = KNOCK_SPEED3_PWM;
                    break ;
                case 4:
                    PWM = KNOCK_SPEED4_PWM;
                    break ;
                case 5:
                    PWM = KNOCK_SPEED5_PWM;
                    break ;
                case 6:
                    PWM = KNOCK_SPEED6_PWM;
                    break ;
                }
                if(kneadPhase == 0)
                    KnockMotor_ClockRun();
                else
                    KnockMotor_UnClockRun();
                KnockMotor_Set_Pwm_Data(PWM);
            }

            if(bMasterSendPacket)
            {
                OutBuffer[0] = SOI ;
                OutBuffer[1] = kneadSpeed;
                OutBuffer[2] = kneadPhase;
                OutBuffer[3] = Input_GetKneadPosition();
                OutBuffer[4] = EOI ;
                nOutBufferCount = 5;
                DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
                bMasterSendPacket = FALSE ;
            }
        }
        break;
        case LINGO_KNEAD_TEST:
        {
            Valve_Send_Data();
            Input_Proce();
            if(kneadSpeed == 0)
            {
                KneadMotor_Control(STATE_KNEAD_IDLE, KNEAD_SPEED0_PWM);
            }
            else
            {
                switch(kneadSpeed)
                {
                default:
                case 0:
                    PWM = KNEAD_SPEED0_PWM;
                    break ;
                case 1:
                    PWM = KNEAD_SPEED1_PWM;
                    break ;
                case 2:
                    PWM = KNEAD_SPEED2_PWM;
                    break ;
                case 3:
                    PWM = KNEAD_SPEED3_PWM;
                    break ;
                case 4:
                    PWM = KNEAD_SPEED4_PWM;
                    break ;
                case 5:
                    PWM = KNEAD_SPEED5_PWM;
                    break ;
                case 6:
                    PWM = KNEAD_SPEED6_PWM;
                    break ;
                }
                if(kneadPhase == 0)
                    KneadMotor_Control(STATE_KNEAD_CLOCK_RUN, PWM);
                else
                    KneadMotor_Control(STATE_KNEAD_UNCLOCK_RUN, PWM);
            }

            if(bMasterSendPacket)
            {
                OutBuffer[0] = SOI ;
                OutBuffer[1] = kneadSpeed;
                OutBuffer[2] = kneadPhase;
                OutBuffer[3] = Input_GetKneadPosition();
                OutBuffer[4] = EOI ;
                nOutBufferCount = 5;
                DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
                bMasterSendPacket = FALSE ;
            }
        }
        break;
        case LINGO_INPUT:
            if(bMasterSendPacket)
            {
                if(Timer_Counter(C_TIMER_TEMP + T_LOOP, 10))
                {
                    ADC_Get_Voltage(ADC_VCC, &adcVcc);
                    ADC_Get_Voltage(ADC_V24, &adc24);
                    ADC_Get_Voltage(ADC_V24_1, &adc24_1);
                    tempture = ADC_Get_Inttemp();
                }
                OutBuffer[0] = SOI ;
                //5V电压
                OutBuffer[1] = (unsigned char)(adcVcc / 100);
                OutBuffer[2] = (unsigned char)(adcVcc % 100);
                //24V马达电压
                OutBuffer[3] = (unsigned char)(adc24 / 100);
                OutBuffer[4] = (unsigned char)(adc24 % 100);
                //24V气阀电压
                OutBuffer[5] = (unsigned char)(adc24_1 / 100);
                OutBuffer[6] = (unsigned char)(adc24_1 % 100);
                //CPU温度
                OutBuffer[7] = (unsigned char)(tempture / 100);
                OutBuffer[8] = (unsigned char)(tempture % 100);
                OutBuffer[9] = EOI ;
                nOutBufferCount = 10;
                DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
                bMasterSendPacket = FALSE ;
            }
            break;
        case LINGO_AIRBAG:
            if(bMasterSendPacket)
            {
                OutBuffer[0] = SOI ;
                OutBuffer[1] = airpumpIndex;
                OutBuffer[2] = airbagIndex;
                OutBuffer[3] = EOI ;
                nOutBufferCount = 4;
                DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
                bMasterSendPacket = FALSE ;
            }
            if(airbagIndex & 0x80)
            {
                airbag = 0xffffff;
                Valve_Test_Set_Data(airbag);
            }
            else
            {
                airbag = 1 << airbagIndex;
                Valve_Test_Set_Data(airbag);
            }
            if(airpumpIndex & 0x03)
            {
                Valve_LegFootAirPumpACPowerOn(); //小腿和臀部
            }
            else
            {
                Valve_LegFootAirPumpACPowerOff(); //小腿和臀部
            }
            Valve_Send_Data();
            break;
        case LINGO_ENG:
        case LINGO_MENU:
            engineering_stop_all();
            RollerMotor_Control(ROLLER_SPEED_STOP, 0);
            KneadMotor_Control(STATE_KNEAD_IDLE, KNEAD_SPEED0_PWM);
            Valve_LegFootAirPumpACPowerOff(); //小腿和臀部
            Valve_Test_Set_Data(0);
            {
                if(bMasterSendPacket)
                {
                    OutBuffer[0] = SOI ;
                    unsigned int snH = DEVINFO->UNIQUEH;
                    unsigned int snL = DEVINFO->UNIQUEL;
                    OutBuffer[1] = (unsigned char)(snH >> 24);
                    OutBuffer[2] = (unsigned char)(snH >> 16);
                    OutBuffer[3] = (unsigned char)(snH >> 8);
                    OutBuffer[4] = (unsigned char)(snH);
                    OutBuffer[5] = (unsigned char)(snL >> 24);
                    OutBuffer[6] = (unsigned char)(snL >> 16);
                    OutBuffer[7] = (unsigned char)(snL >> 8);
                    OutBuffer[8] = (unsigned char)(snL);
                    OutBuffer[9] = (unsigned char)ReadEEByte(USER_DATA_BASE + SOFT_MAIN_VER_ADDRESS);
                    OutBuffer[10] = (unsigned char)ReadEEByte(USER_DATA_BASE + SOFT_SECONDARY_VER_ADDRESS);
                    OutBuffer[11] = (unsigned char)ReadEEByte(USER_DATA_BASE + SETTLE_ADDRESS);
                    OutBuffer[12] = (unsigned char)ReadEEByte(USER_DATA_BASE + AIRBAG_STRETCH_ADDRESS);
                    OutBuffer[13] = (unsigned char)ReadEEByte(USER_DATA_BASE + SLIDE_MOTOR_ENABLE_ADDRESS);
                    strengthMode = *(pInformation + STRETCH_OUT_ADDRESS);
                    OutBuffer[11] |= (strengthMode << 1);
                    sleepMode = *(pInformation + SLEEP_PROGRAM_ADDRESS);
                    OutBuffer[11] |= (sleepMode << 2);
                    // OutBuffer[14] = (unsigned char)ReadEEByte(USER_DATA_BASE+ACC_TIME_0_ADDRESS);
                    // OutBuffer[15] = (unsigned char)ReadEEByte(USER_DATA_BASE+ACC_TIME_1_ADDRESS);
                    // OutBuffer[16] = (unsigned char)ReadEEByte(USER_DATA_BASE+ACC_TIME_2_ADDRESS);
                    // OutBuffer[17] = (unsigned char)ReadEEByte(USER_DATA_BASE+ACC_TIME_3_ADDRESS);
                    OutBuffer[14] = EOI ;
                    nOutBufferCount = 15;
                    DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
                    bMasterSendPacket = FALSE ;
                }
            }
            break;
        case LINGO_PROGRAM:
        {
            Valve_LegFootAirPumpACPowerOff(); //小腿和臀部
            Valve_Test_Set_Data(0);
            if(bMasterSendPacket)
            {
                if(bProgram)
                {
                    OutBuffer[0] = SOI ;
                    OutBuffer[1] = 'p';
                    OutBuffer[2] = 'r';
                    OutBuffer[3] = 'o' ;
                    OutBuffer[4] = 'g' ;
                    OutBuffer[5] = 'r' ;
                    OutBuffer[6] = 'a' ;
                    OutBuffer[7] = 'm' ;
                    OutBuffer[8] = EOI ;
                    nOutBufferCount = 9;
                    DMAUart_Transmit_Packet(OutBuffer, nOutBufferCount);
                    overCounter++;
                    if(overCounter >= 3)
                    {
                        password = 0;
                        NVIC_SystemReset(); //复位CPU
                    }
                }
                bMasterSendPacket = FALSE ;
            }
        }
        break;
        default:
            Valve_LegFootAirPumpACPowerOff(); //小腿和臀部
            Valve_Test_Set_Data(0);
            break;
        }
        /******************************/
        if(DMAUart_GetCtrlType() != ENGGER_CTRL)
        {
            password = 0;
            NVIC_SystemReset(); //复位CPU
        }
    }
    Main_Initial_Data(); //重新初始化数据
}
//UART 测试程序 需要借助电脑的RS232duan
unsigned char     welcomeString[]  = "Test kdjfhkshdkgjsakljglsjelUART\n";

//时钟测试程序 需要借助示波器
void Main_Clock_Test(void)
{
    bool bStatus = TRUE;

    CMU_HFRCOBandSet(cmuHFRCOBand_21MHz);  /** main system clock - internal RC 28MHz*/
    SystemCoreClockUpdate();
    CMU_ClockEnable(cmuClock_HFPER, true); /** High frequency peripheral clock */
    CMU_ClockEnable(cmuClock_CORELE, true);/* Enable CORELE clock */
    CMU_ClockEnable(cmuClock_GPIO, true);  /** General purpose input/output clock. */

    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
    GPIO_PinModeSet(gpioPortA, 1, gpioModePushPull, 0);
    GPIO_PinModeSet(gpioPortA, 2, gpioModePushPull, 0);

    CMU->ROUTE = CMU_ROUTE_CLKOUT0PEN | CMU_ROUTE_CLKOUT1PEN | CMU_ROUTE_LOCATION_LOC0;


    while(bStatus)
    {
        WDOG_Feed();
    }
}

void Main_Auto_Program_Test(void)
{
    for(int j = 0; j < 6; j++)
    {
        for(int i = 0; i < BACK_AUTO_STEPS[j]; i++)
        {
            switch(j)
            {
            case 0:
                AutoDirector = AutoFunction0[i] ;
                break;
            case 1:
                AutoDirector = AutoFunction1[i] ;
                break;
            case 2:
                AutoDirector = AutoFunction2[i] ;
                break;
            case 3:
                AutoDirector = AutoFunction3[i] ;
                break;
            case 4:
                AutoDirector = AutoFunction4[i] ;
                break;
            case 5:
                AutoDirector = AutoFunction5[i] ;
                break;
            }

            switch(AutoDirector.nSubFunction)
            {
            case BACK_SUB_MODE_KNEAD:
            {
                if(AutoDirector.nKneadMotorState == KNEAD_STOP)
                {
                    printf("auto%d-KNEAD-step:[%d]\n", j, i);
                }
            }
            break;

            case BACK_SUB_MODE_KNOCK:
            {
                if(AutoDirector.nKnockMotorState == KNOCK_STOP)
                {
                    printf("auto%d-KNOCK-step:[%d]\n", j, i);
                }
            }
            break;

            case BACK_SUB_MODE_WAVELET:
            {
                if(AutoDirector.nKnockMotorState == KNOCK_STOP ||
                        AutoDirector.nKneadMotorState == KNEAD_STOP )
                {
                    printf("auto%d-WAVELET-step:[%d]\n", j, i);
                }
            }
            break;
            case BACK_SUB_MODE_SOFT_KNOCK:
            {
                if(AutoDirector.nKnockMotorState != KNOCK_RUN_STOP)
                {
                    printf("auto%d-SOFT_KNOCK-step:[%d]\n", j, i);
                }
            }
            break;

            case BACK_SUB_MODE_PRESS:
            {
                if(AutoDirector.nKnockMotorState == KNOCK_RUN ||
                        AutoDirector.nKneadMotorState == KNEAD_RUN )
                {
                    printf("auto%d-PRESS-step:[%d]\n", j, i);
                }
            }
            break;
            case BACK_SUB_MODE_RUBBING:
            {
                if(AutoDirector.nKneadMotorState != KNEAD_RUN_RUBBING)
                {
                    printf("auto%d-RUBBING-step:[%d]\n", j, i);
                }
            }
            break ;
            case BACK_SUB_MODE_MUSIC:
            default:
                printf("error[%d]\n", j);
                break;
            }
        }
    }
    while(1);
}

void Main_Save_Acctime(void)
{
    unsigned int totalTime;
    totalTime = Data_Get_ProgramExecTime();
    if(totalTime == 0) return;
    Data_Clear_ProgramExecTime();
    totalTime += MEM_Get_Total_Time();
    MEM_Save_Total_Time(totalTime);
}

void Main_Demo(void)
{
    int demoStep = 0;
    int demoCounter = 0;
    nChairRunDemoCount=0;//fww
    nChairRunState = CHAIR_STATE_DEMO;
    bKeyPowerSwitch = TRUE ;
    Data_Set_Start(1, RUN_TIME_120);
    
#ifdef FOOT_ROLLER_ENABLE
    bRollerEnable = TRUE;
    if(nRollerPWM == 0)
    {
        nRollerPWM = 2;
        Valve_SetRollerPWM(nRollerPWM);
    }
#endif
    nIndicateTimer = RUN_INDICATE_TIME;
    /////////////////////
    //设置背部功能
    
    //bBodyDetectSuccess = TRUE ; //禁止检测体型
    nReworkShoulderPosition = 0;
    nBackMainRunMode = BACK_MAIN_MODE_AUTO ;
    nBackSubRunMode = BACK_SUB_MODE_AUTO_0 ;
    bBackAutoModeInit = TRUE ;
    //设置气囊功能
    nKeyAirBagLocate = AIRBAG_LOCATE_AUTO ;
    Valve_SetAirBagStrength(AIRBAG_STRENGTH_3);
    st_AirBagAuto0.init = TRUE ;//Fungares
    Timer_Counter_Clear(C_TIMER_TEMP);
    while(nChairRunState == CHAIR_STATE_DEMO)
    {
        Power_All_On();
        WDOG_Feed();
        if(Timer_Counter(C_TIMER_INDICATE + T_LOOP, nIndicateTimer))
        {
            IndicateLED_Toggle();
        }
        Valve_Send_Data();
        Input_Proce();
        LED_RGB_Proce(nChairRunState);
        //10ms变量更新
        if(bTimer10MS == TRUE)
        {
            bTimer10MS = FALSE ;
            Main_10ms_Proce();
            nTimer100MS++ ;
            if(nTimer100MS >= 10)
            {
                nTimer100MS = 0 ;
                Main_100ms_Proce();
            }
            nTimer500MS++ ;
            if(nTimer500MS >= 50)
            {
                nTimer500MS = 0 ;
                Main_500ms_Proce();
            }
        }
        //蜂鸣器的处理
        Main_Walk_Beep_Proce();
        //靠背升降电机手动处理
        Main_BackPad_Proce();
        //小腿升降电机手动处理
        Main_LegPad_Proce();
        //小腿伸缩电机手动处理
        //Main_FlexPad_Proce();//fww

        Main_Massage_Position_Proce();

        //FlexMotorFollowingFood();

        main_GetKneadPosition();

        if(Data_Time_Counter_Proce())
        {
            demoCounter++;
        }
       
        if(Timer_Counter(C_TIMER_TEMP + T_LOOP,5*60*10))
        //if(Timer_Counter(C_TIMER_TEMP + T_LOOP,20))
        {
          engineering_stop_all();
          Main_Save_Acctime();   //在这里记录累计时间
        }
        
        if(Data_Get_Time() == 0)
        {
            engineering_stop_all();
            Main_Save_Acctime();   //在这里记录累计时间
            password = 0;
            NVIC_SystemReset();
        }
        //背部功能处理
        Main_BackProce();
        //不管行走电机是否完成自己的动作，都要时刻监控行程开关，以免冲击行程
        WalkMotorControl(nWalkMotorControlParam1, nWalkMotorControlParam2) ;
        //揉捏电机完成步骤要求动作后，无论其它电机完成与否，都不再处理
        KneadMotorControl(nKneadMotorControlParam1, nKneadMotorControlParam2) ;
        //捶击电机完成步骤要求动作后，无论其它电机完成与否，都不再处理
        KnockMotorControl(nKnockMotorControlParam1, nKnockMotorControlParam2, nKnockMotorControlParam3) ;
        //气囊部分的处理
        /*******************************************/
        //气阀处理
        Main_Valve_Proce();

        MusicSampling();
        //加热处理
        if(bKeyWaistHeat == TRUE)
        {
            WaistHeat_On();
        }
        else
        {
            WaistHeat_Off();
        }
        /******错误状态处理****************************/
        //ExceptionHandles();
        /**********************************/
        //通信处理
        //CommProcess() ;
        switch(demoStep)
        {
        case 0:
            nTargetMassagePosition = MASSAGE_MAX_POSITION;
            bMassagePositionUpdate = TRUE;
            //demoCounter = 0;
            demoStep++;
            break;
        case 1:
            if(!bMassagePositionUpdate) demoStep++;
            break;
        case 2:
            nTargetMassagePosition = MASSAGE_RESET_POSITION;
            bMassagePositionUpdate = TRUE;
            demoStep++;
            break;
        case 3:
            if(!bMassagePositionUpdate)
            {
              demoStep++;
              nTargetMassagePosition = MASSAGE_OPTIMAL_POSITION;
              bMassagePositionUpdate = TRUE;
            }
            nChairRunDemoCount=0;
            break;
        case 4:
          if(nChairRunDemoCount>=240)
          {
            demoStep++;
          }
          break;
        case 5:
        default:
            demoStep = 0;
            break;
        }

        BYTE key = Main_GetKey();
        switch(key)
        {
        case H10_KEY_POWER_SWITCH:
        {
          
          engineering_stop_all();
          Main_Save_Acctime();   //在这里记录累计时间
            password = 0;
            NVIC_SystemReset();
        }
        break;
        case H10_KEY_ZERO_START:
        {
          bKeyWaistHeat = ~bKeyWaistHeat;
        }
        break;
        }
        Main_Send();
        Main_BlueToothSend();
    }
    bDemoRun = false;
    password = 0;
    NVIC_SystemReset();
}

void Main_Sleep(void)
{
	bool bPowerOn = false;
	int powerCounter = 0;
	int ledCounter;
	BYTE key;
	Power_Off();
	Power_3V3_Off();
	Main_Save_Acctime();   //在这里记录累计时间
	nChairRunState = CHAIR_STATE_SLEEP;
	while(nChairRunState == CHAIR_STATE_SLEEP)
	{
		if(DMAUart_GetCtrlType() == ENGGER_CTRL)
		{
			nChairRunState = CHAIR_STATE_ENGINEERING;
			return;
		}
		key = Main_GetKeyNoClear();
		if(key != H10_KEY_NONE)
		{
			if( key == H10_KEY_POWER_SWITCH ||
				key == H10_KEY_BACKPAD_UP_START ||
				key == H10_KEY_BACKPAD_DOWN_START ||
				key == H10_KEY_LEGPAD_UP_START ||
				key == H10_KEY_LEGPAD_DOWN_START /*||
				key == H10_KEY_LEGPAD_EXTEND_START ||
				key == H10_KEY_LEGPAD_CONTRACT_START*/)
			{
				bPowerOn = true;
				Power_All_On();
			}
		}
		if(bTimer10MS == TRUE)
		{
			ledCounter++;
			ledCounter %= 200;
			bTimer10MS = FALSE ;
			if(bPowerOn)
			{
				powerCounter++;
				if(powerCounter > 4)
				{
					nChairRunState = CHAIR_STATE_IDLE;
				}
			}
			else
			{
				powerCounter = 0;
			}
		}
		if(ledCounter < 10)
		{
			IndicateLED_On();
		}
		else
		{
			IndicateLED_Off();
		}

		if(bPowerOn)
		{
			Input_Proce();
			Valve_Send_Data();
		}
		Main_Send();
		Main_BlueToothSend();
		//
		//141116
		nKneadBalanceCounter = 0;
	}
}

/*
*@brief  :
*@param  :no
*@retval :no
*********************************************/
//MOTOR_RUN_DIS_PROCESS(bResetStateORchangeModeDis,bSelectKeyValue)
/************************fww************************************
void MOTOR_RUN_DIS_PROCESS(unsigned char ConfirmKey)
{
  if(bResetStateORchangeModeDis == EnterDisReset) //Enter reset process and display program
  {
    switch(ConfirmKey)
    {
      //0
    case KEYCANCLE:
      if(nChairResetReason == POWERKEY_PRESSED)
      {
        //when cancle ,then make the massage chair to the chair_run state
        nChairRunState = CHAIR_STATE_RUN ;
        //other resetreason do not allow pressing any key to enter "CHAIR_STATE_SETTLE_1ST" 
        nTimerOverResetAgainFlag = FIRST_TIME_ENTER;
        bKeyPowerSwitch = TRUE ;
        //140530
        nBackMainRunMode = nCurBackMainRunModeStore ;
        bBackAutoModeInit = TRUE ;
        nIndicateTimer = RUN_INDICATE_TIME;
        switch(nBackMainRunMode)
        {
          //0
        case BACK_MAIN_MODE_AUTO:
          nBackSubRunMode = nCurBackSubRunModeStore ;
          //reconfigure the params of airbag
          nKeyAirBagLocate = nKeyAirBagLocateStore ; //AIRBAG_LOCATE_AUTO ;
          
          if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
          {
            Valve_SetAirBagStrength(AIRBAG_STRENGTH_3);
          }
          //do not recheck shoulder again
          nReworkShoulderPosition = 0;
          //turn on all AirPump
          if(nKeyAirBagLocate != AIRBAG_LOCATE_NONE)
          {
            Valve_BodyUpAirPumpACPowerOn();
            Valve_LegFootAirPumpACPowerOn();
          }
          //reload the previous runtime
          Data_Set_Start(1, w_PresetTimeStore);
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          
          
          break;
          //1
        case BACK_MAIN_MODE_MANUAL:
          nMaunalSubMode = nCurMaunalSubModeStore ;
          //reconfirm the motor's speed,if it's 0 ,then let the speed level 2
          if(nKeyKneadKnockSpeed == SPEED_0)
          {
            nKeyKneadKnockSpeed = SPEED_2 ;
          }
          //reconfigure the params of airbag
          nKeyAirBagLocate = nKeyAirBagLocateStore ;
          if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
          {
            Valve_SetAirBagStrength(AIRBAG_STRENGTH_3);
          }
          //turn on all AirPump
          if(nKeyAirBagLocate != AIRBAG_LOCATE_NONE)
          {
            Valve_BodyUpAirPumpACPowerOn();
            Valve_LegFootAirPumpACPowerOn();
          }
          //recovery the the motorcore's part
          nKeyBackLocate = nKeyBackLocateStore ;
          //manual mode ,continue the next step
          bGetNextActionStep = TRUE ;
          //reload the previous runtime
          Data_Set_Start(1, w_PresetTimeStore);
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
          
          break;
          //2
        default :
          //bKeyPowerSwitch = TRUE ;
          //nChairRunState = CHAIR_STATE_WAIT_COMMAND ;//按摩椅等待按键命令
          nChairStateCount = 0 ;
          break;
        }
        if(nKeyAirBagLocateStore != AIRBAG_LOCATE_NONE)
        {
          nKeyAirBagLocate = nKeyAirBagLocateStore ;
          if( Valve_GetAirBagStrength() == AIRBAG_STRENGTH_0)
          {
            Valve_SetAirBagStrength(AIRBAG_STRENGTH_3);
          }
          Data_Set_Start(1, w_PresetTimeStore);
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
        }
        if(bKeyWaistHeatStore == TRUE)
        {
          bKeyWaistHeat = TRUE;
          Data_Set_Start(1, w_PresetTimeStore);
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
        }
        if(bRollerEnableStore == TRUE)
        {
          bRollerEnable = TRUE ;
          nRollerPWM = nRollerPWMStore ;
          Valve_SetRollerPWM(nRollerPWM);
          Data_Set_Start(1, w_PresetTimeStore);
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
          bBlueToothSendBuzzerMode = TRUE;
        }
        if((bRollerEnableStore == FALSE)&&(bKeyWaistHeatStore == FALSE)&&
           (nKeyAirBagLocateStore == AIRBAG_LOCATE_NONE)&&(nBackMainRunMode != BACK_MAIN_MODE_AUTO)&&
             (nBackMainRunMode != BACK_MAIN_MODE_MANUAL))
        {
          bKeyPowerSwitch = TRUE ;
          nChairRunState = CHAIR_STATE_WAIT_COMMAND ;//按摩椅等待按键命令
          nChairStateCount = 0 ;
        }
        
        //bMassagePositionUpdate = FALSE ;
      }
      else if(nChairResetReason == TIMEOVER)
      {
        nChairRunState = CHAIR_STATE_IDLE ;
        nIndicateTimer = IDLE_INDICATE_TIME;
        //140625
        nKeyAirBagLocateStore = AIRBAG_LOCATE_NONE ;
        bKeyWaistHeatStore = FALSE ;
        bRollerEnableStore = FALSE ;
        nRollerPWM = 0 ;
        Valve_SetRollerPWM(nRollerPWM);
        
        nTimerOverResetAgainFlag = SECOND_TIME_ENTER;
        bMassagePositionUpdate = FALSE ;
        bKeyPowerSwitch = FALSE ;
      }
      else if(nChairResetReason == CHANGE_MODE)
      {
        nChairRunState = CHAIR_STATE_RUN ;
        bKeyPowerSwitch = TRUE ;
        bMassagePositionUpdate = FALSE ;
      }
      else 
      {
        nChairRunState = CHAIR_STATE_IDLE ;
        nTimerOverResetAgainFlag = FIRST_TIME_ENTER;
      }
      bResetStateORchangeModeDis = ExitDisReset ;
      bSelectKeyValue = KEY_NOT_CONFIRMORCANCLE ;
      //bBackLegPadSettle = FALSE ;
      
      break;
      //1
    case KEYCONFIRM:
      if(bResetFlag == RESET_CHAIR)
      {
        //display in reset state 1
        nChairRunState = CHAIR_STATE_SETTLE ;
        
      }
      if(bResetFlag == CHANGE_WORKMODE)
      {
        //when in workmode change, make the handcontroller display resetstate's language
        nChairRunState = CHAIR_STATE_SETTLE ;
      }
      if(nWaitCustomActionTime >= 10)
      {
        bResetStateORchangeModeDis = ExitDisReset ;
        bSelectKeyValue = KEY_NOT_CONFIRMORCANCLE ;
        
        if(bResetFlag == RESET_CHAIR)
        {
          //reset the massage chair back to it's original position
          bBackLegPadSettle = TRUE ;
          nTargetMassagePosition = MASSAGE_RESET_POSITION;
          bMassagePositionUpdate = TRUE;
          //reset the datas which was stored 
          nKeyAirBagLocateStore = AIRBAG_LOCATE_NONE ;
          bKeyWaistHeatStore = FALSE ;
          bRollerEnableStore = FALSE ;
          nRollerPWM = 0 ;
          Valve_SetRollerPWM(nRollerPWM);
        }
        
        if(nChairResetReason == CHANGE_MODE)
        {
          nChairRunState = CHAIR_STATE_RUN ;
        }
      }
      
      break;
      //2
    case KEY_NOT_CONFIRMORCANCLE:
      switch(bResetFlag)
      {
        //0
      case NORESET_CHAIR:
        //display in reset IDLE state
        nChairRunState = CHAIR_STATE_IDLE ;
        bResetStateORchangeModeDis = ExitDisReset ;
        
        break;
        //1
      case RESET_CHAIR:
        //display in reset state 2
        nChairRunState = CHAIR_STATE_SETTLE_1ST ;
        //
        /////////////////////fww////////////////////////
        if((nBuzzer100msFlag == TRUE) && (nSendBuzzerTimes < 3))
        {
          nBuzzer100msFlag = FALSE ;
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
        }
        else
        {
          nBuzzerMode = BUZZER_MODE_OFF ;
          bSendBuzzerMode = TRUE ;
        }  
        /////////////////////////fww//////////////////////////////////
        
        break;
        //2
      case CHANGE_WORKMODE:
        //display in reset state 2
        nChairRunState = CHAIR_STATE_SETTLE_1ST ;
        //
         //////////////////////////fww/////////////////////////
        if((nBuzzer100msFlag == TRUE) && (nSendBuzzerTimes < 3))
        {
          nBuzzer100msFlag = FALSE ;
          nBuzzerMode = BUZZER_MODE_ONETIME ;
          bSendBuzzerMode = TRUE ;
        }
        else
        {
          nBuzzerMode = BUZZER_MODE_OFF ;
          bSendBuzzerMode = TRUE ;
        }
        ////////////////////////////fww/////////////////////////// 
        break;
        //
      default :
        break;
      }
      
      //trun off all function of the massage chair
      PowerOff_Pause_all();
      
      break;
      //
    default :
      break;
    }
  }
}
*********************fww************************/

extern unsigned short __checksum;

#define PLANT_ADDR_BASE          ((uint32_t) 0x0001f800UL)//

unsigned char AES_PlantTest1[16] = "Rongtai Health";

void AES_ECB_128bit_Encrypt(void)
{
  unsigned char AES_PlantTest2[16];
  unsigned char AES_PlantTest3[16];

  for(unsigned char i = 0; i < 16; i++)
  {

    AES_PlantTest2[i] =0;

    AES_PlantTest3[i] =0;
    
  }
  
  unsigned char g_ucKey[16] =
  {
      0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89,
      0x9A, 0xAB, 0xBC, 0xCD, 0xDE, 0xEF, 0xF0, 0x00
  };  
  
    unsigned int PlantAddr = PLANT_ADDR_BASE;
       
    unsigned int snH = DEVINFO->UNIQUEH;
    unsigned int snL = DEVINFO->UNIQUEL;
    
    for(unsigned char i = 0; i < 16; i++)
    {    
        AES_PlantTest2[i] = ReadEEByte(PlantAddr);
        PlantAddr++;
    }
     
    g_ucKey[1] = (unsigned char)(snH >> 24);
    g_ucKey[2] = (unsigned char)(snH >> 16);
    g_ucKey[3] = (unsigned char)(snH >> 8);
    g_ucKey[4] = (unsigned char)(snH);
    
    g_ucKey[5] = (unsigned char)(snL >> 24);
    g_ucKey[6] = (unsigned char)(snL >> 16);
    g_ucKey[7] = (unsigned char)(snL >> 8);
    g_ucKey[8] = (unsigned char)(snL);
    
    //AES_ECB128(AES_PlantTest2,AES_PlantTest1,16,g_ucKey,true);
    
    AES_DecryptKey128(g_ucKey,g_ucKey);
    
    AES_ECB128(AES_PlantTest3,AES_PlantTest2,16,g_ucKey,false);
    if(strcmp(AES_PlantTest1,AES_PlantTest3) != 0)
    {
        while(1);
    }
}

void main()
{
	SCB->VTOR = (uint32_t)(8 * 1024);
	if (__checksum == 0) __checksum = 1;
	Main_Initial_IO();    //hardware initial
	WDOG_Init(&WDTinit);  //WDT initial
	if(password == 0x55AA1234)
	{
		goto program_start;
	}
	Power_All_On();
	Power_AMP_On();
	Main_Initial_Data();  //software initial
	nIndicateTimer = IDLE_INDICATE_TIME;
	nChairRunState = CHAIR_STATE_IDLE;  //1122开机进入待运行模式
program_start:
	password = 0x55AA1234;
	AES_ECB_128bit_Encrypt();//fww 加密
	while(1)
	{
		/****************公共状态区域*********************/
		Power_All_On();
		WDOG_Feed();
		if(Timer_Counter(C_TIMER_INDICATE + T_LOOP, nIndicateTimer))
		{
			IndicateLED_Toggle();
		}
		#ifdef TEST_ON
		if(nChairRunState == CHAIR_STATE_IDLE)
		{
			DMAUart_SetKey(H10_KEY_POWER_SWITCH);
			DMAUart_SetRXStatus();
		}
		if(nChairRunState == CHAIR_STATE_WAIT_COMMAND)
		{
			DMAUart_SetKey(H10_KEY_WAVELET);
			DMAUart_SetRXStatus();
		}
		#endif
		Valve_Send_Data();
		Input_Proce();
		if(bDemoRun)
		{
			Main_Demo();
		}
		LED_RGB_Proce(nChairRunState);
		//10ms变量更新
		if(bTimer10MS == TRUE)
		{
			bTimer10MS = FALSE ;
			Main_10ms_Proce();
			nTimer100MS++ ;
			if(nTimer100MS >= 10)
			{
				nTimer100MS = 0 ;
				Main_100ms_Proce();
			}
			nTimer500MS++ ;
			if(nTimer500MS >= 50)
			{
				nTimer500MS = 0 ;
				Main_500ms_Proce();
			}
		}
		//蜂鸣器的处理
		Main_Walk_Beep_Proce();
		//靠背升降电机手动处理
		Main_BackPad_Proce();
		//小腿升降电机手动处理
		Main_LegPad_Proce();
		//小腿伸缩电机手动处理
		//Main_FlexPad_Proce();//fww
      
		Main_Massage_Position_Proce();
      
		//FlexMotorFollowingFood();
		//调整敲击电机的位置（在指压，揉捏，揉搓，以及韵律无音乐时，即是敲击电机不动作时）
		//TapMotor_Position_Proce();
      
		//PowerOff_1st_3second_backMotor_Process
		//Main_BackMotor_PowerOff_Up();
		//140528 the process for resetting's display and massage chair
		//MOTOR_RUN_DIS_PROCESS(bSelectKeyValue);
      
		/****************公共状态区域*结束****************/
		//电源关闭时间为5秒
		if(Timer_Counter(C_TIMER_POWER + T_LOOP, 30) || (nChairRunState == CHAIR_STATE_SLEEP))
		{
			//Power_Off();
			Main_Sleep();
		}
		//mp3按键释放时间 0.3秒
		if(Timer_Counter(C_TIMER_MP3, 3))
		{
			MP3Control1_Clear();
		}
		//快闪时间处理 0.3秒
		if(Timer_Counter(C_TIMER_FAST + T_LOOP, 3))
		{
			bFastDisplayFlash = ~bFastDisplayFlash ;
		}
		//慢闪时间处理 0.8秒
		if(Timer_Counter(C_TIMER_SLOW + T_LOOP, 8))
		{
			//MP3Power_Toggle();
			bSlowDisplayFlash = ~bSlowDisplayFlash ;
		}
      
		if(nChairRunState == CHAIR_STATE_ENGINEERING)
		{
			Main_Engineering();
		}
      /*if(bKeySeatVibrate)
      {
      if(Timer_Counter(C_TIMER_WAVE_START, 30))
      {
      bKeySeatEnable = TRUE;
    }
    }
      else
      {
      Timer_Counter_Clear(C_TIMER_WAVE_START);
      bKeySeatEnable = FALSE;
    }*/
      
      /*****************/
		if(nChairRunState == CHAIR_STATE_SETTLE)
		{
			//设置连续蜂鸣器声音
			if(bBackLegPadSettle == TRUE)
			{
				nBuzzerMode = BUZZER_MODE_SLOW ;
				bSendBuzzerMode = TRUE ;
				bBlueToothSendBuzzerMode = TRUE;
			}
		}
		else if((bKeyBackPadUp == FALSE) && (bKeyBackPadDown == FALSE) &&
			(bKeyLegPadUp == FALSE) && (bKeyLegPadDown == FALSE) &&
			(bKeyWalkUp == FALSE) && (bKeyWalkDown == FALSE) /*&&
			(bKeyFlexOut == FALSE) && (bKeyFlexIn == FALSE)*/)
		{
			if((nBuzzerMode == BUZZER_MODE_FAST) || (nBuzzerMode == BUZZER_MODE_SLOW))
			{
				if(bErrorOverFlag == FALSE)
				{
					nBuzzerMode = BUZZER_MODE_OFF ;
					bSendBuzzerMode = TRUE ;
					bBlueToothSendBuzzerMode = TRUE;
				}
			}
		}
		//收藏功能的处理
		if(bBackLegPadSettle == TRUE)
		{
			//if(bReachBackPadUpPosition == TRUE)
			if(Input_GetBackUpSwitch() != 0)
			{
				bKeyBackPadUp = FALSE ;
				bKeyBackPadDown = FALSE ;
			}
			if(Input_GetLegDownSwitch() != 0)
			{
				bKeyLegPadUp = FALSE ;
				bKeyLegPadDown = FALSE ;
			}
			if((Input_GetBackUpSwitch() != 0) && (Input_GetLegDownSwitch() != 0))
			{
				bBackLegPadSettle = FALSE ;
			}
		}
      /*
      //检测到了小腿侧的感应器时，关闭电动缸的功能
      if(Sensor_Check() == TRUE)
      {
      bKeyLegPadUp = FALSE ;
      bKeyLegPadDown = FALSE ;
      //
      bKeyBackPadUp = FALSE ;
      bKeyBackPadDown = FALSE;
      bLegPadLinkage = FALSE ;
      //
      //bKeyFlexOut = FALSE ;
      if(bKeyFlexIn == TRUE)
      {
      bKeyFlexIn = FALSE ;
      FlexMotor_Control(STATE_FLEX_IDLE, FLEX_SPEED_FAST, FLEX_CURRENT_3A_1);
    }
      FlexMotorSetDisable();
      //
      if(bMassagePositionUpdate == 1)
      {
      bMassagePositionUpdate = 0;
      nTargetMassagePosition = MASSAGE_RESET_POSITION;
    }
      //
      if(st_Stretch.active == TRUE)
      {
      st_Stretch.active = FALSE;
      Valve_SetStretchUp();
    }
      //
      LegMotor_Control(STATE_LEG_IDLE);
      BackMotor_Control(STATE_BACK_IDLE);
      SlideMotorControl(STATE_SLIDE_IDLE);
      //如果在复位过程中，则切换状态到待机状态
      if(nChairRunState == CHAIR_STATE_SETTLE)
      {
      nChairRunState = CHAIR_STATE_IDLE;
      nIndicateTimer = IDLE_INDICATE_TIME;
    }
    }
      */
		/***********************************/
		// Main_Shoulder_Detect();
      
		main_GetKneadPosition();
      
		Data_Time_Counter_Proce();
      
		#ifdef   BACK_MODE
		//背部功能处理
		Main_BackProce();
		//不管行走电机是否完成自己的动作，都要时刻监控行程开关，以免冲击行程
		WalkMotorControl(nWalkMotorControlParam1, nWalkMotorControlParam2) ;
		//揉捏电机完成步骤要求动作后，无论其它电机完成与否，都不再处理
		KneadMotorControl(nKneadMotorControlParam1, nKneadMotorControlParam2) ;
		//捶击电机完成步骤要求动作后，无论其它电机完成与否，都不再处理
		KnockMotorControl(nKnockMotorControlParam1, nKnockMotorControlParam2, nKnockMotorControlParam3) ;
		#endif
		//气囊部分的处理
		/*******************************************/
		//气阀处理
		#ifndef TEST_VALVE
		Main_Valve_Proce();
		#endif
		/****************************/
		//振动(摇摆)处理
		VibrateMotorControl() ;
 		/*
		if(nChairRunState == CHAIR_STATE_RUN)
		{
			RunTimeRefresh(bKeyPowerSwitch,Data_Get_Time());
		}
		*/
		// ZLB_MotorRunController();
		MusicSampling();
		//ZeroMotorController();
		//加热处理
		if(bKeyWaistHeat == TRUE)
		{
			WaistHeat_On();
		}
		else
		{
			WaistHeat_Off();
		}
		/*
		if(Input_GetWaveMotorPosition() == 0)
		{
			nWaveOverTime = 0;
		}
		*/
		/******错误状态处理****************************/
		//ExceptionHandles();
		/**********************************/
		//通信处理
		CommProcess() ;
		//摇摆处理
		//RockProcess();
		#ifndef TEST_VALVE
		if(nChairRunState == CHAIR_STATE_IDLE)
		{
			if((LegMotor_GetPower() == LEG_MOTOR_POWER_ON) ||
				(BackMotor_GetPower() == BACK_MOTOR_POWER_ON) ||
				(FlexPower_Get() == FLEX_POWER_ON) ||
				(WalkPower_Get() == WALK_MOTOR_POWER_ON))
			{
				Timer_Counter_Clear(C_TIMER_POWER);
			}
			/*
			else
			{
			Main_Save_Acctime();
			}
			*/
		}
		else
		{
			Timer_Counter_Clear(C_TIMER_POWER);
		}
		#else
		Timer_Counter_Clear(C_TIMER_POWER);
		Power_On();
		#endif
	} //end while
} //end main loop

//电机参数设置
void refreshAutoDirector(void)
{
	nCurSubFunction = AutoDirector.nSubFunction ;
	nCurKneadKnockSpeed = AutoDirector.nKneadKnockSpeed ;
	//设置行走电机
	bWalkMotorInProcess = TRUE ;
	bUpdateLocate = TRUE ;
	nWalkMotorControlParam1 = AutoDirector.nWalkMotorLocateMethod ;
	nWalkMotorControlParam2 = AutoDirector.nWalkMotorLocateParam ;
	//设置揉捏电机
	bKneadMotorInProcess = TRUE ;
	nKneadMotorControlParam1 = AutoDirector.nKneadMotorState ;
	nKneadMotorControlParam2 = AutoDirector.nKneadMotorCycles ;
	//设置捶击电机
	bKnockMotorInProcess = TRUE ;
	nKnockMotorControlParam1 = AutoDirector.nKnockMotorState ;
	nKnockMotorControlParam2 = AutoDirector.nKnockMotorRunTime ;
	nKnockMotorControlParam3 = AutoDirector.nKnockMotorStopTime ;
}
