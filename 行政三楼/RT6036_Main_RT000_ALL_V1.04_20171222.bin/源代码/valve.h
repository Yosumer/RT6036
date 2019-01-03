#ifndef __VALVE_H__
#define __VALVE_H__
#include "efm32_def.h"
#include "efm32_types.h"
#define PUMP_ON				0
#define PUMP_OFF			1
#define VALVE_ON			1
#define VALVE_OFF			0

#define VALVE_DISABLE 0
#define VALVE_ENABLE  1

#define PV1		0x00000001UL     //上半身
#define PV2		0x00000002UL
#define PV3		0x00000004UL
#define PV4	        0x00000008UL
#define PV5	        0x00000010UL
#define PV6	        0x00000020UL
#define PV7	        0x00000040UL
#define PV8		0x00000080UL

#define PV11	                0x00000100UL    //下半身
#define PV12     	        0x00000200UL
#define PV13	                0x00000400UL
#define PV14	                0x00000800UL
#define PV15	                0x00001000UL
#define PV16	                0x00002000UL


//0-7位为足部气囊
#define F_BACK		        0x00000001UL
#define F_L_SIDE		0x00000002UL
#define F_R_SIDE		0x00000004UL
#define F_HEEL		        0x00000008UL
#define L_SIDE		        0x00000010UL
#define LEG		        0x00000020UL
#define L_B_UP		        0x00000040UL
#define L_B_DOWN		0x00000080UL
//8-15位为手臂气囊和两个气泵
#define R_ARM_1	                0x00000100UL
#define R_ARM_2     	        0x00000200UL
#define R_ARM_3	                0x00000400UL
#define L_ARM_1	                0x00000800UL
#define L_ARM_2	                0x00001000UL
#define L_ARM_3	                0x00002000UL

#define LEFT_ARM_1_CHR    L_ARM_1
#define LEFT_ARM_2_CHR    L_ARM_2
#define LEFT_ARM_3_CHR    L_ARM_3

#define RIGHT_ARM_1_CHR   R_ARM_1
#define RIGHT_ARM_2_CHR   R_ARM_2
#define RIGHT_ARM_3_CHR   R_ARM_3

#define PE1	                0x00008000UL
#define PE2	                0x00008000UL
//16-23位为背腰和肩部气囊
#define R_U_WAIST	        0x00010000UL
#define R_D_WAIST	        0x00020000UL
#define L_U_WAIST	        0x00040000UL
#define L_D_WAIST	        0x00080000UL
#define R_SHOLDER	        0x00100000UL
#define L_SHOLDER	        0x00200000UL

#define LEFT_SHOULDER_CHR   L_SHOLDER
#define RIGHT_SHOULDER_CHR   R_SHOLDER

#define R_THIGH	                0x00400000UL
#define L_THIGH	                0x00800000UL

//24-27位为滚轮旋转方式
/*  0-1位为滚轮速度 00 停止 01 慢速 10 中速 11 高速
    2-3位为滚轮旋转方式 01短间歇 02长间歇 10 连续
*/

#define ROLLER_INTERMITTENT       0x04000000   //间歇旋转
#define ROLLER_SEMI_CIRCLE        0x08000000   //旋转半圈
#define ROLLER_CONTINUOUS         0x0c000000   //连续旋转
#define ROLLER_STOP	      0x00000000//0b00000000
#define ROLLER_SLOW_INT	  0x05000000//0b00000101
#define ROLLER_SLOW_SEM	  0x09000000//0b00001001
#define ROLLER_SLOW_CON	  0x0d000000//0b00001101
#define ROLLER_MID_INT	  0x06000000//0b00000110
#define ROLLER_MID_SEM	  0x0a000000//0b00001010
#define ROLLER_MID_CON	  0x0e000000//0b00001110
#define ROLLER_FAST_INT	  0x07000000//0b00000111
#define ROLLER_FAST_SEM	  0x0b000000//0b00001011
#define ROLLER_FAST_CON	  0x0f000000//0b00001111
#define ROLLER_PHASE	  0x10000000//0b00010000

#define ROLLER_INTERMITTENT_TIME      40
#define ROLLER_INTERMITTENT_ON_TIME   10
#define ROLLER_SEMI_CIRCLE_TIME      80
#define ROLLER_SEMI_CIRCLE_ON_TIME   30
//28-31位为滚轮旋转方式
#define STRETCH_STOP		0x00000000  //0b0000 0000 靠背
#define STRETCH_UP		0x10000000  //0b0001 0000靠背后躺，小腿上升，一直到小腿上升到最高点
#define STRETCH_DOWN		0x20000000  //0b0010 0000靠背不动，小腿下降，一直到小腿下降到最低点 
#define STRETCH_RESET   	0x30000000  //0b0011 0000靠背回到一定角度，小腿不动 

#define ALL_DIS			0x00000000
#define VALVE_POWER_PORT      gpioPortF
#define VALVE_POWER_BIT       3
#define VALVE_POWER_MODE      gpioModePushPull

#define VALVE_LOAD_PORT      gpioPortF   //165 load
#define VALVE_LOAD_BIT       5
#define VALVE_LOAD_MODE      gpioModePushPull

#define VALVE_CLK_PORT              gpioPortD
#define VALVE_CLK_BIT               2
#define VALVE_CLK_MODE              gpioModePushPull

#define VALVE_LE_PORT               gpioPortC //DRV8804 latch 
#define VALVE_LE_BIT                1
#define VALVE_LE_MODE               gpioModePushPull

#define VALVE_DATA_PORT             gpioPortD   //drv8804 serial data in SPI MOSI
#define VALVE_DATA_BIT              0
#define VALVE_DATA_MODE             gpioModePushPull

#define VALVE_DATA_IN_PORT          gpioPortD  //SPI MISO
#define VALVE_DATA_IN_BIT           1
#define VALVE_DATA_IN_MODE          gpioModeInput

#define VALVE_SPI                   USART1
#define VALVE_SPI_ROUTE_LOCAITON    USART_ROUTE_LOCATION_LOC1
#define VALVE_CMU_SPI               cmuClock_USART1     

#define VALVE_AIRPUMP1_PORT          gpioPortA   
#define VALVE_AIRPUMP1_BIT           7
#define VALVE_AIRPUMP1_MODE          gpioModePushPull


extern BITS BITS_ValveData[2] ;

//去掉一个U21,
#define ValveFungares18		BITS_ValveData[0].bD7  
#define ValveFungares17 	BITS_ValveData[0].bD6   
#define ValveFungares16 	BITS_ValveData[0].bD5   
#define ValveFungares15		BITS_ValveData[0].bD4   
#define ValveFungares14	        BITS_ValveData[0].bD3   
#define ValveFungares13		BITS_ValveData[0].bD2   
#define ValveFungares12		BITS_ValveData[0].bD1   
#define ValveFungares11  	BITS_ValveData[0].bD0 

/*
#define ValveFungares18		BITS_ValveData[0].bD3  
#define ValveFungares17 	BITS_ValveData[0].bD2   
#define ValveFungares16 	BITS_ValveData[0].bD1   
#define ValveFungares15		BITS_ValveData[0].bD0   
#define ValveFungares14	        BITS_ValveData[0].bD7   
#define ValveFungares13		BITS_ValveData[0].bD6   
#define ValveFungares12		BITS_ValveData[0].bD5   
#define ValveFungares11  	BITS_ValveData[0].bD4 
*/







#define ValveFungares1		BITS_ValveData[1].bD0  
#define ValveFungares7  	BITS_ValveData[1].bD1   
#define ValveFungares4  	BITS_ValveData[1].bD2   
#define ValveFungares3		BITS_ValveData[1].bD3   
#define ValveFungares5	        BITS_ValveData[1].bD4   
#define ValveFungares6		BITS_ValveData[1].bD5   
#define ValveFungares8		BITS_ValveData[1].bD6   
#define ValveFungares2  	BITS_ValveData[1].bD7 



struct AirBagStruct
{
	UINT32 nPumpValveState ;//气泵和气阀的状态
	unsigned char nKeepTime1 ;//当前状态保持时间,对应弱力度
	unsigned char nKeepTime2 ;//当前状态保持时间,对应中力度
	unsigned char nKeepTime3 ;//当前状态保持时间,对应强力度
} ;

#define STRETCH_MODE_TIME   1 //拉退模式 1为时间控制
#define STRETCH_MODE_SWITCH 0 //拉退模式为行程控制

typedef struct
{
	unsigned char timer;        //拉退程序计时定时器，单位0.1s
	unsigned char step ;        //拉退程序步骤
	unsigned char bBackLegFlag; //拉退程序中电动缸的状态
    unsigned char active;
    unsigned char init;
    unsigned char times;        //拉退程序循环次数
    unsigned char mode;         //拉退模式 1为时间控制 0为行程控制
    unsigned char PresetTime;   //拉退模式为时间控制时的预设时间，单位0.1秒
}StretchStruct;

typedef struct
{
    unsigned char time;        //拉退程序执行时间
    unsigned char times;       //一个回合的拉退次数 一般为3次
    unsigned char mode;        //拉退模式 STRETCH_GO_OUT向前拉 STRETCH_GO_DOWN向下拉
}StretchProgramStruct;


#define C_Stretch_Up                  1
#define C_Stretch_Stop                2
#define C_STRETCH_HOLD_TIME           30 //单位0.1s
#define C_STRETCH_RESET_TIME          100 //单位0.1s
#define C_STRETCH_CHARGE_TIME         120 //单位0.1s
#define C_STRETCH_CHARGE_TIME_NORMAL  70 //单位0.1s  /fww 原来60

#define VALVE_USART_INITSYNC                                                                  \
{                                                                                             \
    usartEnable,       /* Enable RX/TX when init completed. */                                \
    0,                 /* Use current configured reference clock for configuring baudrate. */ \
    1000000,           /* 1 Mbits/s. */                                                       \
    usartDatabits8,    /* 8 databits. */                                                      \
    true,              /* Master mode. */                                                     \
    true,             /* Send least significant bit first. */                                 \
    usartClockMode0    /* Clock idle low, sample on rising edge. */                           \
}
#define AIRBAG_LOCATE_NONE		0
#define AIRBAG_LOCATE_AUTO		1
#define AIRBAG_LOCATE_LEG_FOOT		2
#define AIRBAG_LOCATE_ARM_SHOLDER	3
#define AIRBAG_LOCATE_SEAT		4

//#define AIRBAG_LOCATE_BACK_WAIST	5

typedef struct
{
  unsigned char init; 
  unsigned char active; 
  unsigned char nCurAirBagStep;
  unsigned char nCurKeepTime1;
  unsigned char nCurKeepTime2; 
  unsigned char nCurKeepTime3; 
  unsigned char nCurKeepTime4;
  unsigned char nCurKeepTime5; 
  const struct AirBagStruct * pAirBagArray;
  UINT32 nCurPumpValveState;
  UINT16 nTotalSteps;
  unsigned char nAirBagCounter ;
  unsigned char locate ;
} st_AirBag;                                               


struct WaveMotorStruct
{
	unsigned char speed; //摇摆马达速度 0-3
	unsigned int  time ;   //摇摆马达持续时间 单位1sec  
} ;

extern unsigned char* pValveData;
extern unsigned char* pInputData;
void Valve_Initial_IO(void);
//void Valve_Send_Data(unsigned char * ucData,unsigned char ucLength);
//void Valve_Send_Data(unsigned char * ucSendData,unsigned char * ucReceiveData,unsigned char ucLength);
void Valve_Send_Data(void);
void Valve_10ms_Int(void);
void Valve_SetData(void);
void Valve_ClearData(void);
void Valve_SetClock(void);
void Valve_ClearClock(void);
void Valve_ClearLatch(void);
void Valve_SetLatch(void);

void Valve_LegFootAirPumpACPowerOn(void);
void Valve_LegFootAirPumpACPowerOff(void);

unsigned char Valve_GetAirBagStrength(void);
void Valve_SetAirBagStrength(unsigned char strength);
void Valve_AddAirBagStrength(void);

void Valve_FootRollerProce(unsigned char bRollerEnable,unsigned char Valve_Enable,st_AirBag* pBag);
void Valve_SetRollerPWM(unsigned char level);

void Valve_SetStretchUp(void);
void Valve_SetStretchCharge(unsigned int start);
void Valve_SetStretchHold(void);

void Valve_Test_Set_Data(unsigned int ValveTestData);
unsigned int Power_Get(void);

void Valve_Control(unsigned char nAirBagSwitch,st_AirBag* pBag,unsigned char level);

unsigned char Valve_Level_Decrease(unsigned char by_Data);
unsigned char Valve_Level_Increase(unsigned char by_Data);

void Valve_SetEnableSholder(unsigned int enable);
void Valve_1ms_Int(void);
int Valve_RollerIsAuto(void);
unsigned char Valve_GetRollerLevel(void);
void Valve_SetBackMode(int backauto);
void Valve_Initial_Data(void);

unsigned char Roller_GetRollerDirection(void);
void Valve_CloseAll(void);
#endif
