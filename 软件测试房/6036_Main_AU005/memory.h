#include "MassageStatus.h"
#define MEMORY_LENGTH 13
//#define PRODUCT_ID_ADDR         ((uint32_t) (128*1024-5)) //((uint32_t) 0x0000FFFBUL) 


/*******************************************************
使用一个页面来记录累计时间
累计时间单位为秒，每次以4个字节长度来存储数据 最大数据为2的32次方-1秒
一个页面为512个字节共可记录512/4=128次
程序首次使用必须先将整个记录区全部擦除，第一次从零地址开始写，
下一次从第四个地址开始写，写到最后一个地址再将整个数据区擦除
*******************************************************/
#define ACC_DATA_BASE   (USER_DATA_BASE+0x100)

#define USER_DATA_BASE          ((uint32_t) 0x0FE00000UL)  /**< user data flash base address  */
//以下地址是按照字节存放
#define SOFT_MAIN_VER_ADDRESS         0
#define SOFT_SECONDARY_VER_ADDRESS    1
#define SETTLE_ADDRESS                2  //按摩完成是否复位 数据为1复位
#define AIRBAG_STRETCH_ADDRESS        3   //按摩椅内部气囊力度
#define SLIDE_MOTOR_ENABLE_ADDRESS    4   //滑动马达使能与禁止
#define PROGRAM_ENABLE_ADDRESS        5   //编程使能地址
#define STRETCH_OUT_ADDRESS           6
#define SLEEP_PROGRAM_ADDRESS         7
#define SLIDE_BLUETOOTH_POWER         8    //FWW 20150313
#define TIME_SWITCH                   9    //FWW 将时间写入，防止掉电后改变


#define ACC_TIME_0_ADDRESS  0x10
#define ACC_TIME_1_ADDRESS  0x11
#define ACC_TIME_2_ADDRESS  0x12
#define ACC_TIME_3_ADDRESS  0x13

#define MEMORY_LENGTH_OF_BYTES		12    //fww
#define PROGRAM_FLAG			'p'
#define PROGRAM_BY_BLUETOOTH_FLAG	'l'
#define SOFT_MAIN_VER			1
//2.2 增加复位时按任意键可以停止
//   调整电动伸缩小腿的力度在手动时为3A在复位时为2A
/*
2.02 常规
2.03 台湾
2.04 美国
2.05 韩国
2.06版开始台湾、常规、us047、韩国程序均可由工程模式切换
*/
#define SOFT_SECONDARY_VER      3

unsigned char ReadEEByte(unsigned int nAddress);
void MEM_Write_Memory(PUINT32 pw_Buffer,int numBytes);
void MEM_Read_Memory(PUINT32 pw_Buffer,int numBytes);

//获取累计工作时间 单位秒
unsigned int MEM_Get_Total_Time(void);

//存储累计工作时间 单位秒
void MEM_Save_Total_Time(unsigned int totalTime);
