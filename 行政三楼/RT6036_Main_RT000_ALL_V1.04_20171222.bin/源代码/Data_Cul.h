#ifndef __DATA_CUL_H__
#define __DATA_CUL_H__
#define DEFAULT_RUN_TIME 60*20
#define DEFAULT_RUN_TEST_TIME 60*2

#define RUN_TIME_10 60*10
#define RUN_TIME_20 60*20
#define RUN_TIME_30 60*30
#define RUN_TIME_120 60*12000//fww
#define VALVE_START_TIME  3*60
void Data_Init(void);
void Data_Flag_Int(void);
unsigned int Data_Get_Time(void);
void Data_Set_Time(unsigned int time);
unsigned int Data_Time_Counter_Proce(void);
unsigned int Data_Get_TimeSecond(void);
void Data_Set_Start(char by_Dat,unsigned int time);
void Data_Update_Time(unsigned int time);
//程序执行时间 返回多少秒
unsigned int Data_Get_ProgramExecTime(void);
void Data_Clear_ProgramExecTime(void);

#endif