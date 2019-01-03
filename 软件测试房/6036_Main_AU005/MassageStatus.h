#ifndef __MASSAGE_STATUS_H__
#define __MASSAGE_STATUS_H__

#define CHAIR_STATE_IDLE			0  //待机状态
#define CHAIR_STATE_SETTLE			1  //回位状态1（所有按摩结构回位）
#define CHAIR_STATE_WAIT_COMMAND	        2  //等待按命令,相关的指示灯闪烁
#define CHAIR_STATE_RUN				3  //运行状态
#define CHAIR_STATE_WAIT_MEMORY		        4  
#define CHAIR_STATE_PROBLEM			5  //故障模式
#define CHAIR_STATE_ENGINEERING                 6  //工程模式
#define CHAIR_STATE_SLEEP			7  //椅子工作在睡眠模式，此时要求待机功耗小于0.5W
#define CHAIR_STATE_DEMO                        9
#define CHAIR_STATE_SETTLE_1ST			8  //回位状态2（提示收藏中）

#define MOTOR_STOP_BREAK    0  //马达停止并刹车
#define MOTOR_STOP_HZ       1  //马达停止，马达输出端口为高阻
#define MOTOR_RUN           2  //马达运行
#define MOTOR_STOP_HZ_TIME  10  //单位10ms 

#define MOTOR_ON	1
#define MOTOR_OFF	0

extern unsigned char nChairRunState;
//韩国程序
//#define KR_PROGRAM    1  //要求带有睡眠功能

#define STRETCH_GO_DOWN 0  //向下拉退
#define STRETCH_GO_OUT  1  //向前拉退


#endif /*__MASSAGE_STATUS_H__*/