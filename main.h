#ifndef __MAIN_H__
#define __MAIN_H__

/******************Macro Define **************/
//机器识别码定义
//1 :RT8302S
//2 :RT8600
#define MACHINEID                   2

#define A_STATE_IDLE                1
#define A_STATE_SETTLE              2
#define A_STATE_WAIT_COMMAND        3
#define A_STATE_PROBLEM             4     
#define A_STATE_WAIT_MEMORY         5
#define A_STATE_RUN                 6
#define A_TEST                      7

#define M_NONE                      1
#define M_NEXT                      2
#define	M_IDLE                      3
#define	M_RUN                       4
#define M_PROBLEM                   5
#define	M_WAIT_COMMAND              6
#define	M_ENG                       7
#define	M_SETTLE                    8
#define	M_TEST                      9
/******************fww*********************
//140528 Key cancle or confirm 
//#define KEYCANCLE                   0  //fww
//#define KEYCONFIRM                  1  //fww
//#define KEY_NOT_CONFIRMORCANCLE     2  //fww

//140528 Enter Dis when in Reset program
#define ExitDisReset                0
#define EnterDisReset               1

//140528 ResetFlag and WorkMode flag
//#define NORESET_CHAIR               0  //fww
//#define RESET_CHAIR                 1  //fww
//#define CHANGE_WORKMODE             2  //fww

//140528 ResetReason flage
#define TIMEOVER                    0
#define POWERKEY_PRESSED            1
#define CHANGE_MODE                 2
//140528 TimeOverResetAgain flag
//#define FIRST_TIME_ENTER            0   //fww
//#define SECOND_TIME_ENTER           1   //fww
*******************fww********************/

//
#define ZERO_POSITION_RESET         0  //所有的电动缸复位
#define ZERO_POSITION1              1  //第一个零重力点  
#define ZERO_POSITION2              2  //第二个零重力点  
#define ZERO_POSITION_STRETCH_UP    3  //向上拉退  
#define ZERO_POSITION_STRETCH_DOWN  4  //向下拉退  

//摇摆功能相关宏定义
#define RockDisable                   false
#define RockEnable                    true
#define ExitRock                      0
#define EnterRock                     1
#define StartRock                     0
#define LieDown                       1
#define LieUP                         2

//141116
#define MAX_KNEADBALANCE_TIME_1MIN  20 //0.5min


#define MASSAGE_BACK_OPTIMAL_POSITION  1000
#define MASSAGE_LEG_OPTIMAL_POSITION   800

#define MASSAGE_BACK_OPTIMAL1_POSITION  1800
#define MASSAGE_LEG_OPTIMAL1_POSITION   1200

//
#define MASSAGE_BACK_ROCK_POSITION        1500
#define MASSAGE_LEG_ROCK_POSITION         1200
#define MASSAGE_BACK_DOWN_ROCK_POSITION   380
#define MASSAGE_BACK_UP_ROCK_POSITION     100
#define MASSAGE_DIFFERENT_POSITION        50

enum
{
   MASSAGE_RESET_POSITION,    // 按摩椅复位位置，前滑电动缸收回，靠背电动缸在最高，小腿电动缸在最低，也是关机后的位置
   MASSAGE_INIT_POSITION,     // 按摩椅初始位置，前滑电动缸在最前，靠背电动缸在最高，小腿电动缸在最低
   MASSAGE_OPTIMAL_POSITION, //前滑电动缸在最前，靠背电动缸和小腿电动缸在自动程序初始位置
   MASSAGE_OPTIMAL2_POSITION, //前滑电动缸在最前，靠背电动缸和小腿电动缸在自动程序初始位置之后
   MASSAGE_UNKNOW_POSITION, //
   MASSAGE_MAX_POSITION,      //按摩椅最平位置 前滑电动缸在最前，靠背电动缸在最低，小腿电动缸在最高
   MASSAGE_ANY_POSITION  
};

/******************Functions Prototype *********************/

void Main_Initial_IO(void);
void Main_Idle(void);
void Main_Settle(void);
void Main_WaitCommand(void);
void Main_Work(void);
void Main_Problem(void);
void MOTOR_RUN_DIS_PROCESS(unsigned char ConfirmKey);
void PowerOff_Pause_all(void);

#endif
