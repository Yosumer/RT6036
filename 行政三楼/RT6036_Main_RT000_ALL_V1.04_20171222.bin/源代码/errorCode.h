/* 错误代码列表
1 电源	1	24V电源过高	24V电源电压高于36V
	2	24V电源过低	24V电源电压低于10V
	3	气阀电源过高	气阀电压高于36V
	4	气阀电源过低	气阀电压低于10V
	5	5V电压过高	5V电压高于5.5V
	6	5V电压过低	5V电压低于4.5V
2 靠背电动缸
        1	靠背电动缸检测信号异常	两个信号开路
	2	靠背电动缸驱动异常	过载
	3	靠背电动缸驱动异常或靠背电动缸检测信号异常	单项连续运行时间超过30秒未碰到行程开关
3 小腿电动缸
        1	小腿电动缸检测信号异常	两个信号开路
	2	小腿电动缸驱动异常	过载
	3	小腿电动缸驱动异常或小腿电动缸检测信号异常	单项连续运行时间超过30秒未碰到行程开关
4 零重力电动缸
        1	零重力电动缸检测信号异常	两个信号开路
	2	零重力电动缸驱动异常	过载
	3	零重力电动缸驱动异常或零重力电动缸检测信号异常	单项连续运行时间超过30秒未碰到行程开关
5 前滑电动缸
	1	前滑电动缸检测信号异常	两个信号开路
	2	前滑电动缸驱动异常	过载
	3	前滑电动缸驱动异常或前滑电动缸检测信号异常	单项连续运行时间超过30秒未碰到行程开关
6 揉捏电机
	1	宽中窄信号检测异常	信号开路
	2	揉捏电机驱动异常	过载
	3	揉捏电机驱动异常或宽中窄信号检测异常	单项连续运行时间超过10秒未碰到行程开关
7 敲击电机
	1	敲击电机驱动异常	过载
8 行走电机
	1	行走电机脉冲检测信号异常	电机未运行，有脉冲信号
	2	行走电机行程检测信号异常	行程检测信号开路
	3	行走电机驱动异常	过载
	4	行走电机驱动异常或检测信号异常	行走电机连续单向运行时间超过10秒无脉冲信号
9 脚底滚轮电机
	1	脚底滚轮电机驱动异常	过载
10 电动小腿电机
	1	电动小腿检测信号异常	信号开路
	2	电动小腿电机驱动异常	过载
	3	电动小腿电机驱动异常或信号检测异常	单项连续运行时间超过30秒未碰到行程开关
*/


#define ERROR_CODE_NULL        0
//power
#define ERROR_CODE_POWER       1

  #define ERROR_CODE_POWER_MOTOR_HIGHER  1
  #define ERROR_CODE_POWER_MOTOR_LOWER   2
  #define ERROR_CODE_POWER_VALVE_HIGHER  3
  #define ERROR_CODE_POWER_VALVE_LOWER   4
  #define ERROR_CODE_POWER_VCC_HIGHER    5
  #define ERROR_CODE_POWER_VCC_LOWER     6
//backpad
#define ERROR_CODE_BACKPAD     2
  #define ERROR_CODE_BACKPAD_SWITCH     1
  #define ERROR_CODE_BACKPAD_OVERLOAD   2
  #define ERROR_CODE_BACKPAD_TIMEOVER   3
//legpad
#define LEGPAD_ERROR      3
  #define ERROR_CODE_LEGPAD_SWITCH     1
  #define ERROR_CODE_LEGPAD_OVERLOAD   2
  #define ERROR_CODE_LEGPAD_TIMEOVER   3
//zero motor
#define ZEROMOTOR_ERROR   4
  #define ERROR_CODE_ZEROPAD_SWITCH     1
  #define ERROR_CODE_ZEROPAD_OVERLOAD   2
  #define ERROR_CODE_ZEROPAD_TIMEOVER   3
//slide motor
#define SLIDEMOTOR_ERROR  5
  #define ERROR_CODE_SLIDEMOTOR_SWITCH     1
  #define ERROR_CODE_SLIDEMOTOR_OVERLOAD   2
  #define ERROR_CODE_SLIDEMOTOR_TIMEOVER   3
//knead motor
#define KNEADMOTOR_ERROR  6
  #define ERROR_CODE_KNEADMOTOR_SWITCH     1
  #define ERROR_CODE_KNEADMOTOR_OVERLOAD   2
  #define ERROR_CODE_KNEADMOTOR_TIMEOVER   3
//knoc kmotor
#define KNOCKMOTOR_ERROR  7
  #define ERROR_CODE_KNOCKMOTOR_OVERLOAD   2
//walk motor
#define WALKMOTOR_ERROR   8
  #define ERROR_CODE_WALKMOTOR_SWITCH     1
  #define ERROR_CODE_WALKMOTOR_OVERLOAD   2
  #define ERROR_CODE_WALKMOTOR_TIMEOVER   3
//roller motor
#define ROLLERMOTOR_ERROR 9
  #define ERROR_CODE_ROLLERMOTOR_OVERLOAD   2
//flex motor
#define FLEXMOTOR_ERROR   10
  #define ERROR_CODE_FLEXMOTOR_OVERLOAD   2

