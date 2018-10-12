/********************************
转义序列 		表示 
\a 			响铃(警告) 
\b 			回退 
\f 			换页 
\n 			换行 
\r 			回车 
\t 			水平制表 
\v 			垂直制表 
\′ 			单引号 
\" 			双引号 
\\ 			反向斜杠 
\? 			文字问号 
\ooo 		八进制表示的ASCII字符 
\xhhh 		十六进制表示的ASCII字符 
*********************************/
#define VIBRATION_ON	1
#define VIBRATION_OFF 	0

//nVibDescriptor
#define VIB_STEADY				0 //持续式
#define VIB_PULSE				1 //脉冲式(间隔式)
#define VIB_WAVE				2 //波浪式
#define VIB_INTERLACE			3 //交替式
//nVibTimeMode
#define TIME_MODE_SYSTEM		0
#define TIME_MODE_SELF			1
//nVibStrengthMode
#define STRENGTH_MODE_SYSTEM	0
#define STRENGTH_MODE_SELF		1
//自动振动模式包含的振动步骤
#define TOTAL_AUTO_VIBRATE_ITEMS	3
//通用振动模型(指针+结构类模型,有些单片机编译器不支持此模式)
typedef struct Vibration_Struct_Point
{
	//振动描述(名称)
	unsigned char nVibDescriptor ;
	//步骤总数
	unsigned char nVibTotalSteps ;
	//动作组合指针
	unsigned char const * pVibAction ;
	//单步动作持续时间模式
	//TIME_MODE_SYSTEM:由按键确定的速度决定
	//TIME_MODE_SELF:由下面的指针变量确定
	unsigned char nVibTimeMode ;
	//单步动作持续时间指针,在nVibTimeMode = TIME_MODE_SELF时有效
	unsigned char const * pVibTime ;
	//单部动作力度模式
	//STRENGTH_MODE_SYSTEM:由按键确定的力度决定
	//STRENGTH_MODE_SELF:由下面的指针变量确定
	unsigned char nVibStrengthMode ;
	//单步动作力度指针,在nVibStrengthMode = STRENGTH_MODE_SELF时有效
	unsigned char const * pVibStrength ;
} VIBRATION_STRUCT_POINT ;

//通用振动模型(结构＋数组类模型,有一定的数据冗余，有些单片机编译器不支持此模式)
typedef struct Vibration_Struct_Array
{
	//振动描述(名称)
	unsigned char nVibDescriptor ;
	//步骤总数
	unsigned char nVibTotalSteps ;
	//动作组合
	unsigned char nVibAction ;
	//单步动作持续时间模式
	//TIME_MODE_SYSTEM:由按键确定的速度决定
	//TIME_MODE_SELF:由下面的值确定
	unsigned char nVibTimeMode ;
	//单步动作持续时间,在nVibTimeMode = TIME_MODE_SELF时有效
	unsigned char nVibTime ;
	//单部动作力度模式
	//STRENGTH_MODE_SYSTEM:由按键确定的力度决定
	//STRENGTH_MODE_SELF:由下面的值确定
	unsigned char nVibStrengthMode ;
	//单步动作力度,在nVibStrengthMode = STRENGTH_MODE_SELF时有效
	unsigned char nVibStrength ;
} VIBRATION_STRUCT_ARRAY ;

//通用振动模型(一维数组模型,有较大的数据冗余，所有的单片机编译器都支持此模式)
//将VIBRATION_STRUCT_ARRAY结构的每个元素拆分成一维数组，在此省略

//时间单位:100ms
const VIBRATION_STRUCT_POINT VibrateAuto[] =
{
	{
		VIB_STEADY,
		12,
		"\x01\x02\x01\x02\x03\x00\x03\x00\x01\x02\x01\x02",
		TIME_MODE_SELF,
		"\x14\x14\x14\x14\xc8\x14\x14\x14\x14\x14\x14\x14",
		STRENGTH_MODE_SYSTEM,
		"\x0"
	},
	{
		VIB_WAVE,
		13,
		"\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03",
		TIME_MODE_SELF,
		"\x0a\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x14\x0a",
		STRENGTH_MODE_SELF,
		"\x00\x03\x06\x09\x0c\x0f\x12\x0f\x0c\x09\x06\x03\x00"
	},
	{
		VIB_PULSE,
		28,
		"\x01\x02\x01\x02\x01\x02\x01\x02\x01\x02\x01\x02\x01\x02\x01\x03\x02\x00\x03\x00\x03\x00\x01\x02\x01\x02\x01\x00",
		TIME_MODE_SELF,
		"\x0a\x05\x0a\x05\x08\x04\x08\x04\x08\x04\x06\x03\x06\x03\x06\x03\x06\x03\x04\x02\x04\x02\x04\x02\x04\x02\x04\x02",
		STRENGTH_MODE_SYSTEM,
		"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
	}
} ;

//3 shift strength
//convert to mark ratio

//以下部分参考2006/03/16笔记
//振动PWM周期
#define VIB_PWM_PERIOD				18 //18*2ms=36ms
//音乐互动相关常数
#define MUSIC_AD_LOW_THRESHOLD		10	//启动振动音乐互动的门限值,对应电压0.13V
#define MUSIC_AD_HIGH_THRESHOLD		200 //全开振动门限，对应电压2.6V
#define MUSIC_VIB_AD_RATIO			4   //
/*
#define MUSIC_AD_LOW_THRESHOLD		10	//启动振动音乐互动的门限值,对应电压0.2V
#define MUSIC_AD_HIGH_THRESHOLD		172 //全开振动门限，对应电压3.37V
#define MUSIC_VIB_AD_RATIO			9   //
*/
