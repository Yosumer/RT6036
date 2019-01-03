#ifndef __MP3CTRL_H__
#define __MP3CTRL_H__
/*
//MP3状态输入
#define MP3_CTRL0_PORT   gpioPortD
#define MP3_CTRL0_BIT    3
#define MP3_CTRL0_MODE   gpioModeInput
//对应原理图的网络标号 MP3_Ctrl2
#define MP3_CTRL1_PORT   gpioPortD
#define MP3_CTRL1_BIT    5
#define MP3_CTRL1_MODE   gpioModePushPull
//对应原理图的网络标号 MP3_Ctrl3
#define MP3_CTRL2_PORT   gpioPortC
#define MP3_CTRL2_BIT    8
#define MP3_CTRL2_MODE   gpioModePushPull
//对应原理图的网络标号 I2C_SDA
#define MP3_CTRL3_PORT   gpioPortD
#define MP3_CTRL3_BIT    6
#define MP3_CTRL3_MODE   gpioModePushPull
//对应原理图的网络标号 I2C_SCL
#define MP3_CTRL4_PORT   gpioPortD
#define MP3_CTRL4_BIT    7
#define MP3_CTRL4_MODE   gpioModePushPull
*/
/*
#define MP3_POWER_PORT   gpioPortC
#define MP3_POWER_BIT    12
#define MP3_POWER_MODE   gpioModePushPull
*/


void MP3Control1_Initial_IO(void);
void MP3Control1_Set(void);
void MP3Control1_Clear(void);
void MP3Control2_Set(void);
void MP3Control2_Clear(void);
void MP3Control3_Set(void);
void MP3Control3_Clear(void);
void MP3Control4_Set(void);
void MP3Control4_Clear(void);

void MP3KeyControl1_PlayPause(void);
void MP3KeyControl1_Stop(void);
void MP3KeyControl1_Previous(void);
void MP3KeyControl1_Next(void);
void MP3KeyControl1_VolumeDec(void);
void MP3KeyControl1_VolumeInc(void);

#endif /*__MP3CTRL_H__*/