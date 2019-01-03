#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__
#include "MassageStatus.h"

#define BlueTooth_TX_PORT           gpioPortC
#define BlueTooth_TX_BIT            11
#define BlueTooth_TX_MODE           gpioModePushPull

#define BlueTooth_RX_PORT           gpioPortC
#define BlueTooth_RX_BIT            10
#define BlueTooth_RX_MODE           gpioModeInputPull

//BlueTooth's Switch Port
#define BlueTooth_MUTE_PORT         gpioPortC
#define BlueTooth_MUTE_BIT          8
#define BlueTooth_MUTE_MODE         gpioModePushPull
#define BlueTooth_MUTE_INIT_ON      1

#define BlueTooth_UART              USART0
#define BlueTooth_CLK               cmuClock_USART0
#define BlueTooth_RX                USART_Rx
#define BlueTooth_LOCATION          USART_ROUTE_LOCATION_LOC2

#define DMAUART_IRQn              USART0_RX_IRQn

#define BUFFER_LENGTH             32

#define BlueTooth_SOI                0XF0
#define BlueTooth_EOI                0XF1
//Master->Slave Packet Type
#define PACKET_MASTER_GET_COMMAND 0x00
#define PACKET_MASTER_ACK_COMMAND 0x01
#define PACKET_MASTER_SET_STATE   0x02
//Slave->Master Packet Type
#define PACKET_SLAVE_SEND_COMMAND 0x03

//BlueTooth'mute pin State
#define BlueTooth_Speak_Out_On          1
#define BlueTooth_Speak_Out_Off         0
#define BlueTooth_MutePin_Value         0xef    //将第四位，即是输出Buffer的第4位清零

void BlueToothUart_Initial_IO(void);
void BlueToothUart_Transmit_Packet(unsigned char* buf,unsigned int length);
unsigned char BlueToothUart_GetRXStatus(void);
unsigned char BlueToothUart_GetKey(void);
unsigned char BlueToothUart_GetExternKey(void);
void BlueToothUart_SetKey(unsigned char by_Data);
void BlueToothUart_ClearRXStatus(void);
unsigned int BlueToothUart_GetCtrlType(void);
unsigned char BlueToothMuteState(void);
void BlueToothOff(void);
void BlueToothOn(void);
void WL_SetReadyToSendFlag(bool flag);
bool WL_IsReadyToSendFlag(void);

#endif // __BLUETOOTH_H__