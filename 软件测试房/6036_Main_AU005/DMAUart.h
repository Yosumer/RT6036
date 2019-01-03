#include "MassageStatus.h"

#define DMAUART_TX_PORT           gpioPortC
#define DMAUART_TX_BIT            12
#define DMAUART_TX_MODE           gpioModePushPull

#define DMAUART_RX_PORT           gpioPortC
#define DMAUART_RX_BIT            13
#define DMAUART_RX_MODE           gpioModeInputPull

#define DMAUART_UART              USART0
#define DMAUART_CLK               cmuClock_USART0
#define DMAUART_RX                USART_Rx
#define DMAUART_LOCATION          USART_ROUTE_LOCATION_LOC2

#define DMAUART_IRQn              USART0_RX_IRQn

#define BUFFER_LENGTH             32

#ifdef RT8301_CONTROL 
  #define SOI			0x7e
  #define EOI			0x0d
#else
  #define SOI                    0XF0
  #define EOI                    0XF1
#endif

//Master->Slave Packet Type
#define PACKET_MASTER_GET_COMMAND 0x00
#define PACKET_MASTER_ACK_COMMAND 0x01
#define PACKET_MASTER_SET_STATE   0x02
//Slave->Master Packet Type
#define PACKET_SLAVE_SEND_COMMAND 0x03

void DMAUart_Initial_IO(void);
void uartPutData(unsigned char* dataPtr, unsigned int dataLen);
unsigned int uartGetData(unsigned char* dataPtr, unsigned int dataLen);
void DMAUart_Transmit_Packet(unsigned char* buf,unsigned int length);
unsigned char DMAUart_GetRXStatus(void);
unsigned char DMAUart_GetKey(void);
void DMAUart_SetKey(unsigned char by_Data);
unsigned char DMAUart_GetExternKey(void);
void DMAUart_ClearRXStatus(void);
unsigned int DMAUart_GetCtrlType(void);
//unsigned char getBluetoothCMD();//fww
//unsigned char* getBluetoothAddr();//fww
//void bluetoothCountIncrease();//fww
