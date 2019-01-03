#include <stdio.h>
#include <stdbool.h>
#include "string.h"
#include "BlueTooth.h"
#include "DMAUart.h"
#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "efm32_types.h"
#include "efm32_def.h"
#include "ControlBox.h"
//#include "efm32.h"
//#include "em_cmu.h"
//#include "em_int.h"
//#include "em_gpio.h"
//#include "em_usart.h"
//#include "em_dma.h"

/* Declare some strings */
//const char     welcomeString[]  = "Energy Micro RS-232 - Please press a key\n";
//const char     overflowString[] = "\n---RX OVERFLOW---\n";
//const uint32_t welLen           = sizeof(welcomeString) - 1;
//const uint32_t ofsLen           = sizeof(overflowString) - 1;

/* Define termination character */
//#define TERMINATION_CHAR    '.'

/* Declare a circular buffer structure to use for Rx and Tx queues */
#define BUFFERSIZE          256

volatile struct circularBuffer
{
  uint8_t  data[BUFFERSIZE];  /* data buffer */
 // uint32_t rdI;               /* read index */
  uint32_t wrI;               /* write index */
  uint32_t pendingBytes;      /* count of how many bytes are not yet handled */
  bool     overflow;          /* buffer overflow indicator */
} BlueToothtxBuf = { 0, 0, 0, false };

/* Setup UART1 in async mode for RS232*/
static USART_TypeDef           * uart   = USART0;
static USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;

unsigned char ucBlueToothRXBuffer[BUFFER_LENGTH] = {0};
//unsigned char ucTXBuffer[BUFFER_LENGTH] = {0};
static volatile int  rxWriteIndex = 0;
unsigned char BlueToothRX_Index;
bool  bBlueToothRXOK;
unsigned char by_Button,by_Button1;
unsigned int nctrlType = NORMAL_CTRL;
//
bool wl_ready_to_send_flag = true;  //140410
/******************************************************************************
* @brief  uartSetup function
*
******************************************************************************/
void BlueToothUart_Initial_IO(void)
{
  /* Init BlueTooth's GPIO (the bluetooth switch channel) */
  GPIO_PinModeSet(BlueTooth_MUTE_PORT,BlueTooth_MUTE_BIT,BlueTooth_MUTE_MODE,BlueTooth_MUTE_INIT_ON);
  
  /* Configure GPIO pins For BlueTooth channel(USART0 #2) */
  GPIO_PinModeSet(BlueTooth_TX_PORT,BlueTooth_TX_BIT,BlueTooth_TX_MODE, 1);
  GPIO_PinModeSet(BlueTooth_RX_PORT,BlueTooth_RX_BIT,BlueTooth_RX_MODE, 0);

  /* Prepare struct for initializing UART in asynchronous mode*/
  uartInit.enable       = usartDisable;   /* Don't enable UART upon intialization */
  uartInit.refFreq      = 0;              /* Provide information on reference frequency. When set to 0, the reference frequency is */
  uartInit.baudrate     = 115200;         /* Baud rate */
  uartInit.oversampling = usartOVS6;      /* Oversampling. Range is 4x, 6x, 8x or 16x */
  uartInit.databits     = usartDatabits8; /* Number of data bits. Range is 4 to 10 */
  uartInit.parity       = usartNoParity;  /* Parity mode */
  uartInit.stopbits     = usartStopbits1; /* Number of stop bits. Range is 0 to 2 */
  //uartInit.mvdis        = false;          /* Disable majority voting */
  //uartInit.prsRxEnable  = false;          /* Enable USART Rx via Peripheral Reflex System */
  //uartInit.prsRxCh      = usartPrsRxCh0;  /* Select PRS channel if enabled */

  /* Initialize USART with uartInit struct */
  USART_InitAsync(uart, &uartInit);//fww

  /* Prepare UART Rx and Tx interrupts */
  USART_IntClear(uart, _UART_IF_MASK);//fww
  USART_IntEnable(uart, UART_IF_RXDATAV);//fww
  NVIC_ClearPendingIRQ(USART0_RX_IRQn);
  NVIC_ClearPendingIRQ(USART0_TX_IRQn);
  NVIC_EnableIRQ(USART0_RX_IRQn);
  NVIC_EnableIRQ(USART0_TX_IRQn);
  

  /* Enable I/O pins at UART1 location #3 */
  uart->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | UART_ROUTE_LOCATION_LOC2;//fww

  /* Enable UART */
  USART_Enable(uart, usartEnable);//fww
  
  
  /* Write welcome message to UART */
 // uartPutData((uint8_t*) welcomeString, welLen);
}
/**************************************************************************//**
 * @brief UART0 RX IRQ Handler
 *
 * Set up the interrupt prior to use
 *
 * Note that this function handles overflows in a very simple way.
 *
 *****************************************************************************/
void USART0_RX_IRQHandler(void)
{
  unsigned char by_Data;
  /* Check for RX data valid interrupt */
  
  if (uart->STATUS & USART_STATUS_RXDATAV)
  {
    /* Copy data into RX Buffer */
    uint8_t rxData = USART_Rx(uart);

    if(rxData == BlueTooth_SOI)
    {
      BlueToothRX_Index = 0;  
    }
    else
      if(rxData == BlueTooth_EOI)
      {
        by_Data = ucBlueToothRXBuffer[0];
        nctrlType = by_Data;

        //if (by_Data == ucRXBuffer[2])
          by_Button = ucBlueToothRXBuffer[1];
          if(by_Button < 0x80) //按键值0-127
          {
            bBlueToothRXOK = 1;
          }
         by_Button1 = ucBlueToothRXBuffer[2]; //常规模式无此按键，硬件测试时使用      
      }
    else
    {
      ucBlueToothRXBuffer[BlueToothRX_Index] = rxData;
      BlueToothRX_Index++;
      BlueToothRX_Index %= BUFFER_LENGTH;
    }
    /* Clear RXDATAV interrupt */
    USART_IntClear(USART0, USART_IF_RXDATAV);
  }
}

/**************************************************************************//**
 * @brief UART1 TX IRQ Handler
 *
 * Set up the interrupt prior to use
 *
 *****************************************************************************/
void USART0_TX_IRQHandler(void)
{
  uint32_t irqFlags = USART_IntGet(USART0);

  /* Check TX buffer level status */
  if (uart->STATUS & USART_STATUS_TXBL)
  {
    if (BlueToothtxBuf.pendingBytes > 0)
    {
      /* Transmit pending character */
      USART_Tx(uart, BlueToothtxBuf.data[BlueToothtxBuf.wrI]);//fww
      BlueToothtxBuf.wrI++;
      BlueToothtxBuf.pendingBytes--;
    }

    /* Disable Tx interrupt if no more bytes in queue */
    if (BlueToothtxBuf.pendingBytes == 0)
    {
      USART_IntDisable(uart, USART_IF_TXBL);//fww
    }
  }
}

/******************************************************************************
 * @brief  uartPutData function
 *每50ms刷新数据
 *****************************************************************************/
void uartBlueToothPutData(uint8_t * dataPtr, uint32_t dataLen)
{
  int i = 0;

  /* Check if buffer is large enough for data */
  if (dataLen > BUFFERSIZE)
  {
    /* Buffer can never fit the requested amount of data */
    return;
  }
  /*
  if ((txBuf.pendingBytes + dataLen) > BUFFERSIZE)
  {
    while ((txBuf.pendingBytes + dataLen) > BUFFERSIZE) ;
  }
  */
  while (i < dataLen)
  {
    BlueToothtxBuf.wrI = 0;
    BlueToothtxBuf.data[i] = *(dataPtr + i);
    i++;
  }

  /* Increment pending byte counter */
  BlueToothtxBuf.pendingBytes = dataLen;

  /* Enable interrupt on USART TX Buffer*/
  USART_IntEnable(uart, USART_IF_TXBL);//fww
}

void BlueToothUart_Transmit_Packet(unsigned char* buf,unsigned int length)
{
   uartBlueToothPutData(buf,length);
}
unsigned char BlueToothUart_GetRXStatus(void)
{
  return((unsigned char)bBlueToothRXOK);
}

unsigned char BlueToothUart_GetKey(void)
{
  return((unsigned char)by_Button);
}
unsigned char BlueToothUart_GetExternKey(void)
{
  return((unsigned char)by_Button1);
}

                                  
void BlueToothUart_SetKey(unsigned char by_Data)
{
  by_Button = by_Data;
}
void BlueToothUart_ClearRXStatus(void)
{
  bBlueToothRXOK = 0;
} 

void BlueToothUart_SetRXStatus(void)
{
  bBlueToothRXOK = 1;
} 
unsigned int BlueToothUart_GetCtrlType(void)
{
  return(nctrlType); 
}
void BlueToothOn(void)
{
  GPIO_PinOutSet(BlueTooth_MUTE_PORT,BlueTooth_MUTE_BIT);
}
void BlueToothOff(void)
{
  GPIO_PinOutClear(BlueTooth_MUTE_PORT,BlueTooth_MUTE_BIT);
}
unsigned char BlueToothMuteState(void)
{
  return GPIO_PinOutGet(BlueTooth_MUTE_PORT,BlueTooth_MUTE_BIT);
}
/*================================================================//
功能：    设置发送标志，为true时，有数据在缓存中需要发送
参数：    无
返回：    发送标志
调用方法：外部接口，用来判断是否有数据正在发送
//================================================================*/
bool WL_IsReadyToSendFlag(void)
{
    return(wl_ready_to_send_flag);
}
/*================================================================//
功能：    设置发送标志，为true时，有数据在缓存中需要发送
参数：    发送标志
返回：    无
调用方法：外部接口，当数据存入缓冲时，设置此标志位
//================================================================*/
void WL_SetReadyToSendFlag(bool flag)
{
    wl_ready_to_send_flag = flag;
}