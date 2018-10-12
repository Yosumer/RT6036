#include <stdio.h>
#include "string.h"
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
} txBuf = { 0, 0, 0, 0, false };

/* Setup UART1 in async mode for RS232*/
static USART_TypeDef           * uart   = UART1;
static USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;

unsigned char ucRXBuffer[BUFFER_LENGTH] = {0};
unsigned char ucTXBuffer[BUFFER_LENGTH] = {0};
static volatile int     rxWriteIndex = 0;
unsigned char RX_Index;
bool  bRXOK;

DMA_DESCRIPTOR_TypeDef dmaControlBlock[DMA_CHAN_COUNT * 2];
unsigned char by_Key,by_Key1;

unsigned int ctrlType = NORMAL_CTRL;

unsigned char *blueToothBuffer1 = "MS=02,";
unsigned char *blueToothBuffer2 = "RS=02,";
unsigned char blueToothAddr[12];
unsigned char blueToothAddrIndex;
unsigned char blueToothCompareStep = 0;
bool blueToothShouldConnect = false;
unsigned int blueToothCount = 0;
bool compareBluetoothMSG(uint8_t rxData);
bool compareBluetoothMSG2(uint8_t rxData);


/******************************************************************************
* @brief  uartSetup function
*
******************************************************************************/
void DMAUart_Initial_IO(void)
{
  /* Enable clock for GPIO module (required for pin configuration) */
 // CMU_ClockEnable(cmuClock_GPIO, true);
  /* Configure GPIO pins */
  GPIO_PinModeSet(DMAUART_TX_PORT,DMAUART_TX_BIT,DMAUART_TX_MODE, 1);
  GPIO_PinModeSet(DMAUART_RX_PORT,DMAUART_RX_BIT,DMAUART_RX_MODE, 0);

  /* Prepare struct for initializing UART in asynchronous mode*/
  uartInit.enable       = usartDisable;   /* Don't enable UART upon intialization */
  uartInit.refFreq      = 0;              /* Provide information on reference frequency. When set to 0, the reference frequency is */
  uartInit.baudrate     = 9600;//115200;         /* Baud rate */
  uartInit.oversampling = usartOVS16;     /* Oversampling. Range is 4x, 6x, 8x or 16x */
  uartInit.databits     = usartDatabits8; /* Number of data bits. Range is 4 to 10 */
  uartInit.parity       = usartNoParity;  /* Parity mode */
  uartInit.stopbits     = usartStopbits1; /* Number of stop bits. Range is 0 to 2 */
  //uartInit.mvdis        = false;          /* Disable majority voting */
  //uartInit.prsRxEnable  = false;          /* Enable USART Rx via Peripheral Reflex System */
  //uartInit.prsRxCh      = usartPrsRxCh0;  /* Select PRS channel if enabled */

  /* Initialize USART with uartInit struct */
  USART_InitAsync(uart, &uartInit);

  /* Prepare UART Rx and Tx interrupts */
  USART_IntClear(uart, _UART_IF_MASK);
  USART_IntEnable(uart, UART_IF_RXDATAV);
  NVIC_ClearPendingIRQ(UART1_RX_IRQn);
  NVIC_ClearPendingIRQ(UART1_TX_IRQn);
  NVIC_EnableIRQ(UART1_RX_IRQn);
  NVIC_EnableIRQ(UART1_TX_IRQn);
  

  /* Enable I/O pins at UART1 location #3 */
  uart->ROUTE = UART_ROUTE_RXPEN | UART_ROUTE_TXPEN | UART_ROUTE_LOCATION_LOC0;//fww yuanlai 0 qu S erge

  /* Enable UART */
  USART_Enable(uart, usartEnable);
  
  
  /* Write welcome message to UART */
 // uartPutData((uint8_t*) welcomeString, welLen);
}

unsigned char  AsciiToVal(unsigned char nAscii)
{
	if(nAscii>=0x30 && nAscii<=0x39) return(nAscii-0x30);
	if(nAscii>=0x41 && nAscii<=0x46) return(nAscii-0x37);
	if(nAscii>=0x61 && nAscii<=0x66) return(nAscii-0x57);
	return 0 ;
}

/**************************************************************************//**
 * @brief UART0 RX IRQ Handler
 *
 * Set up the interrupt prior to use
 *
 * Note that this function handles overflows in a very simple way.
 *
 *****************************************************************************/
void UART1_RX_IRQHandler(void)
{
  unsigned char by_Data;
  /* Check for RX data valid interrupt */
  
  if (uart->STATUS & UART_STATUS_RXDATAV)//fww
  {
    /* Copy data into RX Buffer */
    uint8_t rxData = USART_Rx(uart);
    
    /************************蓝牙 start*****fww********************
    //step 3 RS=02,
    if(blueToothShouldConnect && compareBluetoothMSG2(rxData)){
      blueToothShouldConnect = FALSE;
    }
    //step 2
    if(blueToothShouldConnect && blueToothAddrIndex < 12){
      blueToothAddr[blueToothAddrIndex++] = rxData;
    }
    //step 1 MS=02,
    if(!blueToothShouldConnect && compareBluetoothMSG(rxData)){
      blueToothShouldConnect = TRUE;
      blueToothAddrIndex = 0;
      blueToothCount = 0;
    }
    ************************蓝牙 end******fww*******************/
    if(rxData == SOI)
    {
      RX_Index = 0;  
    }
    else
      if(rxData == EOI)
      {
#ifndef RT8301_CONTROL       
        by_Data = ucRXBuffer[0];
        ctrlType = by_Data;
        /*
        by_Data += ucRXBuffer[1];
        by_Data ^= 0xff;
        by_Data += 1;
        by_Data &= 0x7f;
        */
        //if (by_Data == ucRXBuffer[2])
          by_Key = ucRXBuffer[1];
          if(by_Key < 0x80) //按键值0-127
          {
            bRXOK = 1;
          }
         by_Key1 = ucRXBuffer[2]; //常规模式无此按键，硬件测试时使用
#else       
        by_Data = (AsciiToVal(ucRXBuffer[0])<<4) | (AsciiToVal(ucRXBuffer[1])) ;
        if(by_Data == PACKET_SLAVE_SEND_COMMAND )                                            
        {
          by_Key = (AsciiToVal(ucRXBuffer[2])<<4) | (AsciiToVal(ucRXBuffer[3])) ;  //取出按键值
          bRXOK = true;   
        }
#endif        
      }
    else
    {
      ucRXBuffer[RX_Index] = rxData;
      RX_Index++;
      RX_Index %= BUFFER_LENGTH;
    }
   
    /* Clear RXDATAV interrupt */
    USART_IntClear(UART1, UART_IF_RXDATAV);//fww
  }
}

/**************************************************************************//**
 * @brief UART1 TX IRQ Handler
 *
 * Set up the interrupt prior to use
 *
 *****************************************************************************/
void UART1_TX_IRQHandler(void)
{
  uint32_t irqFlags = USART_IntGet(UART1);//fww原来为USART0

  /* Check TX buffer level status */
  if (uart->STATUS & UART_STATUS_TXBL)//fww
  {
    if (txBuf.pendingBytes > 0)
    {
      /* Transmit pending character */
      USART_Tx(uart, txBuf.data[txBuf.wrI]);
      txBuf.wrI++;
      txBuf.pendingBytes--;
    }

    /* Disable Tx interrupt if no more bytes in queue */
    if (txBuf.pendingBytes == 0)
    {
      USART_IntDisable(uart, UART_IF_TXBL);
    }
  }
}

/******************************************************************************
 * @brief  uartPutData function
 *每50ms刷新数据
 *****************************************************************************/
void uartPutData(uint8_t * dataPtr, uint32_t dataLen)
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
    txBuf.wrI = 0;
    txBuf.data[i] = *(dataPtr + i);
    i++;
  }

  /* Increment pending byte counter */
  txBuf.pendingBytes = dataLen;

  /* Enable interrupt on USART TX Buffer*/
  USART_IntEnable(uart, UART_IF_TXBL);
}

void DMAUart_Transmit_Packet(unsigned char* buf,unsigned int length)
{
   uartPutData(buf,length);
}
unsigned char DMAUart_GetRXStatus(void)
{
  return((unsigned char)bRXOK);
}

unsigned char DMAUart_GetKey(void)
{
  return((unsigned char)by_Key);
}
unsigned char DMAUart_GetExternKey(void)
{
  return((unsigned char)by_Key1);
}

                                  
void DMAUart_SetKey(unsigned char by_Data)
{
  by_Key = by_Data;
}
void DMAUart_ClearRXStatus(void)
{
  bRXOK = 0;
} 

void DMAUart_SetRXStatus(void)
{
  bRXOK = 1;
} 
unsigned int DMAUart_GetCtrlType(void)
{
  return(ctrlType); 
}

/**********************************bluetooth start*****fww**********************
bool compareBluetoothMSG(uint8_t rxData)
{
  bool result = false;
  if(blueToothBuffer1[blueToothCompareStep] == rxData)
  {
    blueToothCompareStep++;
  }else{
    blueToothCompareStep = 0;
  }
  if(blueToothCompareStep == 6){
    result = true;
    blueToothCompareStep = 0;
  }
  return result;
}

bool compareBluetoothMSG2(uint8_t rxData)
{
  bool result = false;
  if(blueToothBuffer2[blueToothCompareStep] == rxData)
  {
    blueToothCompareStep++;
  }else{
    blueToothCompareStep = 0;
  }
  if(blueToothCompareStep == 6){
    result = true;
    blueToothCompareStep = 0;
  }
  return result;
}

unsigned char getBluetoothCMD(){
  if(blueToothShouldConnect && blueToothCount >= 300){
    blueToothShouldConnect = FALSE;
    return TRUE;
  }else{
    return FALSE; 
  }
}

unsigned char* getBluetoothAddr(){
  return blueToothAddr;
}

void bluetoothCountIncrease(){
  if(blueToothShouldConnect){
    blueToothCount++;
  }
}
**********************************bluetooth end*****fww**********************/