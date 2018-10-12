#include <stdio.h>
#include "string.h"
#include "MotorPosCheckRx.h"
#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "efm32_types.h"
#include "efm32_def.h"
#include "ControlBox.h"

//
/* Setup UART0#3 in async mode for RS232*/
//static USART_TypeDef           * Motoruart   = UART0;
static USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;
//
unsigned char ucMotorPosRXBuffer[MOTOR_BUFFER_LENGTH] = {0};
BYTE by_KneadPosition = KNEAD_WIDTH_UNKNOWN;


BITS MotorInputData0;
#define bKneadMotorMin		MotorInputData0.bD0   
#define bKneadMotorMed		MotorInputData0.bD1 
#define bKneadMotorMax		MotorInputData0.bD2//
#define bTapMotorPos 	        MotorInputData0.bD3//
#define bShoulderPos            MotorInputData0.bD4//
#define bNC1	                MotorInputData0.bD5//
#define bNC2		        MotorInputData0.bD6
#define bNC3 		        MotorInputData0.bD7


//
unsigned char MotroPosRX_Index;
bool  MotroPosRXOK;



/*
*@brief   :Init The MotorPosCheck Rx's pin
*@param   :no
*@retval  :no
*************************************/
void MotorPosCheck_Init(void)
{
  /* Enable clock for GPIO module (required for pin configuration) */
  // CMU_ClockEnable(cmuClock_GPIO, true);
  /* Configure GPIO pins */
  //GPIO_PinModeSet(DMAUART_TX_PORT,DMAUART_TX_BIT,DMAUART_TX_MODE, 1);
  GPIO_PinModeSet(MOTOR_POSCHECK_RX_PORT,MOTOR_POSCHECK_RX_BIT,MOTOR_POSCHECK_RX_MODE, 0);
  
  /* Prepare struct for initializing UART in asynchronous mode*/
  uartInit.enable       = usartDisable;   /* Don't enable UART upon intialization */
  uartInit.refFreq      = 0;              /* Provide information on reference frequency. When set to 0, the reference frequency is */
  uartInit.baudrate     = 9600;           /* Baud rate */
  uartInit.oversampling = usartOVS16;     /* Oversampling. Range is 4x, 6x, 8x or 16x */
  uartInit.databits     = usartDatabits8; /* Number of data bits. Range is 4 to 10 */
  uartInit.parity       = usartNoParity;  /* Parity mode */
  uartInit.stopbits     = usartStopbits1; /* Number of stop bits. Range is 0 to 2 */
  
  /* Initialize USART with uartInit struct */
  USART_InitAsync(UART0, &uartInit);
  
  /* Prepare UART Rx and Tx interrupts */
  USART_IntClear(UART0, _UART_IF_MASK);
  USART_IntEnable(UART0, UART_IF_RXDATAV);
  NVIC_ClearPendingIRQ(UART0_RX_IRQn);
  //NVIC_ClearPendingIRQ(UART1_TX_IRQn);
  NVIC_EnableIRQ(UART0_RX_IRQn);
  //NVIC_EnableIRQ(UART1_TX_IRQn);
  
  
  /* Enable I/O pins at UART1 location #3 */
  UART0->ROUTE = UART_ROUTE_RXPEN | UART_ROUTE_LOCATION_LOC3;
  
  /* Enable UART */
  USART_Enable(UART0, usartEnable);
}

void UART0_RX_IRQHandler(void)
{
  /* Check for RX data valid interrupt */
  
  if (UART0->STATUS & USART_STATUS_RXDATAV)
  {
    /* Copy data into RX Buffer */
    uint8_t rxData = USART_Rx(UART0);
    //start
    if(rxData == SOI)
    {
      MotroPosRX_Index = 0;  
    }
    else if(rxData == EOI)
    {
      //0
      if(ucMotorPosRXBuffer[0] & 0x01)
      {
        bKneadMotorMin =  0;//0 :认为是检测到了霍尔，为了兼容以前的代码
        by_KneadPosition =  KNEAD_WIDTH_MIN;
      }
      //1
      if(ucMotorPosRXBuffer[0] & 0x02)
      {
        bKneadMotorMed =  0;//0 :认为是检测到了霍尔，为了兼容以前的代码
        by_KneadPosition =  KNEAD_WIDTH_MED;
      }
      //2
      if(ucMotorPosRXBuffer[0] & 0x04)
      {
        bKneadMotorMax =  0;//0 :认为是检测到了霍尔，为了兼容以前的代码
        by_KneadPosition =  KNEAD_WIDTH_MAX;
      }
      //3
      if(ucMotorPosRXBuffer[0] & 0x08)
      {
        bTapMotorPos =  TAP_MOTOR_REACH_POS;
      }
      else
      {
        bTapMotorPos =  0;
      }
      //4
      if(ucMotorPosRXBuffer[0] & 0x10)
      {
        bShoulderPos = 0 ;//141116 change "1"(hex) to "0"
      }
      else
      {
        bShoulderPos = 1 ;
      }
      MotroPosRXOK = true;
    }
    else
    {
      ucMotorPosRXBuffer[MotroPosRX_Index] = rxData;
      MotroPosRX_Index++;
      MotroPosRX_Index %= MOTOR_BUFFER_LENGTH;
    }
    
    /* Clear RXDATAV interrupt */
    USART_IntClear(UART0, UART_IF_RXDATAV);
  }
}

//
//return 0 or 1
unsigned int Input_GetKneadMax(void)
{
  return(bKneadMotorMax);
}
//return 0 or 1
unsigned int Input_GetKneadMid(void)
{
    return(bKneadMotorMed);
}
//return 0 or 1
unsigned int Input_GetKneadMin(void)
{
    return(bKneadMotorMin);
}
//
unsigned int Input_GetKneadPosition(void)
{
  return (unsigned int)by_KneadPosition;
}
//
unsigned char Input_CurTapMotorPos(void)
{
  return(bTapMotorPos);
}
//Shoulder check Pos
unsigned int Input_GetVout(void)
{
  return(bShoulderPos);
}

