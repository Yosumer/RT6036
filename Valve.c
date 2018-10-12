#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "efm32_types.h"
#include "efm32_def.h"
#include "Hot_Roller.h"
#include "Valve.h"
#include "Input.h"
#include "FlexPad.h"
#define VALVE_DATA_LENGTH 2
unsigned char ucSendData[VALVE_DATA_LENGTH];
unsigned char ucReceiveData[VALVE_DATA_LENGTH];
unsigned char* pValveData = ucSendData;
unsigned char* pInputData = ucReceiveData;
unsigned int w_RollerCounter,bSholderEnable;
unsigned int SholderTime = 0;
static bool bAutoRoller;
static bool bRollerEnableFungares;
static unsigned int bRollerClockFungares;
//手动模式PWM设置
__no_init unsigned int w_RollerPWM;
static bool bValveFlag,bRollerFlag;
extern StretchStruct st_Stretch;
__no_init static unsigned char nKeyAirBagStrength ;
//74595串并转换(4片74595)
BITS BITS_ValveData[2] ;
bool bBackauto;
void Valve_Initial_IO(void)
{
    USART_TypeDef *spi = VALVE_SPI;
    USART_InitSync_TypeDef InitSync_Init = VALVE_USART_INITSYNC;
    
    /* Clearing old transfers/receptions, and disabling interrupts */
    spi->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
    spi->IEN = 0;
    
    /* Clear previous interrupts */
    spi->IFC = _USART_IFC_MASK;
    
    USART_InitSync(spi,&InitSync_Init);
    
    /* Enabling pins and setting location */
    spi->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | USART_ROUTE_CLKPEN | VALVE_SPI_ROUTE_LOCAITON;
  /*
    USART_TypeDef *spi = VALVE_SPI;
    USART_InitSync_TypeDef InitSync_Init = VALVE_USART_INITSYNC;
    
    spi->CMD   = USART_CMD_MASTEREN | USART_CMD_TXEN ;
    spi->CTRL |= USART_CTRL_AUTOCS;
    
    spi->IEN = 0;
    
    spi->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_CLKPEN | VALVE_SPI_ROUTE_LOCAITON;
    
    spi->IFC = _USART_IFC_MASK;
    
    USART_InitSync(spi,&InitSync_Init);
    
     IO configuration */
   // GPIO_PinModeSet(VALVE_POWER_PORT,VALVE_POWER_BIT,VALVE_POWER_MODE,0);
    GPIO_PinModeSet(VALVE_LOAD_PORT,VALVE_LOAD_BIT,VALVE_LOAD_MODE,0);
    GPIO_PinModeSet(VALVE_CLK_PORT,VALVE_CLK_BIT,VALVE_CLK_MODE,0);
    GPIO_PinModeSet(VALVE_LE_PORT,VALVE_LE_BIT,VALVE_LE_MODE,0);
    GPIO_PinModeSet(VALVE_DATA_PORT,VALVE_DATA_BIT,VALVE_DATA_MODE,1);
    GPIO_PinModeSet(VALVE_DATA_IN_PORT,VALVE_DATA_IN_BIT,VALVE_DATA_IN_MODE,1);
    GPIO_PinModeSet(VALVE_AIRPUMP1_PORT,VALVE_AIRPUMP1_BIT,VALVE_AIRPUMP1_MODE,0);
    BITS_ValveData[0].nByte = 0;
    BITS_ValveData[1].nByte = 0;
}

void Valve_Initial_Data(void)
{
  w_RollerPWM = ROLLER_SPEED_STOP;
  nKeyAirBagStrength = 0;
}

void Valve_SetData(void)
{
  GPIO_PinOutSet(VALVE_DATA_PORT,VALVE_DATA_BIT);
}

void Valve_ClearData(void)
{
  GPIO_PinOutClear(VALVE_DATA_PORT,VALVE_DATA_BIT);
}

void Valve_SetClock(void)
{
  GPIO_PinOutSet(VALVE_CLK_PORT,VALVE_CLK_BIT);
}
void Valve_ClearClock(void)
{
  GPIO_PinOutClear(VALVE_CLK_PORT,VALVE_CLK_BIT);
}

void Valve_ClearLatch(void)
{
  GPIO_PinOutClear(VALVE_LE_PORT,VALVE_LE_BIT);
}
void Valve_SetLatch(void)
{
  GPIO_PinOutSet(VALVE_LE_PORT,VALVE_LE_BIT);
}
void Valve_10ms_Int(void)
{
  //bValveFlag = true;
  bRollerFlag = true;
  w_RollerCounter++;
  SholderTime++;
}
void Valve_1ms_Int(void)
{
  bValveFlag = true;
}

static unsigned char SPI_FlashWrite(unsigned char data)
{
  VALVE_SPI->TXDATA = data;
  while (!(VALVE_SPI->STATUS & USART_STATUS_TXC))
  {
  }
  return (uint8_t)(USART1->RXDATA);
}

/*本程序10ms执行一次，放在主程序中，数据长度固定为4*/
void Valve_Send_Data(void)
{
  unsigned int i;
  unsigned char ucLength = VALVE_DATA_LENGTH;
  
  if(!bValveFlag) return;
  bValveFlag = false;
  
  for(int i=0;i<2;i++)
  {
    *(ucSendData + i) = BITS_ValveData[i].nByte;
  }
  
  GPIO_PinOutSet(VALVE_LOAD_PORT,VALVE_LOAD_BIT);
  
  
  for(i = 0;i < ucLength;i++)
  {
    *(ucReceiveData + i) = SPI_FlashWrite(*(ucSendData + i));
  }
  /*Waiting for transmission of last byte */
  // while (!(spi->STATUS & USART_STATUS_TXC)) ;
  
  for(i = 100;i > 0;i--)__no_operation();
  
  GPIO_PinOutSet(VALVE_LE_PORT,VALVE_LE_BIT);
  
  for( i = 100;i > 0;i--)__no_operation();
  
  GPIO_PinOutClear(VALVE_LE_PORT,VALVE_LE_BIT);
  
  GPIO_PinOutClear(VALVE_LOAD_PORT,VALVE_LOAD_BIT);
}

void LegFootAirBagAction(bool Enable,unsigned int action)
{    

  if(!Enable)
  {
    ValveFungares11 = VALVE_OFF;
    ValveFungares12 = VALVE_OFF;
    ValveFungares13 = VALVE_OFF;
    ValveFungares14 = VALVE_OFF;
    return;
  }
    if(action & PE2)
  {
    Valve_LegFootAirPumpACPowerOn();
  }
  else
  {
    Valve_LegFootAirPumpACPowerOff();
  }
  if(action & PV11)
  {
    ValveFungares11 = VALVE_ON ;
  }
  else
  {
    ValveFungares11 = VALVE_OFF ;
  }
  
  if(action & PV12)
  {
    ValveFungares12 = VALVE_ON ;
  }
  else
  {
    ValveFungares12 = VALVE_OFF ;
  }
  
  if(action & PV13)
  {
    ValveFungares13 = VALVE_ON ;
  }
  else
  {
    ValveFungares13 = VALVE_OFF ;
  }
  
  if(action & PV14)
  {
    ValveFungares14 = VALVE_ON ;
  }
  else
  {
    ValveFungares14 = VALVE_OFF ;
  }
}

void SeatAirBagAction(bool Enable,unsigned int action)
{
  if(!Enable)
    {
      ValveFungares7       =  VALVE_OFF ;
      ValveFungares8 	   =  VALVE_OFF ;
      return;
    }
    if(action & PE2)
  {
    Valve_LegFootAirPumpACPowerOn();
  }
  else
  {
    Valve_LegFootAirPumpACPowerOff();
  }
  if(action & PV7)
      {
        ValveFungares7 = VALVE_ON ;
      }
      else
      {
        ValveFungares7 = VALVE_OFF ;
      }
  if(action & PV8)
      {
        ValveFungares8 = VALVE_ON ;
      }
      else
      {
        ValveFungares8 = VALVE_OFF ;
      }
}

void ArmSholderAirBagAction(bool Enable,unsigned int action)
{    
  if(!Enable)
  {
    ValveFungares1   =  VALVE_OFF ;
    ValveFungares2   =  VALVE_OFF ;
    ValveFungares3   =  VALVE_OFF ;
    ValveFungares4    =  VALVE_OFF ;
    ValveFungares5    =  VALVE_OFF ;
    ValveFungares6    =  VALVE_OFF ;
    return;
  }
  
  unsigned short nCurWalkMotorLocate = Input_GetWalkMotorPosition();
    if(action & PE2)
  {
    Valve_LegFootAirPumpACPowerOn();
  }
  else
  {
    Valve_LegFootAirPumpACPowerOff();
  }
  if(action & PV1)
  {
    ValveFungares1 = VALVE_ON ;
  }
  else
  {
    ValveFungares1 = VALVE_OFF ;
  }
  if(action & PV2)
  {
    ValveFungares2 = VALVE_ON ;
  }
  else
  {
    ValveFungares2 = VALVE_OFF ;
  }
   if(action & PV3)
  {
    ValveFungares3 = VALVE_ON ;
  }
  else
  {
    ValveFungares3 = VALVE_OFF ;
  }
 if(action & PV4)
  {
    ValveFungares4 = VALVE_ON ;
  }
  else
  {
    ValveFungares4 = VALVE_OFF ;
  }
   if(action & PV5)
  {
    ValveFungares5 = VALVE_ON ;
  }
  else
  {
    ValveFungares5 = VALVE_OFF ;
  }
   if(action & PV6)
  {
    ValveFungares6 = VALVE_ON ;
  }
  else
  {
    ValveFungares6 = VALVE_OFF ;
  }

  if(nCurWalkMotorLocate > 230 && bBackauto)  //机芯超过这个高度不允许动作
  {
    ValveFungares1 = VALVE_OFF ;
    ValveFungares2 = VALVE_OFF ;
    return;
  }
}


bool AirBagGetNextStep(st_AirBag* pBag)
{
  bool bNextStep = FALSE;
  unsigned char counter = pBag->nAirBagCounter;
  unsigned int action = pBag->nCurPumpValveState;
  switch(nKeyAirBagStrength)
  {
  case 1:
    if(counter > pBag->nCurKeepTime1)
    {
      bNextStep = TRUE ;
    }
    break ;
  case 2:
    if(counter > pBag->nCurKeepTime2)
    {
      bNextStep = TRUE ;
    }
    break ;
  case 3:
    if(counter > pBag->nCurKeepTime3)
    {
      bNextStep = TRUE ;
    }
  case 4:
    if(counter > pBag->nCurKeepTime4)
    {
      bNextStep = TRUE ;
    }
    break ;
  case 5:
    if(counter > pBag->nCurKeepTime5)
    {
      bNextStep = TRUE ;
    }
    break ;
  }
  /*********************fww************************/
  unsigned short nCurWalkMotorLocate = Input_GetWalkMotorPosition();
  if((nCurWalkMotorLocate > 230) && (bBackauto) && ((action & PV1) ||(action & PV2)))
  {
    bNextStep = TRUE ;
  }
  /*******************fww**************************/
  return (bNextStep);
}

void Valve_CloseAll(void)
{
  BITS_ValveData[0].nByte = 0;
  BITS_ValveData[1].nByte = 0;
  Valve_LegFootAirPumpACPowerOff();
}

void Valve_SetStretchUp(void)
{
  Valve_LegFootAirPumpACPowerOff();

    ValveFungares1    =  VALVE_OFF ;
    ValveFungares2    =  VALVE_OFF ;
    ValveFungares3    =  VALVE_OFF ;
    ValveFungares4    =  VALVE_OFF ;
    ValveFungares5    =  VALVE_OFF ;
    ValveFungares6    =  VALVE_OFF ;
    ValveFungares7    =  VALVE_OFF ;
    ValveFungares8    =  VALVE_OFF ;
    
    ValveFungares11 = VALVE_OFF;
    ValveFungares12 = VALVE_OFF;
    ValveFungares13 = VALVE_OFF;
    ValveFungares14 = VALVE_OFF;
}

void Valve_SetStretchCharge(unsigned int start)
{
   Valve_LegFootAirPumpACPowerOn();
  static int step = 0;
  if(start)
  {
   step =  0; 
   SholderTime = 0;
  }
  //臂肩气囊
  switch(step)
  {
   case 0: ValveFungares1  = VALVE_ON ; 
           ValveFungares2  = VALVE_ON ; 
           SholderTime = 0;
           step++;
           break;
   case 1: if(SholderTime > 500)
           step++;
           break;       
   case 2: ValveFungares1  = VALVE_ON ; 
           ValveFungares2  = VALVE_ON ;
           step++;
           SholderTime = 0;
           break;       
   case 3: if(SholderTime > 200)
           step = 0;
           break;               
  }
    ValveFungares3    =  VALVE_OFF ;
    ValveFungares4    =  VALVE_OFF ;
    ValveFungares5    =  VALVE_OFF ;
    ValveFungares6    =  VALVE_OFF ;
    ValveFungares7    =  VALVE_OFF ;
    ValveFungares8    =  VALVE_OFF ;
    
    ValveFungares11 = VALVE_ON;
    ValveFungares12 = VALVE_OFF;
    ValveFungares13 = VALVE_ON;
    ValveFungares14 = VALVE_OFF;
}

void Valve_SetStretchHold(void)
{
   Valve_LegFootAirPumpACPowerOn();
   
    ValveFungares1    =  VALVE_ON ; 
    ValveFungares2    =  VALVE_ON ;
    ValveFungares3    =  VALVE_OFF ;
    ValveFungares4    =  VALVE_OFF ;
    ValveFungares5    =  VALVE_OFF ;
    ValveFungares6    =  VALVE_OFF ;
    ValveFungares7    =  VALVE_OFF ;
    ValveFungares8    =  VALVE_OFF ;
    
    ValveFungares11 = VALVE_ON;
    ValveFungares12 = VALVE_ON;
    ValveFungares13 = VALVE_ON;
    ValveFungares14 = VALVE_OFF;
}

void AutoAirBagAction(bool Enable,unsigned int action)
{ 
  unsigned short nCurWalkMotorLocate = Input_GetWalkMotorPosition();
  if(!Enable)
  {
    ValveFungares1    =  VALVE_OFF ;
    ValveFungares2    =  VALVE_OFF ;
    ValveFungares3    =  VALVE_OFF ;
    ValveFungares4    =  VALVE_OFF ;
    ValveFungares5    =  VALVE_OFF ;
    ValveFungares6    =  VALVE_OFF ;
    ValveFungares7    =  VALVE_OFF ;
    ValveFungares8    =  VALVE_OFF ;
    
    ValveFungares11 = VALVE_OFF;
    ValveFungares12 = VALVE_OFF;
    ValveFungares13 = VALVE_OFF;
    ValveFungares14 = VALVE_OFF;
    
    Valve_LegFootAirPumpACPowerOff();
    return;
  }
  if(action & PE2)
  {
    Valve_LegFootAirPumpACPowerOn();
  }
  else
  {
    Valve_LegFootAirPumpACPowerOff();
  }
  if(action & PV1)
  {
    ValveFungares1 = VALVE_ON ;
  }
  else
  {
    ValveFungares1 = VALVE_OFF ;
  }
  if(action & PV2)
  {
    ValveFungares2 = VALVE_ON ;
  }
  else
  {
    ValveFungares2 = VALVE_OFF ;
  }
   if(action & PV3)
  {
    ValveFungares3 = VALVE_ON ;
  }
  else
  {
    ValveFungares3 = VALVE_OFF ;
  }
 if(action & PV4)
  {
    ValveFungares4 = VALVE_ON ;
  }
  else
  {
    ValveFungares4 = VALVE_OFF ;
  }
   if(action & PV5)
  {
    ValveFungares5 = VALVE_ON ;
  }
  else
  {
    ValveFungares5 = VALVE_OFF ;
  }
   if(action & PV6)
  {
    ValveFungares6 = VALVE_ON ;
  }
  else
  {
    ValveFungares6 = VALVE_OFF ;
  }
  if(action & PV7)
      {
        ValveFungares7 = VALVE_ON ;
      }
      else
      {
        ValveFungares7 = VALVE_OFF ;
      }
  if(action & PV8)
      {
        ValveFungares8 = VALVE_ON ;
      }
      else
      {
        ValveFungares8 = VALVE_OFF ;
      }
  
  
  if(action & PV11)
  {
    ValveFungares11 = VALVE_ON ;
  }
  else
  {
    ValveFungares11 = VALVE_OFF ;
  }
  
  if(action & PV12)
  {
    ValveFungares12 = VALVE_ON ;
  }
  else
  {
    ValveFungares12 = VALVE_OFF ;
  }
  
  if(action & PV13)
  {
    ValveFungares13 = VALVE_ON ;
  }
  else
  {
    ValveFungares13 = VALVE_OFF ;
  }
  
  if(action & PV14)
  {
    ValveFungares14 = VALVE_ON ;
  }
  else
  {
    ValveFungares14 = VALVE_OFF ;
  }
  if(nCurWalkMotorLocate > 230 && bBackauto)  //机芯超过这个高度不允许动作
  {
    ValveFungares1 = VALVE_OFF ;
    ValveFungares2 = VALVE_OFF ;
    return;
  }
}

unsigned char Valve_Level_Decrease(unsigned char by_Data)
{
  unsigned char retval;
  unsigned int w_Data;
  unsigned int mod;
  if(by_Data <= 5) 
  {
    retval = by_Data;
  }
  else
  {
    w_Data = by_Data;
    w_Data *= 10;
    mod = w_Data % 15;
    w_Data /= 15;
    if(mod > 7) w_Data++;
    by_Data = (unsigned char)w_Data;
    retval = by_Data;
  }
  return retval;
}

unsigned char Valve_Level_Increase(unsigned char by_Data)
{
  unsigned char retval;
  unsigned int w_Data;
  unsigned int mod;
  if(by_Data <= 5) 
  {
    retval = by_Data;
  }
  else
  {
    w_Data = by_Data;
    w_Data *= 15;
    w_Data /= 10;
    mod = w_Data % 10;
    if(mod > 5) w_Data++;
    if(w_Data > 255) w_Data = 255;
    by_Data = (unsigned char)w_Data;
    retval = by_Data;
  }
  return retval;
}

void Valve_SetEnableSholder(unsigned int enable)
{
  bSholderEnable = enable;
}

void Valve_Control(unsigned char nAirBagSwitch,st_AirBag* pBag,unsigned char level)
{
	bool bNextStep = FALSE ;
	if(nAirBagSwitch == VALVE_DISABLE)
	{
		switch(pBag->locate)
		{
			case AIRBAG_LOCATE_LEG_FOOT:        LegFootAirBagAction(FALSE,pBag->nCurPumpValveState); break;
			case AIRBAG_LOCATE_SEAT:            SeatAirBagAction(FALSE,pBag->nCurPumpValveState); break;
			case AIRBAG_LOCATE_ARM_SHOLDER:     ArmSholderAirBagAction(FALSE,pBag->nCurPumpValveState); break;
			case AIRBAG_LOCATE_AUTO:            AutoAirBagAction(FALSE,pBag->nCurPumpValveState);  break;     
		}
		return; 
	}
	if(pBag->init == TRUE)
	{
		bNextStep = TRUE ;
	}
	else
	{
		bNextStep = AirBagGetNextStep(pBag);   
	}
	if(bNextStep == TRUE)
	{
		if(pBag->locate == AIRBAG_LOCATE_AUTO)
		{
			w_RollerCounter = 0;
		}
		if(pBag->init == TRUE)
		{
			pBag->init = FALSE ;
			pBag->nCurAirBagStep = 0 ;
			pBag->nAirBagCounter = 0 ;
		}
		else
		{
			pBag->nCurAirBagStep++ ;
			if(pBag->nCurAirBagStep >= pBag->nTotalSteps)
			{
				pBag->nCurAirBagStep = 0 ;
			}
		}
		pBag->nCurPumpValveState = pBag->pAirBagArray[pBag->nCurAirBagStep].nPumpValveState ;
		pBag->nCurKeepTime1 = pBag->pAirBagArray[pBag->nCurAirBagStep].nKeepTime1 ;
		pBag->nCurKeepTime3 = pBag->pAirBagArray[pBag->nCurAirBagStep].nKeepTime2 ;
		pBag->nCurKeepTime5 = pBag->pAirBagArray[pBag->nCurAirBagStep].nKeepTime3 ;
   
		if(pBag->nCurPumpValveState != ALL_DIS)
		{
			if(level  == 0)
			{
				pBag->nCurKeepTime1 = Valve_Level_Decrease(pBag->nCurKeepTime1);
				pBag->nCurKeepTime3 = Valve_Level_Decrease(pBag->nCurKeepTime3);
				pBag->nCurKeepTime5 = Valve_Level_Decrease(pBag->nCurKeepTime5);
			}
			if(level  == 2)
			{
				pBag->nCurKeepTime1 = Valve_Level_Increase(pBag->nCurKeepTime1);
				pBag->nCurKeepTime3 = Valve_Level_Increase(pBag->nCurKeepTime3);
				pBag->nCurKeepTime5 = Valve_Level_Increase(pBag->nCurKeepTime5);
			}
		}
		pBag->nCurKeepTime2 = Middle(pBag->nCurKeepTime1,pBag->nCurKeepTime3);
		pBag->nCurKeepTime4 = Middle(pBag->nCurKeepTime3,pBag->nCurKeepTime5);
		pBag->nAirBagCounter = 0 ;
		switch(pBag->locate)
		{
			case AIRBAG_LOCATE_LEG_FOOT:        LegFootAirBagAction(TRUE,pBag->nCurPumpValveState); break;
			case AIRBAG_LOCATE_SEAT:            SeatAirBagAction(TRUE,pBag->nCurPumpValveState); break;
			case AIRBAG_LOCATE_ARM_SHOLDER:     ArmSholderAirBagAction(TRUE,pBag->nCurPumpValveState); break;
			case AIRBAG_LOCATE_AUTO:            AutoAirBagAction(TRUE,pBag->nCurPumpValveState);  break; 
		}
		if((BITS_ValveData[0].nByte == 0)&&(BITS_ValveData[1].nByte == 0))
		{
			Valve_LegFootAirPumpACPowerOff();
		}
	}
}

//24-31位为滚轮旋转方式
/*  0-1位为滚轮速度 00 停止 01 慢速 10 中速 11 高速
    2-3位为滚轮旋转方式 01短间歇 02长间歇 10 连续
    4-7位保留
*/
void Valve_FootRollerProce(unsigned char bRollerEnable,unsigned char Valve_Enable,st_AirBag* pBag)
{ 
  unsigned int RollerSpeed,RollerPWM;
  unsigned int RollerCounter = w_RollerCounter;
  if(bRollerFlag == 0) return;
  bRollerFlag = 0;
  if(!bRollerEnable)
  {
   // RollerMotor_Set_Pwm_Data(HOT_ROLLER_DEFAULT_TOP);
    RollerMotor_Break();
    bRollerEnableFungares = false;
    return;
  }
  bRollerEnableFungares = true;
  if(Valve_Enable)  //自动滚轮模式
  {
    bAutoRoller = true;
    RollerSpeed = ((pBag->nCurPumpValveState & 0x03000000) >> 24);
    switch(RollerSpeed)
    {
     case 0:  RollerPWM = ROLLER_SPEED_STOP; break;
     case 1:  RollerPWM = ROLLER_SPEED_SLOW; break;
     case 2:  RollerPWM = ROLLER_SPEED_MID;  break;
     case 3:  RollerPWM = ROLLER_SPEED_FAST; break;
    }
    if((pBag->nCurPumpValveState & 0x0c000000) == ROLLER_INTERMITTENT ) 
    {
      if(RollerCounter >= ROLLER_INTERMITTENT_TIME)
      {
        RollerCounter %= ROLLER_INTERMITTENT_TIME;
      }
      if(RollerCounter >= ROLLER_INTERMITTENT_ON_TIME) 
        RollerPWM = 0;
    }
    if((pBag->nCurPumpValveState & 0x0c000000) == ROLLER_SEMI_CIRCLE ) 
    {
      if(RollerCounter >= ROLLER_SEMI_CIRCLE_TIME)
      {
        RollerCounter %= ROLLER_SEMI_CIRCLE_TIME;
      }
      if(RollerCounter >= ROLLER_SEMI_CIRCLE_ON_TIME) RollerPWM = HOT_ROLLER_DEFAULT_TOP;
    }
    bRollerClockFungares = pBag->nCurPumpValveState&ROLLER_PHASE;
    RollerMotor_Control(RollerPWM,pBag->nCurPumpValveState&ROLLER_PHASE);  //气阀控制滚轮
    w_RollerCounter = RollerCounter;
   return; 
  }
  bAutoRoller = false;  //手动滚轮模式
  RollerMotor_Control(w_RollerPWM,1);
}

void Valve_SetRollerPWM(unsigned char level)
{
  switch(level)
   {
     case 0:  w_RollerPWM = ROLLER_SPEED_STOP; break;
     case 1:  w_RollerPWM = ROLLER_SPEED_SLOW; break;
     case 2:  w_RollerPWM = ROLLER_SPEED_MID;  break;
     case 3:  w_RollerPWM = ROLLER_SPEED_FAST; break;
    }
}

unsigned char Valve_GetRollerLevel(void)
{
    unsigned char level = 0;
//    if(w_RollerPWM == ROLLER_SPEED_STOP) level = 0;
    if(w_RollerPWM == ROLLER_SPEED_SLOW) level = 1;
    if(w_RollerPWM == ROLLER_SPEED_MID) level = 2;
    if(w_RollerPWM == ROLLER_SPEED_FAST) level = 3;
    
    return level;
}

unsigned char Valve_GetAirBagStrength(void)
{
  return(nKeyAirBagStrength);
}

void Valve_SetAirBagStrength(unsigned char strength)
{
	nKeyAirBagStrength = strength;
}

void Valve_AddAirBagStrength(void)
{
  nKeyAirBagStrength++;
  if(nKeyAirBagStrength > 5) nKeyAirBagStrength =1;
}
/*
void Valve_SetAirBagStrengthWeak(void)
{
  nKeyAirBagStrength =1;
}
void Valve_SetAirBagStrengthMed(void)
{
  nKeyAirBagStrength =3;
}
void Valve_SetAirBagStrengthStrong(void)
{
  nKeyAirBagStrength =5;
}
*/

//小腿和臀部
void Valve_LegFootAirPumpACPowerOn(void)
{
  GPIO_PinOutSet(VALVE_AIRPUMP1_PORT,VALVE_AIRPUMP1_BIT);
}
void Valve_LegFootAirPumpACPowerOff(void)
{
  GPIO_PinOutClear(VALVE_AIRPUMP1_PORT,VALVE_AIRPUMP1_BIT);
}

void Valve_Test_Set_Data(unsigned int ValveTestData)
{
  BITS_ValveData[0].nByte = (unsigned char)ValveTestData;
  BITS_ValveData[1].nByte = (unsigned char)(ValveTestData >> 8);
}

int Valve_RollerIsAuto(void)
{
 return (bAutoRoller) ;
}
void Valve_SetBackMode(int backauto)
{
  bBackauto = (bool)backauto;
}

unsigned char Roller_GetRollerDirection(void)
{
  unsigned char dir = 0;
  if(bRollerEnableFungares)
  {
    //手动
    if(!bAutoRoller)
    {
      dir = 3;
    }
    //自动
    else
    {
      //正
      if(bRollerClockFungares) dir = 1;
      //反
      else dir = 2;
    }
  }
  else dir = 0;
  return dir;
}

