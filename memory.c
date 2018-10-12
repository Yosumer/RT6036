#include "efm32.h"
#include "em_chip.h"
#include "EFM32_def.h"
#include "EFM32_types.h"
#include "string.h"
#include "em_msc.h"
#include "memory.h"

unsigned int membuf[64];

void MEM_Initial(void)
{
}

//Internal information word Read
unsigned char ReadEEByte(unsigned int nAddress)
{
  uint32_t* p;
  p = (uint32_t*)(nAddress);
  return((unsigned char)*p);
}

void MEM_Write_Memory(PUINT32 pw_Buffer,int numBytes)
{
  MSC_Init();
     __disable_irq();

  MSC_ErasePage((uint32_t*)USER_DATA_BASE);
  
  MSC_WriteWord((uint32_t*)USER_DATA_BASE, pw_Buffer, numBytes);
  
    __enable_irq();
  MSC_Deinit();
}

void MEM_Read_Memory(PUINT32 pw_Buffer,int numBytes)
{
  memcpy(pw_Buffer,(uint32_t*)(USER_DATA_BASE),numBytes); 
}

//获取累计工作时间 单位秒
unsigned int MEM_Get_Total_Time(void)
{
  unsigned int data;
  unsigned int retTotalTime = 0;
  for(int i=0;i<64;i++)
  {
     data = *(uint32_t*)(i*4 + ACC_DATA_BASE); 
     if(data == 0xffffffff) break;
     retTotalTime = data;   
  }
  return(retTotalTime);
}
//存储累计工作时间 单位秒
void MEM_Save_Total_Time(unsigned int totalTime)
{
  unsigned int data;
  
  for(int i=0;i<64;i++)
  {
     data = *(uint32_t*)(i*4 + ACC_DATA_BASE); 
     if(data == 0xffffffff) 
     {
       MSC_Init();
     __disable_irq();

       MSC_WriteWord((uint32_t*)(i*4 + ACC_DATA_BASE), &totalTime, 4);
       
     __enable_irq();
      MSC_Deinit();

       return;
     }
  }
  //整页已满
  
  MSC_Init();
     __disable_irq();


  memcpy(membuf,(uint32_t*)(USER_DATA_BASE),64*4);       //将数据copy 到缓冲区
     
  MSC_ErasePage((uint32_t*)USER_DATA_BASE);  //擦除整页
  
  MSC_WriteWord((uint32_t*)USER_DATA_BASE, membuf, 64*4); //写前半页
   
  MSC_WriteWord((uint32_t*)ACC_DATA_BASE, &totalTime, 4); //在后半页的起始地址写累积时间
  
    __enable_irq();
  MSC_Deinit();
  
}