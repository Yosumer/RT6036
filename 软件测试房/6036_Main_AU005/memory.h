#include "MassageStatus.h"
#define MEMORY_LENGTH 13
//#define PRODUCT_ID_ADDR         ((uint32_t) (128*1024-5)) //((uint32_t) 0x0000FFFBUL) 


/*******************************************************
ʹ��һ��ҳ������¼�ۼ�ʱ��
�ۼ�ʱ�䵥λΪ�룬ÿ����4���ֽڳ������洢���� �������Ϊ2��32�η�-1��
һ��ҳ��Ϊ512���ֽڹ��ɼ�¼512/4=128��
�����״�ʹ�ñ����Ƚ�������¼��ȫ����������һ�δ����ַ��ʼд��
��һ�δӵ��ĸ���ַ��ʼд��д�����һ����ַ�ٽ���������������
*******************************************************/
#define ACC_DATA_BASE   (USER_DATA_BASE+0x100)

#define USER_DATA_BASE          ((uint32_t) 0x0FE00000UL)  /**< user data flash base address  */
//���µ�ַ�ǰ����ֽڴ��
#define SOFT_MAIN_VER_ADDRESS         0
#define SOFT_SECONDARY_VER_ADDRESS    1
#define SETTLE_ADDRESS                2  //��Ħ����Ƿ�λ ����Ϊ1��λ
#define AIRBAG_STRETCH_ADDRESS        3   //��Ħ���ڲ���������
#define SLIDE_MOTOR_ENABLE_ADDRESS    4   //�������ʹ�����ֹ
#define PROGRAM_ENABLE_ADDRESS        5   //���ʹ�ܵ�ַ
#define STRETCH_OUT_ADDRESS           6
#define SLEEP_PROGRAM_ADDRESS         7
#define SLIDE_BLUETOOTH_POWER         8    //FWW 20150313
#define TIME_SWITCH                   9    //FWW ��ʱ��д�룬��ֹ�����ı�


#define ACC_TIME_0_ADDRESS  0x10
#define ACC_TIME_1_ADDRESS  0x11
#define ACC_TIME_2_ADDRESS  0x12
#define ACC_TIME_3_ADDRESS  0x13

#define MEMORY_LENGTH_OF_BYTES		12    //fww
#define PROGRAM_FLAG			'p'
#define PROGRAM_BY_BLUETOOTH_FLAG	'l'
#define SOFT_MAIN_VER			1
//2.2 ���Ӹ�λʱ�����������ֹͣ
//   �����綯����С�ȵ��������ֶ�ʱΪ3A�ڸ�λʱΪ2A
/*
2.02 ����
2.03 ̨��
2.04 ����
2.05 ����
2.06�濪ʼ̨�塢���桢us047��������������ɹ���ģʽ�л�
*/
#define SOFT_SECONDARY_VER      3

unsigned char ReadEEByte(unsigned int nAddress);
void MEM_Write_Memory(PUINT32 pw_Buffer,int numBytes);
void MEM_Read_Memory(PUINT32 pw_Buffer,int numBytes);

//��ȡ�ۼƹ���ʱ�� ��λ��
unsigned int MEM_Get_Total_Time(void);

//�洢�ۼƹ���ʱ�� ��λ��
void MEM_Save_Total_Time(unsigned int totalTime);
