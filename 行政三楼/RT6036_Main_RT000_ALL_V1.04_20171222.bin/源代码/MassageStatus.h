#ifndef __MASSAGE_STATUS_H__
#define __MASSAGE_STATUS_H__

#define CHAIR_STATE_IDLE			0  //����״̬
#define CHAIR_STATE_SETTLE			1  //��λ״̬1�����а�Ħ�ṹ��λ��
#define CHAIR_STATE_WAIT_COMMAND	        2  //�ȴ�������,��ص�ָʾ����˸
#define CHAIR_STATE_RUN				3  //����״̬
#define CHAIR_STATE_WAIT_MEMORY		        4  
#define CHAIR_STATE_PROBLEM			5  //����ģʽ
#define CHAIR_STATE_ENGINEERING                 6  //����ģʽ
#define CHAIR_STATE_SLEEP			7  //���ӹ�����˯��ģʽ����ʱҪ���������С��0.5W
#define CHAIR_STATE_DEMO                        9
#define CHAIR_STATE_SETTLE_1ST			8  //��λ״̬2����ʾ�ղ��У�

#define MOTOR_STOP_BREAK    0  //���ֹͣ��ɲ��
#define MOTOR_STOP_HZ       1  //���ֹͣ���������˿�Ϊ����
#define MOTOR_RUN           2  //�������
#define MOTOR_STOP_HZ_TIME  10  //��λ10ms 

#define MOTOR_ON	1
#define MOTOR_OFF	0

extern unsigned char nChairRunState;
//��������
//#define KR_PROGRAM    1  //Ҫ�����˯�߹���

#define STRETCH_GO_DOWN 0  //��������
#define STRETCH_GO_OUT  1  //��ǰ����


#endif /*__MASSAGE_STATUS_H__*/