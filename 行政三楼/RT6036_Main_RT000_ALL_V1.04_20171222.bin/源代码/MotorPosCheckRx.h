#ifndef __MOTORPOSCHECK_H
#define __MOTORPOSCHECK_H


/* Macro --------------------------------------------------*/
//RX
#define MOTOR_POSCHECK_RX_PORT     gpioPortC
#define MOTOR_POSCHECK_RX_BIT      15
#define MOTOR_POSCHECK_RX_MODE     gpioModeInputPull
//BUFFER LENGTH
#define MOTOR_BUFFER_LENGTH        5
//
#define KNEAD_WIDTH_UNKNOWN		0
#define KNEAD_WIDTH_MIN			1
#define KNEAD_WIDTH_MED			2
#define KNEAD_WIDTH_MAX			3


#define TAP_MOTOR_REACH_POS		1


/*Function Prototype  -----------------------------*/
unsigned int Input_GetKneadMax(void);
unsigned int Input_GetKneadMid(void);
unsigned int Input_GetKneadMin(void);
unsigned int Input_GetKneadPosition(void);
unsigned char Input_CurTapMotorPos(void);
void MotorPosCheck_Init(void);
unsigned int Input_GetVout(void);


#endif //__MOTORPOSCHECK_H