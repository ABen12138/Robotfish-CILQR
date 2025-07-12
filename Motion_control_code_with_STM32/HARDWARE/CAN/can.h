#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//CAN���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/7
//�汾��V1.0 
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 
#define CAN1_RX0_INT_ENABLE 1
extern int Can_Receive_Flag; 						    
										 							 				    
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ��
 
u8 CAN1_Send_Msg(u8* msg,u8 len);						//��������

u8 CAN1_Receive_Msg(u8 *buf);							//��������

void Enter_Pre_Operational(uint8_t idx);
void Start_Remote_Node(uint8_t idx);
void CAN_RX_Handle(CanRxMsg msg);
static void CAN_Handle_SDO(CanRxMsg *msg);

extern float fre;
extern float bbb;
extern CanRxMsg Rx_motor_Message;
extern float Battery_Voltage;

extern u32 Motor_Current;
extern u32 Motor_Velocity; 
extern float Motor_Torque;

extern CanTxMsg NMT_msg;
extern CanRxMsg NMT_rmsg;
#endif

















