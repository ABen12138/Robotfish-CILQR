#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//CAN驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/7
//版本：V1.0 
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 
#define CAN1_RX0_INT_ENABLE 1
extern int Can_Receive_Flag; 						    
										 							 				    
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化
 
u8 CAN1_Send_Msg(u8* msg,u8 len);						//发送数据

u8 CAN1_Receive_Msg(u8 *buf);							//接收数据

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

















