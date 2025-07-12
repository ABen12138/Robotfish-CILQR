#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "motor.h"
#include "usart3.h"
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
int x=0;

signed long aaa=0;
#define SDO_COBID_C2S       0x600   // COB-ID SDO client to server, 
#define SDO_COBID_S2C       0x580   // COB-ID SDO server to client
//CAN初始化
//tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:时间段2的时间单元.   范围:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:时间段1的时间单元.   范围:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :波特率分频器.范围:1~1024; tq=(brp)*tpclk1
//波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
//Fpclk1的时钟在初始化的时候设置为42M,如果设置CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//则波特率为:42M/((6+7+1)*6)=500Kbps
//返回值:0,初始化OK;
//    其他,初始化失败; 
CanRxMsg Rx_motor_Message;
MaxonMsg Motor_msg;
//CAN1接收RX0中断使能
	//0,不使能;1,使能.		

CanTxMsg NMT_msg = {0x000, 1, CAN_Id_Standard, CAN_RTR_Data, 8, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
CanRxMsg NMT_rmsg = {0x000, 1, CAN_Id_Standard, CAN_RTR_Data, 8, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};


#define CMDID_VOL       0x220001
#define CMDID_CUR       0x30D102
#define CMDID_POS       0x606400
#define CMDID_VEL       0x606C00
#define CMDID_TOR       0x30D201
#define CMDID_TMP       0x320101


#define TPDO1_COBID          0x40000180
#define TPDO2_COBID          0x40000280
#define TPDO3_COBID          0x40000380

#define TPDO1_CUR       0x00000181
#define TPDO2_VEL       0x00000281
#define TPDO3_TOR       0x00000381



void Enter_Pre_Operational(uint8_t idx)  // 
{
    NMT_msg.Data[0] = 0x80;
    NMT_msg.Data[1] = idx;
    CAN_Transmit(CAN1, &NMT_msg);
}
void Start_Remote_Node(uint8_t idx)      // ??Operational??, NMT, SDO, PDO???, ??????TPDO, ????????TPDO
{
    NMT_msg.Data[0] = 0x01;
    NMT_msg.Data[1] = idx;
    CAN_Transmit(CAN1, &NMT_msg);
	 // CAN_Receive(CAN1, CAN_FIFO0, &NMT_rmsg);
}
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{

  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN1_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //使能相关时钟
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PORTA时钟	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	
    //初始化GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA11,PA12
	
	  //引脚复用映射配置
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11复用为CAN1
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12复用为CAN1
	  
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1 
    
		//配置过滤器
 	  CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
		
#if CAN1_RX0_INT_ENABLE
	
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}   
 
#if CAN1_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数			    
void CAN1_RX0_IRQHandler(void)
{
	int i=0;

    CAN_Receive(CAN1, 0, &Rx_motor_Message);
//	for(i=0;i<8;i++)
//	printf("rxbuf[%d]:%d\r\n",i,RxMessage.Data[i]);
		CAN_RX_Handle(Rx_motor_Message);
		//CAN_Handle_SDO(&Rx_motor_Message);
//		if(Can_Receive_Flag==1)
//	{
//		u3_printf("CAN1_RX0_IRQHandler\r\n");		
//	}
}
#endif

//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x12;	 // 标准标识符为0
  TxMessage.ExtId=0x12;	 // 设置扩展标示符（29位）
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=len;							 // 发送两帧信息
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 1;
  return 0;		

}
//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}

void CAN_RX_Handle(CanRxMsg msg)
{
		if(Can_Receive_Flag==1)
	{
	//u3_printf("CAN_RX_Handle \r\n");		
	if((msg.StdId & 0xFFF0) == SDO_COBID_S2C) 
	{
		Cvt_CanRxMsg_2_MaxonMsg(&Motor_msg, &msg); 
	if(((Motor_msg.Index << 8) + Motor_msg.SubIndex)==CMDID_VOL)
	{
	  Battery_Voltage = (msg.Data[4] & 0xFFFF) * 0.1; 
		u3_printf("Vol%0.1f ",Battery_Voltage);
	}
	else if(((Motor_msg.Index << 8) + Motor_msg.SubIndex)==CMDID_VEL)
	{
		Motor_Velocity=msg.Data[4]+ (msg.Data[5] << 8)+(msg.Data[6] << 16)+(msg.Data[7] << 24);	
		u3_printf("Vel%d ",Motor_Velocity);
	}
		else if(((Motor_msg.Index << 8) + Motor_msg.SubIndex)==CMDID_CUR)
	{
		Motor_Current=msg.Data[4]+ (msg.Data[5] << 8)+(msg.Data[6] << 16)+(msg.Data[7] << 24);	
		u3_printf("Cur%d ",Motor_Current);
	}
			else if(((Motor_msg.Index << 8) + Motor_msg.SubIndex)==CMDID_TOR)
	{
		Motor_Torque=(msg.Data[4]+ (msg.Data[5] << 8)+(msg.Data[6] << 16)+(msg.Data[7] << 24))*0.1;	
		u3_printf("Tor%0.1f ",Motor_Torque);
	}
	else ;
		//u3_printf("Error!!!!!!!!!!!!!!!!!!!!!!!! \r\n");
	}
	else if(msg.StdId  == TPDO1_CUR) 
	{
		Motor_Current = msg.Data[0]+ (msg.Data[1] << 8)+(msg.Data[2] << 16)+(msg.Data[3] << 24);
		u3_printf("Current:%d \r\n",Motor_Current);		
	}
	else if(msg.StdId  == TPDO2_VEL) 
	{
		Motor_Velocity=msg.Data[0]+ (msg.Data[1] << 8)+(msg.Data[2] << 16)+(msg.Data[3] << 24);	
		u3_printf("Velocity:%d \r\n",Motor_Velocity);	
	}
	else if(msg.StdId  == TPDO3_TOR)
	{
		Motor_Torque=((float)(msg.Data[0]+ (msg.Data[1] << 8)))*0.1;
		u3_printf("Torque:%f \r\n",Motor_Velocity);	
	}
	else ;
}
}


static void CAN_Handle_SDO(CanRxMsg *msg)
{
	;
}










