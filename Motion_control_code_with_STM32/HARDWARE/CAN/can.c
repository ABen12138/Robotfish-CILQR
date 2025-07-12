#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "motor.h"
#include "usart3.h"
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
int x=0;

signed long aaa=0;
#define SDO_COBID_C2S       0x600   // COB-ID SDO client to server, 
#define SDO_COBID_S2C       0x580   // COB-ID SDO server to client
//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:ʱ���1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024; tq=(brp)*tpclk1
//������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ42M,�������CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//������Ϊ:42M/((6+7+1)*6)=500Kbps
//����ֵ:0,��ʼ��OK;
//    ����,��ʼ��ʧ��; 
CanRxMsg Rx_motor_Message;
MaxonMsg Motor_msg;
//CAN1����RX0�ж�ʹ��
	//0,��ʹ��;1,ʹ��.		

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
    //ʹ�����ʱ��
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
	
    //��ʼ��GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��PA11,PA12
	
	  //���Ÿ���ӳ������
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11����ΪCAN1
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12����ΪCAN1
	  
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=tsjw;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 
    
		//���ù�����
 	  CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
		
#if CAN1_RX0_INT_ENABLE
	
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}   
 
#if CAN1_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����			    
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

//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//		 ����,ʧ��;
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x12;	 // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=0x12;	 // ������չ��ʾ����29λ��
  TxMessage.IDE=0;		  // ʹ����չ��ʶ��
  TxMessage.RTR=0;		  // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=len;							 // ������֡��Ϣ
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)return 1;
  return 0;		

}
//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		 ����,���յ����ݳ���;
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����	
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










