#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "remote.h"
#include "pwm.h"
#include "control.h"
#include "can.h"
#include "motor.h"

//����ͷ�ļ�
#include "hc05.h" 	 
#include "usart3.h" 

#include "key.h"
#include "string.h"	 


#define cnt_fre 1000

int Can_Receive_Flag=0;
u32 vel_fre=260;
float fre=0.0;
float fre_last=0.0;
float Battery_Voltage=0;
u32 Motor_Current=0;
u32 Motor_Velocity=0; 
float Motor_Torque=0;



 ////////////////////////////////////////////////////////////////////////////////
 // ������ƣ���ʱ����ͨ��TIM2_CH3 TIM2_CH4
 // ��Ӧ�����ţ� PA2 PA3
 
 ////////////////////////////////////////////////////////////////////////////////
 
 
 //��ʾATK-HC05ģ�������״̬
void HC05_Role_Show(void)
{
	if(HC05_Get_Role()==1)LCD_ShowString(30,400,200,16,16,"ROLE:Master");	//����
	else LCD_ShowString(30,400,200,16,16,"ROLE:Slave ");			 		//�ӻ�
}
//��ʾATK-HC05ģ�������״̬
void HC05_Sta_Show(void)
{												 
	if(HC05_LED)LCD_ShowString(180,400,120,16,16,"STA:Connected ");			//���ӳɹ�
	else LCD_ShowString(180,400,120,16,16,"STA:Disconnect");	 			//δ����				 
}	 
 
 
int main(void)
{ 
	
	// ��������
	u8 t_lanya;
	u8 key_lanya;
	u8 sendmask=1;
	u8 sendcnt=0;
	u8 sendbuf[20];	  
	u8 reclen=0; 
	u16 key=5;
	u8 a=0;
	u8 i=0,t=0;
	u32 cnt=0;
	u8 canbuf[8];
	float set_fre=50;
	u16 LA=0;
	u32 LA_PWM=0;
	u16 RA=0;
	u32 RA_PWM=0;
	u8 res;
	u8 mode=0;//CAN����ģʽ;0,��ͨģʽ;1,����ģʽ
	u32 rel_fre=(u32)((100000/set_fre)-1);
		int Send_Message_Frequently_Flag=0;
	//CanTxMsg M_msg;
	//Cvt_MaxonMsg_2_CanTxMsg(&M_msg,&PD0_msg_get_vel);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);   //����ϵͳ�ж����ȼ�����2
	delay_init(168);        //��ʼ����ʱ����
	TIM4_PWM_Init(rel_fre,840-1);	//84M/84=1Mhz�ļ���Ƶ��,��װ��ֵ500������PWMƵ��Ϊ 1M/500=2Khz.  arr:5000-1 
	usart3_init(115200);	
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal);//CAN��ʼ������ģʽ,������500Kbps 
	USART3_RX_STA=0;
	//delay_ms(3000);
	
// 	for (a=0;a<=5;a++)
//	{
		motor_init();	
		delay_ms(500);
		SDO_Set_Vel(1,fre*vel_fre);	
	//}			
	//delay_ms(1000);
	Can_Receive_Flag=1;
		//delay_ms(10000);
//	for (a=0;a<=100;a++)
//	{
//		SDO_send(1,&msg_get_tor);
//		delay_ms(100);
//	}
	USART3_RX_STA=0;
	TIM_SetCompare2(TIM4,1750); //������
	TIM_SetCompare1(TIM4,1750);	//������

	while(1)
	{
		//
		//cnt++;
		//if(cnt<=1000000000)
		//{
			//SDO_send(1,&msg_get_tor);
		//	delay_ms(1000);
		//}
		//u3_printf("pdo");		
//		if(cnt<=1000)
//		{
//	   SDO_send(1,&msg_get_vol);
//			//delay_ms(100);
//		}
//		else
//			cnt=5000;
		if(USART3_RX_STA&0X8000)			//���յ�һ��������
		{
 			reclen=USART3_RX_STA&0X7FFF;	//�õ����ݳ���
		  USART3_RX_BUF[reclen]=0;	 	//���������
 				if((((char)USART3_RX_BUF[0])=='H')&&(((char )USART3_RX_BUF[1])=='z'))
			{
				if(((char)USART3_RX_BUF[4])=='.') //Hz10.0
				{
					fre=(USART3_RX_BUF[2]-48)*10+USART3_RX_BUF[3]-48+(float)(0.1*(USART3_RX_BUF[5]-48));
				}
				else if(((char)USART3_RX_BUF[3])=='.') //Hz8.0
				{
					fre=USART3_RX_BUF[2]-48+(float)(0.1*(USART3_RX_BUF[4]-48));
				}
				SDO_Set_Vel(1,fre*vel_fre);	
			}
			else if	((((char)USART3_RX_BUF[0])=='G')&&(((char )USART3_RX_BUF[1])=='V')&&(((char )USART3_RX_BUF[2])=='o'))	 //GVo ��ȡ��ѹ
			{
				SDO_send(1,&msg_get_vol);
			}	
			else if	((((char)USART3_RX_BUF[0])=='G')&&(((char )USART3_RX_BUF[1])=='V')&&(((char )USART3_RX_BUF[2])=='e'))	//GVe ��ȡ�ٶ�
			{
				SDO_send(1,&msg_get_vel);
			}	
			else if	((((char)USART3_RX_BUF[0])=='G')&&(((char )USART3_RX_BUF[1])=='C'))	//GC ��ȡ����
			{
				SDO_send(1,&msg_get_cur);
			}	
			else if	((((char)USART3_RX_BUF[0])=='G')&&(((char )USART3_RX_BUF[1])=='T'))	//GT ��ȡŤ��
			{
				SDO_send(1,&msg_get_tor);
			}	
			else if((((char)USART3_RX_BUF[0])=='L')&&(((char )USART3_RX_BUF[1])=='A'))	//LA ����������
			{
				LA=(USART3_RX_BUF[2]-48)*100+(USART3_RX_BUF[3]-48)*10+(USART3_RX_BUF[4]-48);
				LA_PWM=LA*1.11+1750;
				if(LA_PWM<=1750) LA_PWM=1750;
				else if(LA_PWM>=1950) LA_PWM=1950;
				else ;					
				TIM_SetCompare2(TIM4,LA_PWM); //������
			}	
			else if((((char)USART3_RX_BUF[0])=='R')&&(((char )USART3_RX_BUF[1])=='A'))	//RA ����������
			{
				RA=(USART3_RX_BUF[2]-48)*100+(USART3_RX_BUF[3]-48)*10+(USART3_RX_BUF[4]-48);
				RA_PWM=RA*1.11+1750;
				if(RA_PWM<=1750) RA_PWM=1750;
				else if(RA_PWM>=1950) RA_PWM=1950;
				else ;					
				TIM_SetCompare1(TIM4,RA_PWM); //������
			}
			else if((((char)USART3_RX_BUF[0])=='G')&&(((char )USART3_RX_BUF[1])=='M')&&(((char )USART3_RX_BUF[2])=='F'))
			{
				Send_Message_Frequently_Flag=1;
			}
			else if((((char)USART3_RX_BUF[0])=='D')&&(((char )USART3_RX_BUF[1])=='G')&&(((char )USART3_RX_BUF[2])=='M'))
			{
				Send_Message_Frequently_Flag=0;
			}
			else ;			
 			USART3_RX_STA=0;	
			//u3_printf("fre: %f\r\n",fre);			
		}	 
			if (Send_Message_Frequently_Flag==1)
			{
				SDO_send(1,&msg_get_vel);
				delay_ms(200);
				//Send_Message_Frequently_Flag=0;
				//delay_ms(200);
				SDO_send(1,&msg_get_cur);
				delay_ms(200);
				SDO_send(1,&msg_get_tor);		
				delay_ms(200);
			}		

	}
}
