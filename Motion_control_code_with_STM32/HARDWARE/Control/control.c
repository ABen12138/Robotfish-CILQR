#include "control.h"
#include "led.h"
#include "usart.h"
#include "remote.h"

 
 u8 Mode = 1;         //����ģʽ 0  ��֧��������ѹ  1 ֧��������ѹ
 u8 t = 0;
// mode = 1 ֧��������
// mode = 0 ��֧��������
 
// ֧����������������ѹģʽ ɨ��
u8 Keyvalue_mode(u8 keyvalue , u8 mode){
  
		static u8 key_up=1;          // ״̬����     
	
	  if(mode)key_up=1;           // 
	  if((key_up)&&(keyvalue)&&RmtCnt!=0){
			key_up = 0;
		 return keyvalue;
		}else if(keyvalue==0 && RmtCnt==0){
		 key_up = 1;
		}
	  return 0;
		
}

// �ı�ģʽ
void Change_Keymode(){

//    if(Mode == 1){
//		  Mode = 0;
//		  }
//		else{
//		  Mode=1;
//		}
	
	 Mode = !Mode;
	
}


	u8 deta_deta =1;          // ����
	u8 deta = 10;

void Add_deta(){
	
  deta = deta+deta_deta;    // ��������

}

void Sub_deta(){
	
  deta = deta-deta_deta;    // ��������
	
}

  u16 pwmvalue1 = 0;

void Add_pwmvalue1(){

  pwmvalue1 = pwmvalue1+deta; //pwmֵ1 ����
  TIM_SetCompare3(TIM2,pwmvalue1);

}


void Sub_pwmvalue1(){

  pwmvalue1 = pwmvalue1-deta; //pwmֵ1 ����
	TIM_SetCompare3(TIM2,pwmvalue1);

}


  u16 pwmvalue2 = 0;

void Add_pwmvalue2(){

  pwmvalue2 = pwmvalue2+deta;  //pwmֵ2 ����
  TIM_SetCompare4(TIM2,pwmvalue2);

}


void Sub_pwmvalue2(){

  pwmvalue2 = pwmvalue2-deta; //pwmֵ2 ����
  TIM_SetCompare4(TIM2,pwmvalue2);
}

