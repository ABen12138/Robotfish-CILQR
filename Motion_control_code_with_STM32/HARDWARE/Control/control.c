#include "control.h"
#include "led.h"
#include "usart.h"
#include "remote.h"

 
 u8 Mode = 1;         //定义模式 0  不支持连续按压  1 支持连续按压
 u8 t = 0;
// mode = 1 支持连续按
// mode = 0 不支持连续按
 
// 支持连续、非连续按压模式 扫描
u8 Keyvalue_mode(u8 keyvalue , u8 mode){
  
		static u8 key_up=1;          // 状态变量     
	
	  if(mode)key_up=1;           // 
	  if((key_up)&&(keyvalue)&&RmtCnt!=0){
			key_up = 0;
		 return keyvalue;
		}else if(keyvalue==0 && RmtCnt==0){
		 key_up = 1;
		}
	  return 0;
		
}

// 改变模式
void Change_Keymode(){

//    if(Mode == 1){
//		  Mode = 0;
//		  }
//		else{
//		  Mode=1;
//		}
	
	 Mode = !Mode;
	
}


	u8 deta_deta =1;          // 增量
	u8 deta = 10;

void Add_deta(){
	
  deta = deta+deta_deta;    // 增量增加

}

void Sub_deta(){
	
  deta = deta-deta_deta;    // 增量减少
	
}

  u16 pwmvalue1 = 0;

void Add_pwmvalue1(){

  pwmvalue1 = pwmvalue1+deta; //pwm值1 增加
  TIM_SetCompare3(TIM2,pwmvalue1);

}


void Sub_pwmvalue1(){

  pwmvalue1 = pwmvalue1-deta; //pwm值1 减少
	TIM_SetCompare3(TIM2,pwmvalue1);

}


  u16 pwmvalue2 = 0;

void Add_pwmvalue2(){

  pwmvalue2 = pwmvalue2+deta;  //pwm值2 增加
  TIM_SetCompare4(TIM2,pwmvalue2);

}


void Sub_pwmvalue2(){

  pwmvalue2 = pwmvalue2-deta; //pwm值2 减少
  TIM_SetCompare4(TIM2,pwmvalue2);
}

