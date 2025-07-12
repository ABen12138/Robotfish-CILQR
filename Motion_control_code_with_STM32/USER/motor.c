#include "motor.h"
#include "can.h"
#include "delay.h"
#include <stdbool.h>
#include "stm32f4xx.h"

uint16_t Motor_Status[MOTOR_NUM] = {0};
uint8_t  Motor_Mode[MOTOR_NUM] = {0};

//Maxon 格式    COB-ID   CS  INDEX   SUBINDEX  32位数据
MaxonMsg msg_PPM_enter=         {0x600, 0x22, 0x6060, 0x00, 0x00000001};
MaxonMsg msg_PPM_ProVel=        {0x600, 0x22, 0x6081, 0x00, 0x00000000};
MaxonMsg msg_PPM_MaxProVel=     {0x600, 0x22, 0x607F, 0x00, 0x00000000};
MaxonMsg msg_PPM_TarPos=        {0x600, 0x22, 0x607A, 0x00, 0x00000000};
MaxonMsg msg_PPM_CtrlAbs=       {0x600, 0x22, 0x6040, 0x00, 0x0000001F};
MaxonMsg msg_PPM_CtrlAbsStart=  {0x600, 0x22, 0x6040, 0x00, 0x0000003F};
MaxonMsg msg_PPM_CtrlRlt=       {0x600, 0x22, 0x6040, 0x00, 0x0000007F};
MaxonMsg msg_PPM_CtrlRltStart=  {0x600, 0x22, 0x6040, 0x00, 0x0000005F};
MaxonMsg msg_PPM_NewPos=        {0x600, 0x22, 0x6040, 0x00, 0x0000000F};




MaxonMsg msg_PVM_enter=         {0x600, 0x22, 0x6060, 0x00, 0x00000003};
MaxonMsg msg_PVM_tarvel=        {0x600, 0x22, 0x60FF, 0x00, 0x00000000};
MaxonMsg msg_PVM_NewVel=        {0x600, 0x22, 0x6040, 0x00, 0x0000000F};

MaxonMsg msg_HMM_enter=        {0x600, 0x22, 0x6060, 0x00, 0x00000006};
MaxonMsg msg_CST_enter=        {0x600, 0x22, 0x6060, 0x00, 0x0000000A};


#define MODE_PPM            1
#define MODE_PVM            3
#define MODE_HMM            6
#define MODE_CSP            8
#define MODE_CSV            9
#define MODE_CST            10

#define STATE_FLAG_MASK                 0x006F
#define STATE_NOT_READY_TO_SWITCH_ON    0x0000
#define STATE_SWITCH_ON_DISABLED        0x0020
#define STATE_READY_TO_SWITCH_ON        0x0011
#define STATE_SWITCHED_ON               0x0013
#define STATE_OPERATION_ENABLED         0x0027
#define STATE_QUICK_STOP_ACTIVE         0x0007
#define STATE_FAULT_REACTION_ACTIVE     0x000F
#define STATE_FAULT                     0x0008



//#define STATE_FLAG_MASK                 0x006F
//#define STATE_NOT_READY_TO_SWITCH_ON    0x0000
//#define STATE_SWITCH_ON_DISABLED        0x0040
//#define STATE_READY_TO_SWITCH_ON        0x0021
//#define STATE_SWITCHED_ON               0x0023
//#define STATE_OPERATION_ENABLED         0x0027
//#define STATE_QUICK_STOP_ACTIVE         0x0007
//#define STATE_FAULT_REACTION_ACTIVE     0x000F
//#define STATE_FAULT                     0x0008








#define CAN_SDO_DELAY   500

#define SDO_CAN_DELAY       1   // ??PDO??????, ??: ms
#define PDO_CAN_DELAY       1   // ??SDO??????, ??: ms
#define SDO_COBID_C2S       0x600   // COB-ID SDO client to server, 
#define SDO_COBID_S2C       0x580   // COB-ID SDO server to client
#define SDO_WAIT_MAX_MS     10      // max time for waiting a SDO response (Unit: ms)

//#define PDO1_COBID          0x180
//#define PDO2_COBID          0x280
//#define PDO3_COBID          0x380
//#define PDO4_COBID          0x480

#define PDO1_COBID          0x200
#define PDO2_COBID          0x300
#define PDO3_COBID          0x400
#define PDO4_COBID          0x500



//#define TPDO1_COBID          0x40000180
//#define TPDO2_COBID          0x40000280
//#define TPDO3_COBID          0x40000380
//#define TPDO4_COBID          0xC0000480


MaxonMsg msg_CtrlWord_Halt=         {0x600, 0x22, 0x6040, 0x00, 0x0000010F};
MaxonMsg msg_CtrlWord_QuickStop=    {0x600, 0x22, 0x6040, 0x00, 0x0000000B};
MaxonMsg msg_CtrlWord_ShutDown=     {0x600, 0x22, 0x6040, 0x00, 0x00000006};
MaxonMsg msg_CtrlWord_SwitchOn=     {0x600, 0x22, 0x6040, 0x00, 0x0000000F};
MaxonMsg msg_CtrlWord_Start=        {0x600, 0x22, 0x6040, 0x00, 0x0000001F};

                                                                                
CanTxMsg PDO_CtrlWord_Halt =        {PDO1_COBID, 1, CAN_Id_Standard, CAN_RTR_Data, 8, {0x0F, 0x01, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00}};
CanTxMsg PDO_CtrlWord_QucikStop =   {PDO1_COBID, 1, CAN_Id_Standard, CAN_RTR_Data, 8, {0x0B, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00}};
//CanTxMsg PDO_CtrlWord_ShutDown =    {PDO2_COBID, 1, CAN_Id_Standard, CAN_RTR_Data, 8, {0x06, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00}};
//CanTxMsg PDO_CtrlWord_SwitchOn =    {PDO2_COBID, 1, CAN_Id_Standard, CAN_RTR_Data, 8, {0x0F, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00}};
CanTxMsg PDO_CtrlWord_ShutDown =    {PDO1_COBID, 1, CAN_Id_Standard, CAN_RTR_Data, 8, {0x06, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00}};
CanTxMsg PDO_CtrlWord_SwitchOn =    {PDO1_COBID, 1, CAN_Id_Standard, CAN_RTR_Data, 8, {0x0F, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00}};
CanTxMsg PDO_CtrlWord_OprStart_rel_target =    {PDO1_COBID, 1, CAN_Id_Standard, CAN_RTR_Data, 8, {0x7F, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00}};
CanTxMsg PDO_CtrlWord_OprStart_Abl_target =    {PDO1_COBID, 1, CAN_Id_Standard, CAN_RTR_Data, 8, {0x3F, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00}};

CanTxMsg PDO_PPM_SetPos =   {PDO2_COBID, 1, CAN_Id_Standard, CAN_RTR_Data, 8, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
CanTxMsg PDO_PPM_NewPos =   {PDO3_COBID, 1, CAN_Id_Standard, CAN_RTR_Data, 8, {0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
CanTxMsg PDO_PVM_SetVel =   {PDO4_COBID, 1, CAN_Id_Standard, CAN_RTR_Data, 8, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

CanTxMsg PDO_CtrlWord_OprStart_HMM =    {PDO1_COBID, 1, CAN_Id_Standard, CAN_RTR_Data, 8, {0x1F, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00}};

/* Homing method, 寻零模式, 设置当前的位置为0位等 */                            
// -3 <--> 0xFD : Current Threshold Positive Speed
// -4 <--> 0xFC : Current Threshold negative Speed
// 37 <--> 0x25 : Actual Position
MaxonMsg msg_HMM_method=            {0x600, 0x22, 0x6098, 0x00, 0x000000FD};    // -3
MaxonMsg msg_HMM_offset=            {0x600, 0x22, 0x30B1, 0x00, 0x00001644};    //
MaxonMsg msg_HMM_current=           {0x600, 0x22, 0x30B2, 0x00, 0x000001F4};
MaxonMsg msg_HMM_position=          {0x600, 0x22, 0x30B0, 0x00, 0x00000000};




MaxonMsg msg_get_vol=   {0x600, 0x40, 0x2200, 0x01, 0x00000000};
MaxonMsg msg_get_cur=   {0x600, 0x40, 0x30D1, 0x02, 0x00000000};
MaxonMsg msg_get_pos=   {0x600, 0x40, 0x6064, 0x00, 0x00000000};
MaxonMsg msg_get_vel=   {0x600, 0x40, 0x606C, 0x00, 0x00000000};
MaxonMsg msg_get_tor=   {0x600, 0x40, 0x30D2, 0x01, 0x00000000};
MaxonMsg msg_get_tmp=   {0x600, 0x40, 0x3201, 0x01, 0x00000000};


//MaxonMsg PD0_msg_get_cur=   {TPDO1_COBID, 0x40, 0x30D1, 0x02, 0x00000000};
//MaxonMsg PD0_msg_get_vel=   {TPDO2_COBID, 0x40, 0x606C, 0x00, 0x00000000};
//MaxonMsg PD0_msg_get_tor=   {TPDO3_COBID, 0x40, 0x30D2, 0x01, 0x00000000};


void Cvt_CanRxMsg_2_MaxonMsg(MaxonMsg* Dmsg, const CanRxMsg *Smsg)
{
    if(Smsg->Data[0] == CAN_COMMAND_RECV_ERROR)
			;
        //return -RT_ERROR;   // ???????,????
    
    Dmsg->COB_ID = Smsg->StdId;
    Dmsg->CS = Smsg->Data[0];
    Dmsg->Index = *(signed short *)&Smsg->Data[1];
    Dmsg->SubIndex = Smsg->Data[3];
    Dmsg->Value = *(unsigned long *)&Smsg->Data[4];
    switch(Dmsg->CS)
    {
        case CAN_COMMAND_READ_1BYTE: Dmsg->Value = 0x000000FF & Dmsg->Value; break;
        case CAN_COMMAND_READ_2BYTE: Dmsg->Value = 0x0000FFFF & Dmsg->Value; break;
        case CAN_COMMAND_READ_3BYTE: Dmsg->Value = 0x00FFFFFF & Dmsg->Value; break;
        case CAN_COMMAND_READ_4BYTE: Dmsg->Value = 0xFFFFFFFF & Dmsg->Value; break;
        default: ;
    }
}




uint8_t Cvt_MaxonMsg_2_CanTxMsg(CanTxMsg* Dmsg, const MaxonMsg *Smsg)
{
    Dmsg->StdId = Smsg->COB_ID;
    Dmsg->ExtId = 1;
    Dmsg->IDE   = CAN_Id_Standard;
    Dmsg->RTR   = CAN_RTR_Data;
    Dmsg->DLC   = 8;
    Dmsg->Data[0] = Smsg->CS;
    *(uint16_t*)(&Dmsg->Data[1]) = Smsg->Index;
    Dmsg->Data[3] = Smsg->SubIndex;
    *(uint32_t*)(&Dmsg->Data[4]) = Smsg->Value;
	  return 0;
}


uint8_t SDO_send(uint8_t idx, const MaxonMsg* msg_send)
{
    CanTxMsg stm32_msg;
    if(Cvt_MaxonMsg_2_CanTxMsg(&stm32_msg, msg_send) != 0)
        return -1;
    stm32_msg.StdId = SDO_COBID_C2S + idx;             // The SDO COB-ID are determined by the configured Node-ID (DIP-Switch or so
    CAN_Transmit(MAXON_CAN_PORT, &stm32_msg);
		return 0;
}

void PDO_send(uint8_t idx, const CanTxMsg msg_send)
{
    CanTxMsg stm32_msg = msg_send;
    stm32_msg.StdId = msg_send.StdId + idx;             
    CAN_Transmit(MAXON_CAN_PORT, &stm32_msg);
}

void motor_init(void)
{
	int i=0;
	for(i = 0; i < MOTOR_NUM; i++)
  {
    Enter_Pre_Operational(i + 1);   
    delay_ms(1000);
    Start_Remote_Node(i + 1);      
    delay_ms(500);
   }
}

void SDO_Set_Pos(uint8_t idx, int32_t pos)
{
    if((Motor_Status[idx - 1] & STATE_FLAG_MASK) != STATE_OPERATION_ENABLED)
    {
        SDO_send(idx, &msg_CtrlWord_ShutDown);  delay_ms(1);
        SDO_send(idx, &msg_CtrlWord_SwitchOn);  delay_ms(1);
    }
    
    if(Motor_Mode[idx - 1] != MODE_PPM)
    {
        SDO_send(idx, &msg_PPM_enter);          delay_ms(CAN_SDO_DELAY);
    }
    msg_PPM_TarPos.Value = (uint32_t)pos;
    SDO_send(idx, &msg_PPM_TarPos);         delay_ms(1);
    SDO_send(idx, &msg_PPM_CtrlRltStart);   delay_ms(1);
    SDO_send(idx, &msg_PPM_NewPos);         delay_ms(1);
}


void SDO_Set_Vel(uint8_t idx, int32_t vel)
{
	 //if((Motor_Status[idx - 1] & STATE_FLAG_MASK) != STATE_OPERATION_ENABLED)
   // {
        SDO_send(idx, &msg_CtrlWord_ShutDown);   delay_ms(1);
        SDO_send(idx, &msg_CtrlWord_SwitchOn);   delay_ms(1);
 //   }
   /// if(Motor_Mode[idx - 1] != MODE_PVM)
   // {
        SDO_send(idx, &msg_PVM_enter);          delay_ms(CAN_SDO_DELAY);
		    Motor_Mode[idx - 1] = MODE_PVM;
 //   }
		
		msg_PVM_tarvel.Value = (uint32_t)vel;  
		SDO_send(idx,&msg_PVM_tarvel);       // delay_ms(1);
    SDO_send(idx, &msg_PVM_NewVel);     //  delay_ms(1);
}


void SDO_Set_Home_Here(uint8_t idx)
{    
    SDO_send(idx, &msg_CtrlWord_Halt); 
    delay_ms(CAN_SDO_DELAY);
    
    SDO_send(idx, &msg_HMM_enter); 
    delay_ms(CAN_SDO_DELAY);                // 延时为必须, 确保进入Home设置模式
    msg_HMM_position.Value = 0;          // Home position 0
    SDO_send(idx, &msg_HMM_position);
    delay_ms(CAN_SDO_DELAY);
    msg_HMM_offset.Value = 0;            // offset 0 inc
    SDO_send(idx, &msg_HMM_offset);
    delay_ms(CAN_SDO_DELAY);
    msg_HMM_method.Value = 0x00000025;   // 37, actual position
    SDO_send(idx, &msg_HMM_method);     
    delay_ms(CAN_SDO_DELAY);
    
    SDO_send(idx, &msg_CtrlWord_ShutDown);  delay_ms(CAN_SDO_DELAY);
    SDO_send(idx, &msg_CtrlWord_SwitchOn);  delay_ms(CAN_SDO_DELAY);
    SDO_send(idx, &msg_CtrlWord_Start);     delay_ms(CAN_SDO_DELAY);
}
void SDO_Auto_Home(uint8_t idx)
{
    SDO_send(idx, &msg_CtrlWord_Halt); 
    delay_ms(CAN_SDO_DELAY);
    
    SDO_send(idx, &msg_HMM_enter);
    delay_ms(CAN_SDO_DELAY);                   // 延时为必须, 确保进入Home设置模式
    msg_HMM_offset.Value = 5700;            // offset 5700 inc
    SDO_send(idx, &msg_HMM_offset);
    delay_ms(CAN_SDO_DELAY);
    msg_HMM_current.Value = 500;           // stall current 500mA
    SDO_send(idx, &msg_HMM_current);
    delay_ms(CAN_SDO_DELAY);
    msg_HMM_method.Value = 0x000000FD;      // -3, Current Threshold Positive Speed
    SDO_send(idx, &msg_HMM_method);
    delay_ms(CAN_SDO_DELAY);
    
    SDO_send(idx, &msg_CtrlWord_ShutDown);  delay_ms(CAN_SDO_DELAY);
    SDO_send(idx, &msg_CtrlWord_SwitchOn);  delay_ms(CAN_SDO_DELAY);
    SDO_send(idx, &msg_CtrlWord_Start);     delay_ms(CAN_SDO_DELAY);
}


void PDO_Set_Pos(uint8_t idx, int32_t pos)
{
	
    if(Motor_Mode[idx - 1] != MODE_PPM)
    {
        SDO_send(idx, &msg_PPM_enter);          delay_ms(CAN_SDO_DELAY);
		   	Motor_Mode[idx - 1] = MODE_PPM;
    }	
	
	
    if((Motor_Status[idx - 1] & STATE_FLAG_MASK) != STATE_OPERATION_ENABLED)
    {
        PDO_send(idx, PDO_CtrlWord_ShutDown);   delay_ms(CAN_SDO_DELAY);
        PDO_send(idx, PDO_CtrlWord_SwitchOn);   delay_ms(CAN_SDO_DELAY);
			  //Motor_Status[idx - 1]=STATE_OPERATION_ENABLED;
    }


    *(uint32_t*)&PDO_PPM_SetPos.Data[0] = (uint32_t)pos;           // set target pos
    PDO_send(idx, PDO_PPM_SetPos);       delay_ms(2);
		PDO_send(idx, PDO_CtrlWord_OprStart_rel_target);       delay_ms(2);
		PDO_send(idx, PDO_PPM_NewPos);       delay_ms(2);
}

void PDO_Set_Vel(uint8_t idx, int32_t vel)
	
{
	    if((Motor_Status[idx - 1] & STATE_FLAG_MASK) != STATE_OPERATION_ENABLED)
    {
        PDO_send(idx, PDO_CtrlWord_ShutDown);   delay_ms(CAN_SDO_DELAY);
        PDO_send(idx, PDO_CtrlWord_SwitchOn);   delay_ms(CAN_SDO_DELAY);
			  Motor_Status[idx - 1]=STATE_OPERATION_ENABLED;
    }
    if(Motor_Mode[idx - 1] != MODE_PVM)
    {
        SDO_send(idx, &msg_PVM_enter);          delay_ms(CAN_SDO_DELAY);
		    Motor_Mode[idx - 1] = MODE_PVM;
    }
		
		
    *(uint32_t*)&PDO_PVM_SetVel.Data[0] = (uint32_t)vel;         
    PDO_send(idx, PDO_PVM_SetVel);       delay_ms(2);
		PDO_send(idx, PDO_CtrlWord_OprStart_rel_target);       delay_ms(2);
}



void PDO_HMM(uint8_t idx, int32_t vel)
{
	    if((Motor_Status[idx - 1] & STATE_FLAG_MASK) != STATE_OPERATION_ENABLED)
    {
        PDO_send(idx, PDO_CtrlWord_ShutDown);   delay_ms(CAN_SDO_DELAY);
        PDO_send(idx, PDO_CtrlWord_SwitchOn);   delay_ms(CAN_SDO_DELAY);
			  Motor_Status[idx - 1]=STATE_OPERATION_ENABLED;
    }
    if(Motor_Mode[idx - 1] != MODE_PVM)
    {
        SDO_send(idx, &msg_PVM_enter);          delay_ms(CAN_SDO_DELAY);
		   	Motor_Mode[idx - 1] = MODE_PVM;
    }
    *(uint32_t*)&PDO_PVM_SetVel.Data[0] = (uint32_t)vel;           // set target pos
    PDO_send(idx, PDO_PVM_SetVel);       delay_ms(2);
		PDO_send(idx, PDO_CtrlWord_OprStart_rel_target);       delay_ms(2);
}