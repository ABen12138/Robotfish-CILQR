#ifndef __MOTOR_H
#define __MOTOR_H	 


#include "stm32f4xx.h"
#include <stdbool.h>

#define MOTOR_NUM               1    
#define MAXON_CAN_PORT      CAN1





typedef struct _MaxonMsg
{
    uint32_t COB_ID;
    uint8_t  CS;     // command specifier
    uint16_t Index;
    uint8_t  SubIndex;   //
    uint32_t Value;      //
} MaxonMsg;

extern MaxonMsg msg_CtrlWord_Halt;
extern MaxonMsg msg_CtrlWord_QuickStop;
extern MaxonMsg msg_CtrlWord_ShutDown;
extern MaxonMsg msg_CtrlWord_SwitchOn;
extern MaxonMsg msg_CtrlWord_Start;

extern MaxonMsg PD0_msg_get_cur;
extern MaxonMsg PD0_msg_get_vel;
extern MaxonMsg PD0_msg_get_tor;

extern MaxonMsg msg_PPM_enter;
extern MaxonMsg msg_PPM_ProVel;
extern MaxonMsg msg_PPM_MaxProVel;
extern MaxonMsg msg_PPM_TarPos;
extern MaxonMsg msg_PPM_CtrlAbs;
extern MaxonMsg msg_PPM_CtrlAbsStart;
extern MaxonMsg msg_PPM_CtrlRlt;
extern MaxonMsg msg_PPM_CtrlRltStart;
extern MaxonMsg msg_PPM_NewPos;


extern MaxonMsg msg_get_vol;
extern MaxonMsg msg_get_cur;
extern MaxonMsg msg_get_pos;
extern MaxonMsg msg_get_vel;
extern MaxonMsg msg_get_tor;
extern MaxonMsg msg_get_tmp;

#define CAN_COMMAND_RECV_ERROR      0x80
#define CAN_COMMAND_READ_1BYTE      0x4F
#define CAN_COMMAND_READ_2BYTE      0x4B
#define CAN_COMMAND_READ_3BYTE      0x47
#define CAN_COMMAND_READ_4BYTE      0x43



#define RT_EOK                          0               /**< There is no error */
#define RT_ERROR                        1               /**< A generic error happens */
#define RT_ETIMEOUT                     2               /**< Timed out */
#define RT_EFULL                        3               /**< The resource is full */
#define RT_EEMPTY                       4               /**< The resource is empty */
#define RT_ENOMEM                       5               /**< No memory */
#define RT_ENOSYS                       6               /**< No system */
#define RT_EBUSY                        7               /**< Busy */
#define RT_EIO                          8               /**< IO error */
#define RT_EINTR                        9               /**< Interrupted system call */
#define RT_EINVAL                       10              /**< Invalid argument */



void motor_init(void);
uint8_t Cvt_MaxonMsg_2_CanTxMsg(CanTxMsg* Dmsg, const MaxonMsg *Smsg);
void Cvt_CanRxMsg_2_MaxonMsg(MaxonMsg* Dmsg, const CanRxMsg *Smsg);

uint8_t SDO_send(uint8_t idx, const MaxonMsg* msg_send);
void SDO_Set_Pos(uint8_t idx, int32_t pos);
void SDO_Set_Vel(uint8_t idx, int32_t vel);
void SDO_Set_Home_Here(uint8_t idx);
void SDO_Auto_Home(uint8_t idx);
void PDO_HMM(uint8_t idx, int32_t vel);
void PDO_Set_Vel(uint8_t idx, int32_t vel);
void PDO_Set_Pos(uint8_t idx, int32_t pos);
void PDO_send(uint8_t idx, const CanTxMsg msg_send);

#endif