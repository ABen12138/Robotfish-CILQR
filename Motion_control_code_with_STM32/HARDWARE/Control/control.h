#ifndef _CONTROL_H
#define _CONTROL_H
#include "sys.h"

u8 Keyvalue_mode(u8 keyvalue , u8 mode);

void Add_deta(void);
void Sub_deta(void);

void Add_pwmvalue1(void);
void Sub_pwmvalue1(void);
void Add_pwmvalue2(void);
void Sub_pwmvalue2(void);

void Change_Keymode(void);
extern u8 Mode;

extern u8 deta_deta;        
extern u8 deta;   

extern u16 pwmvalue1;
extern u16 pwmvalue2;
#endif


