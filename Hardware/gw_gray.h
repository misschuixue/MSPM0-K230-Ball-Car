#ifndef __GW_GRAY_H__
#define __GW_GRAY_H__

#include "main.h"

uint16_t Huidu_Read(void);
float Huidu_Proc(uint16_t huidu_data);

extern float huidu_lasterror;

extern uint16_t Huidu_Datas;
extern float Huidu_Error;
extern int Huidu_Sum;

#endif
