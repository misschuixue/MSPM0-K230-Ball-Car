#ifndef __BSP_HC05_H__
#define __BSP_HC05_H__

#include "ti_msp_dl_config.h"
#include <string.h>

void Bluetooth_Init(void);
void Receive_Bluetooth_Data(void);
void BLE_send_String(unsigned char *str);

#endif
