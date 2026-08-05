#ifndef __KEY_H__
#define __KEY_H__
#include "main.h"
/*-------------------------------------------------------------------------------------------*/
/*-----------------------------------LED操作函数-------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
// #define LED0_ON()		DL_GPIO_clearPins(LED_LED0_PORT, LED_LED0_PIN)
// #define LED0_OFF()		DL_GPIO_setPins(LED_LED0_PORT, LED_LED0_PIN)
// #define LED0_TOGGLE()	DL_GPIO_togglePins(LED_LED0_PORT, LED_LED0_PIN)

#define LED1_ON() DL_GPIO_clearPins(LED_PORT, LED_LED1_PIN)
#define LED1_OFF() DL_GPIO_setPins(LED_PORT, LED_LED1_PIN)
#define LED1_TOGGLE() DL_GPIO_togglePins(LED_PORT, LED_LED1_PIN)

// #define LED2_ON()		DL_GPIO_clearPins(LED_LED2_PORT, LED_LED2_PIN)
// #define LED2_OFF()		DL_GPIO_setPins(LED_LED2_PORT, LED_LED2_PIN)
// #define LED2_TOGGLE()	DL_GPIO_togglePins(LED_LED2_PORT, LED_LED2_PIN)

#define LED3_OFF() DL_GPIO_clearPins(LED_PORT, LED_LED3_PIN)
#define LED3_ON() DL_GPIO_setPins(LED_PORT, LED_LED3_PIN)
#define LED3_TOGGLE() DL_GPIO_togglePins(LED_PORT, LED_LED3_PIN)



/*-------------------------------------------------------------------------------------------*/
/*--------------------------------------按键读取宏定义---------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
#define KEY_Number 3

#define KEY1                                                                   \
  (DL_GPIO_readPins(KEY_KEY1_PORT, KEY_KEY1_PIN) == KEY_KEY1_PIN) ? 1 : 0
#define KEY2                                                                   \
  (DL_GPIO_readPins(KEY_KEY2_PORT, KEY_KEY2_PIN) == KEY_KEY2_PIN) ? 1 : 0
#define KEY3                                                                   \
  (DL_GPIO_readPins(KEY_KEY3_PORT, KEY_KEY3_PIN) == KEY_KEY3_PIN) ? 1 : 0

typedef struct {
  bool Down_State;
  bool Short_Flag;
  bool Double_Flag;
  bool Long_Flag;
  bool Double_Time_EN;
  uint8_t Double_Time;
  uint8_t Down_Time;
  uint8_t Judge_State;
} KEY;

void Key_Read(void);
extern KEY Key[KEY_Number];
void KEY_PROC(void);

#endif
