#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"

// 电机基本参数
#define ENCODE_13X 13    // 编码器线数
#define JIANSUBI 20.0f   // 减速比
#define BEIPIN 4         // 倍频 (修复后完美4倍频)
#define SAMPLE_TIME 0.01 // 采样时间
#define CC (ENCODE_13X * JIANSUBI * BEIPIN * SAMPLE_TIME)

#define PI 3.1415926f
#define RR 24.0f // 车轮半径单位mm
#define COUNTS_PER_MM                                                          \
  ((ENCODE_13X * JIANSUBI * BEIPIN) / (PI * RR * 2.0f)) // 每毫米脉冲数

/*编码器端口读取宏定义*/
#define Read_Encoder_A                                                         \
  ((DL_GPIO_readPins(Encoder_PORT, Encoder_A_PIN) == Encoder_A_PIN)            \
       ? 0                                                                     \
       : 1) // 左轮 A相
#define Read_Encoder_B                                                         \
  ((DL_GPIO_readPins(Encoder_PORT, Encoder_B_PIN) == Encoder_B_PIN)            \
       ? 0                                                                     \
       : 1) // 左轮 B相
#define Read_Encoder_C                                                         \
  ((DL_GPIO_readPins(Encoder_PORT, Encoder_C_PIN) == Encoder_C_PIN)            \
       ? 0                                                                     \
       : 1) // 右轮 A相
#define Read_Encoder_D                                                         \
  ((DL_GPIO_readPins(Encoder_PORT, Encoder_D_PIN) == Encoder_D_PIN)            \
       ? 0                                                                     \
       : 1) // 右轮 B相

extern volatile float Motor1_Speed;
extern volatile float Motor2_Speed;
extern volatile float Measure_Distance;

extern volatile uint32_t raw_interrupt_count_m1;
extern volatile uint32_t raw_interrupt_count_m2;

void Motor1_Get_Speed(void);
void Motor2_Get_Speed(void);
// 测量所有电机速度
void MEASURE_MOTORS_SPEED(void);

#endif
