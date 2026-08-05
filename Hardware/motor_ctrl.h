/**
 * @file motor_ctrl.h
 * @brief 电机驱动与运动学闭环控制抽象层 (Motor Control & Kinematic Closed-Loop
 * BSP)
 * @note 该模块负责底层硬件 PWM 驱动、运动学解算以及所有相关的 PID 状态机控制。
 */

#ifndef __MOTOR_CTRL_H__
#define __MOTOR_CTRL_H__

#include "main.h"

// ==================== 运动学物理约束宏 ====================
#define MAX_ALLOWED_SPEED_MM_S 1850.0f // 平台最大允许物理速度限制 (mm/s)

// ==================== PID 控制器抽象结构体 ====================
/**
 * @brief 标准位置式 PID 结构体定义 (Standard Positional PID Controller)
 */
typedef struct pid_t {
  float Kp; // 比例增益 (Proportional Gain)
  float Ki; // 积分增益 (Integral Gain)
  float Kd; // 微分增益 (Derivative Gain)

  float Target;  // 目标期望值 (Setpoint)
  float Measure; // 实际测量值 (Feedback Measurement)
  float Error;   // 当前偏差 (Current Error)

  float KpOut;   // 比例项独立输出 (Proportional Output)
  float KiOut;   // 积分项独立输出 (Integral Output)
  float KdOut;   // 微分项独立输出 (Derivative Output)
  float PID_Out; // 控制器总体输出 (Total Output)

  uint32_t PID_Limit_MAX; // 总体输出限幅 (Absolute Maximum Output)
  uint32_t Ki_Limit_MAX;  // 积分限幅 (Absolute Maximum Integral Output)
} pid_t;

// ==================== 硬件引脚控制宏 (Hardware GPIO Macros)
// ====================
#define AIN1(x)                                                                \
  ((x) ? DL_GPIO_setPins(Motor_Ctrl_AIN1_PORT, Motor_Ctrl_AIN1_PIN)            \
       : DL_GPIO_clearPins(Motor_Ctrl_AIN1_PORT, Motor_Ctrl_AIN1_PIN))
#define AIN2(x)                                                                \
  ((x) ? DL_GPIO_setPins(Motor_Ctrl_AIN2_PORT, Motor_Ctrl_AIN2_PIN)            \
       : DL_GPIO_clearPins(Motor_Ctrl_AIN2_PORT, Motor_Ctrl_AIN2_PIN))
#define BIN1(x)                                                                \
  ((x) ? DL_GPIO_setPins(Motor_Ctrl_BIN1_PORT, Motor_Ctrl_BIN1_PIN)            \
       : DL_GPIO_clearPins(Motor_Ctrl_BIN1_PORT, Motor_Ctrl_BIN1_PIN))
#define BIN2(x)                                                                \
  ((x) ? DL_GPIO_setPins(Motor_Ctrl_BIN2_PORT, Motor_Ctrl_BIN2_PIN)            \
       : DL_GPIO_clearPins(Motor_Ctrl_BIN2_PORT, Motor_Ctrl_BIN2_PIN))

#define Motor1_Forward()                                                       \
  {                                                                            \
    BIN1(0);                                                                   \
    BIN2(1);                                                                   \
  }
#define Motor1_Backward()                                                      \
  {                                                                            \
    BIN1(1);                                                                   \
    BIN2(0);                                                                   \
  }
#define Motor1_Stop()                                                          \
  {                                                                            \
    BIN1(0);                                                                   \
    BIN2(0);                                                                   \
  }

#define Motor2_Forward()                                                       \
  {                                                                            \
    AIN1(0);                                                                   \
    AIN2(1);                                                                   \
  }
#define Motor2_Backward()                                                      \
  {                                                                            \
    AIN1(1);                                                                   \
    AIN2(0);                                                                   \
  }
#define Motor2_Stop()                                                          \
  {                                                                            \
    AIN1(0);                                                                   \
    AIN2(0);                                                                   \
  }

#define MOTORS_ENABLE() Motors_Enable()
#define MOTORS_DISABLE() Motors_Disable()

// ==================== 接口函数声明 ====================

// PID 控制器通用接口
float PID_Calculate(pid_t *pid, float Measure, float Target);
void Set_PID_Param(pid_t *pid, float P, float I, float D);
void PID_Reset(pid_t *pid);

// 底层电机基础操作接口
void Motors_Enable(void);
void Motors_Disable(void);
void Motor1_Enable(void);
void Motor2_Enable(void);
void Motor1_Disable(void);
void Motor2_Disable(void);

// 速度设定接口
void Set_Motor1_Speed(int Target_Speed);
void Set_Motor2_Speed(int Target_Speed);
void SET_MOTORS_SPEED(int Target_Motor1_Speed, int Target_Motor2_Speed);

// 扩展驱动接口
void Car_Set_Speed_Unified(float speed_L, float speed_R);

// 自动整定 (Auto-Tune) 接口
void AutoTune_Init(void);
void AutoTune_Process(float *pwm1, float *pwm2);

// ==================== 全局 PID 实例与状态变量暴露 ====================
extern pid_t pid_Motor1_Speed;
extern pid_t pid_Motor2_Speed;
extern pid_t pid_Stand_Angle;
extern pid_t pid_Turn;
extern pid_t pid_Distance;
extern pid_t pid_Gyro;
extern pid_t pid_Angle;
extern pid_t pid_Ball_Beam;

extern float Gyro_Feedforward_Coefficient;
extern float Target_Gyro;

extern uint8_t Turn_PID_Flag;
extern uint8_t Distance_PID_Flag;
extern uint8_t Gyro_PID_Flag;
extern uint8_t Angle_PID_Flag;
extern uint8_t Stand_PID_Flag;

#endif // __MOTOR_CTRL_H__
