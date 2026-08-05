/**
 * @file main.h
 * @brief 核心中枢抽象层：统筹外设头文件、状态机宏定义及全局控制变量
 * @author WILLiam
 * @date 2026-06-12
 */
#ifndef __MAIN_H__
#define __MAIN_H__

/* ==================== 基础数据类型与核心库 ==================== */
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ==================== 芯片与板级抽象层 (BSP) ==================== */
#include "delay.h"            // 高精度阻塞延时支持
#include "ti_msp_dl_config.h" // 芯片底层资源配置 (Sysconfig)

/* ==================== 业务驱动与中间件 ==================== */
#include "process_frame.h" // 协议帧解析与上位机通信
#include "smd.h"           // 步进电机闭环运动控制核心驱动

/* ==================== 遗留数据类型重定义 (兼容旧版) ==================== */
typedef int32_t s32;
typedef int16_t s16;
typedef int8_t s8;
typedef const int32_t sc32;
typedef const int16_t sc16;
typedef const int8_t sc8;
typedef __IO int32_t vs32;
typedef __IO int16_t vs16;
typedef __IO int8_t vs8;
typedef __I int32_t vsc32;
typedef __I int16_t vsc16;
typedef __I int8_t vsc8;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;
typedef const uint32_t uc32;
typedef const uint16_t uc16;
typedef const uint8_t uc8;
typedef __IO uint32_t vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t vu8;
typedef __I uint32_t vuc32;
typedef __I uint16_t vuc16;
typedef __I uint8_t vuc8;
typedef float fp32;
typedef double fp64;

/* ==================== 小车状态机枚举宏 ==================== */
// 定义主控系统的有限状态机(FSM)运转阶段
#define Run_Mode 0      // 综合运行模式
#define Speed_Mode 1    // 开环/闭环速度标定测试
#define Turn_Mode 2     // 航向角原地旋转测试
#define Distance_Mode 3 // 前向定长位移测试
#define Gyro_Mode 4     // 陀螺仪零偏及角度闭环测试
#define Angle_Mode 5    // 圆弧/曲线轨迹拟合模式
#define Stand_Mode 6    // 待机/急停安全状态
#define AutoTune_Mode 7 // Ziegler-Nichols 底层继电自动整定模式
#define Ball_Task2_Mode 8 // 第三问：小球定点位置控制模式

// 像素与实际物理距离（厘米）的比例换算
// 新标定标准：25cm 对应 790 个显示像素(800x480屏幕)。
// M0 收到的是 K230 原始坐标(rgb888p 640x360)，故需换算到原始坐标：
// 1cm = (790/25) * 640/800 = 25.28 原始像素
#define PIXEL_PER_CM  (25.28f) // 宏定义：1cm 对应的原始像素值

/* ==================== 核心状态机及控制变量声明 ==================== */
extern uint8_t Car_Mode;         // 当前系统状态机阶段
extern uint8_t OLED_View_Select; // OLED 屏幕 UI 交互页面指针

// 运动学闭环目标期望值 (Setpoints)
extern float Basic_Speed;     // 前向基础牵引速度 (mm/s)
extern float Target_Distance; // 目标位移量 (mm)
extern float Target_Gyro;     // 目标自旋角速度 (度/s)
extern float Target_Angle;    // 目标航向角 (度)

/* ==================== 全局生命周期任务接口 ==================== */
void NVIC_EnableIRQ_Init(void); // 硬件中断嵌套及总线初始化
void Protocol_Datas_Proc(void); // 上位机协议非阻塞轮询解析任务
void OLED_Show_Proc(void);      // 人机交互 UI 异步刷新任务

#endif
