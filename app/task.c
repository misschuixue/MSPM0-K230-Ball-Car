/**
 * @file task.c
 * @brief 底层驱动与业务逻辑模块
 * @details 该文件由 TeamSpark 于 2026 NUEDC 备赛期间重构，符合统一编码规范。
 * @author TeamSpark
 * @date 2026-07-10
 */
#include "task.h"
#include "Encoder.h"
#include "bsp_gyro.h"
#include "delay.h"
#include "gw_gray.h"
#include "key.h"
#include "motor_ctrl.h"
#include "process_frame.h"
#include "smd.h"
#include "timer.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>

// ================= 外部变量声明 =================
extern float Basic_Speed;
extern volatile float Measure_Distance;
extern volatile float Motor1_Lucheng;
extern volatile float Motor2_Lucheng;
extern uint8_t Car_Mode;
extern volatile bool g_rx_frame_flag;
extern volatile uint8_t g_rx_len;
extern uint8_t g_rx_cmd[128];
extern volatile bool g_rx2_frame_flag;
extern volatile uint8_t g_rx2_len;
extern uint8_t g_rx2_cmd[128];
extern Gyro_Struct *JY61P_Data;
extern volatile uint32_t ms_ticks;
// ==============================================

// =================================================================
// task.c 状态变量
// =================================================================
uint8_t Task_Mode = 0; // 0:待机, 1:跑任务一, 2:跑任务二
uint8_t Task1_Time_flag = 0;
uint8_t Task2_Time_flag = 0;
uint8_t Task_Select = 3; // 默认选择任务
uint8_t Test_Speed_Mode = 0;
extern uint8_t Tracking_Test_Flag;
float Target_Speed_Test = 300.0f;
float Debug_Yaw_Diff = 0.0f; // 共用变量，用于推送到 LCD 显示差值
float Task3_Cruise_Speed = 260.0f;  // 巡航速度
float Task3_Accel_Coeff = 3.0f;     // 加速时间(秒): 从起步速度加速到巡航速度的时间
float Task3_Decel_Coeff = 2.5f;     // 减速时间(秒): 从巡航速度减速到0的时间
float Task3_Start_Speed = 30.0f;    // 起步速度

// 任务四参数 (与任务三一致, 多一个角度触发刹车阈值)
float Task4_Cruise_Speed = 250.0f;  // 巡航速度
float Task4_Accel_Coeff = 4.0f;     // 加速时间(秒)
float Task4_Decel_Coeff = 5.0f;     // 减速时间(秒)
float Task4_Start_Speed = 30.0f;    // 起步速度
float Task4_Brake_Angle = 260.0f;   // 角度触发刹车阈值 (347-90, 提前90度开始停车)

// timer.c 中定义的 S 曲线斜坡控制变量
extern volatile uint8_t Task3_Ramp_Active;
extern volatile uint32_t Task3_Ramp_Start;
extern volatile uint8_t Task4_Ramp_Active;
extern volatile uint8_t Task4_Brake_Triggered;
extern volatile uint32_t Task4_Ramp_Start;
extern volatile uint32_t Task4_Brake_Start;
// =================================================================

void Task_Scheduler(void) {
  /* =========================================================================================================
   * [CRITICAL ARCHITECTURE WARNING] Task Scheduler Bypass (Exclusion List)
   * ---------------------------------------------------------------------------------------------------------
   * 此条件语句为底层 PID 闭环模式的旁路豁免名单 (Bypass List)。
   * 当在 timer.c 中新增任何独立接管电机控制权的底层闭环模式（如 Gyro_Mode,
   * Angle_Mode 等）时， 必须同步将该模式的枚举值添加至此判断条件中。
   *
   * [Failure Mode]
   * 若未在豁免名单中注册新模式，任务调度器将因无法匹配当前状态，默认穿透执行至下方的
   * switch(Task_Mode) 的 case 0。 此时，系统将在主循环中以极高频率反复触发
   * Motors_Disable() 和 Basic_Speed = 0。
   * 最终导致上位机发送的任何运动指令（速度/角速度目标值）被瞬间强行覆盖清零，表现为电机持续失能、系统无响应。
   * =========================================================================================================
   */
  if (Car_Mode == 1 || Car_Mode == 3 || Car_Mode == Stand_Mode ||
      Car_Mode == Angle_Mode || Car_Mode == Gyro_Mode) {
    return;
  }

  switch (Task_Mode) {
  case 0:
    // 待机状态：彻底禁止底盘动力，防止误动作
    Motors_Disable();
    Basic_Speed = 0;
    break;

  case 1:
    Task_1(); // 调用任务一状态机
    break;

  case 2:
    Task_2(); // 调用任务二状态机
    break;

  case 3:
    Task_3(); // 调用任务三状态机
    break;

  case 4:
    Task_4(); // 调用任务四状态机
    break;

  default:
    break;
  }
}

//======================PD电机控制任务======================
/**
 * @brief    处理应答
 * @param    over_time : 超时时间（单位ms）
 * @retval   0：处理成功，1：等待应答超时
 */
uint8_t handle_ack(uint8_t motor_id, uint32_t over_time) {
  SERIAL_FRAME g_serial_frame;

  if (motor_id == 1) {
    while (g_rx_frame_flag == false) {
      delay_ms(1);
      if (--over_time == 0) {
        g_rx_len = 0;
        g_rx_frame_flag = false;
        return 1;
      }
    }
    serial_frame_process((uint8_t *)g_rx_cmd, g_rx_len, &g_serial_frame);
    g_rx_frame_flag = false;
    g_rx_len = 0;
  } else if (motor_id == 2) {
    while (g_rx2_frame_flag == false) {
      delay_ms(1);
      if (--over_time == 0) {
        g_rx2_len = 0;
        g_rx2_frame_flag = false;
        return 1;
      }
    }
    serial_frame_process((uint8_t *)g_rx2_cmd, g_rx2_len, &g_serial_frame);
    g_rx2_frame_flag = false;
    g_rx2_len = 0;
  }
  return 0;
}

// 清除对应电机的接收缓存
void clear_uart_rx_buffer(uint8_t motor_id) {
  if (motor_id == 1) {
    g_rx_frame_flag = false;
    g_rx_len = 0;
  } else if (motor_id == 2) {
    g_rx2_frame_flag = false;
    g_rx2_len = 0;
  }
}

//===================================================================

/* ================================================================== */
/* ============================ 任务一 ============================== */
/* ================================================================== */

uint8_t Task_1_State = 0;
uint8_t Lap_Count = 0;
float Start_Yaw = 0.0f;
static uint8_t Super_Lock = 0;

void Task_1(void) {
  static uint32_t task1_led3_timeout = 0;

  // ========================================================
  // 0. 异步 LED 检查层
  // ========================================================
  if (task1_led3_timeout > 0 && ms_ticks >= task1_led3_timeout) {
    LED3_OFF();
    task1_led3_timeout = 0;
  }

  // ========================================================
  // 1. 全天候计算层
  // ========================================================
  if (JY61P_Data != NULL) {
    float current_z = JY61P_Data->total_z;
    float diff = current_z - Start_Yaw;

    if (diff < 0.0f)
      diff = -diff;

    Debug_Yaw_Diff = diff;
  }

  // 综合里程与角度的双重保险停车逻辑
  if (Task_1_State == 2) {
    uint8_t real_laps = 0;

    // 【角度精准停车
    // (最终完美版)】：彻底弃用路程作为主要依据(因轮胎打滑会导致路程失真)。
    // 里程仅保留 >3000 起步防呆。实测终点线真实角度为 350 度，扣除 3
    // 度滑行量，故 347 度触发刹车完美压线！
    // 增加硬件安全兜底：若陀螺仪意外断线(NULL)，则退化为实测的 5047 里程停车
    if ((JY61P_Data != NULL && Measure_Distance >= 3000.0f &&
         Debug_Yaw_Diff >= 347.0f) ||
        (JY61P_Data == NULL && Measure_Distance >= 5047.0f)) {
      real_laps = 1;
    } else {
      real_laps = 0;
    }

    if (real_laps > Lap_Count) {
      Lap_Count = real_laps;
      // 仅启动 100ms 异步 LED 提示圈数，不阻塞主循环
      LED3_ON();
      task1_led3_timeout = ms_ticks + 100;

      if (Lap_Count >= 1) {
        Task_1_State = 3;
      }
    }
  }

  // ========================================================
  // 2. 状态机控制层
  // ========================================================
  switch (Task_1_State) {
  case 0: // 初始待机
    Motors_Disable();
    Basic_Speed = 0;
    Turn_PID_Flag = 0;
    Super_Lock = 0; // 允许重复触发
    break;

  case 1: // 启动初始化
    if (Super_Lock == 0) {
      if (JY61P_Data != NULL)
        Start_Yaw = JY61P_Data->total_z;
      else
        Start_Yaw = 0.0f;

      Lap_Count = 0;
      Super_Lock = 1;

      // 任务启动时重置累计路程
      Measure_Distance = 0;
      Motor1_Lucheng = 0;
      Motor2_Lucheng = 0;
    }

    Car_Mode = 0; // [BUG FIX] 确保脱离其他任务模式时关闭小球平衡
    {
      extern int16_t Gimbal_Speed_X;
      extern int16_t Gimbal_Speed_Y;
      Gimbal_Speed_X = 0;
      Gimbal_Speed_Y = 0;
    }
    Motors_Enable();

    // 任务启动时，仅强制清空 PID 历史积分和历史微分状态，防止起步乱冲
    // 不再强制覆写 PID 参数，以保护你在屏幕/上位机上动态调节的值
    PID_Reset(&pid_Turn);

    Turn_PID_Flag = 1; // 清理干净后，再开启计算
    Distance_PID_Flag = 0;
    Gyro_PID_Flag = 0;
    Angle_PID_Flag = 0;

    Basic_Speed = 350.0f;  // 设置初始测试速度并重置计时器
    Task1_Time_Sec = 0.0f; // 重置计时
    Task_1_State = 2;      // 进入行驶
    break;

  case 2: // 行驶状态（维持）
    Task1_Time_flag = 1;
    // 接近终点平滑减速：完全改用陀螺仪角度发出减速信号 (在最后 80
    // 角度处开始平滑降至 150)
    if (JY61P_Data != NULL) {
      if (Debug_Yaw_Diff > 270.0f) {
        float remain_yaw = 350.0f - Debug_Yaw_Diff;
        if (remain_yaw < 0.0f)
          remain_yaw = 0.0f;
        float slow_speed = 150.0f + (remain_yaw / 80.0f) * (400.0f - 150.0f);
        if (slow_speed < 150.0f)
          slow_speed = 150.0f;
        Basic_Speed = slow_speed;
      }
    } else {
      // 硬件兜底：陀螺仪掉线则退化为里程减速 (延长至最后 800 里程)
      if (Measure_Distance > 4264.0f) {
        float remain_dist = 5064.0f - Measure_Distance;
        if (remain_dist < 0.0f)
          remain_dist = 0.0f;
        float slow_speed = 150.0f + (remain_dist / 800.0f) * (400.0f - 150.0f);
        if (slow_speed < 150.0f)
          slow_speed = 150.0f;
        Basic_Speed = slow_speed;
      }
    }
    break;

  case 3: // 彻底完成，进入主动刹车 (Active Braking)
    if (Basic_Speed != 0.0f) {
      Basic_Speed = 0.0f; // 目标速度设为 0，开始反接制动刹车
      // 注：此处不再立刻停止计时，等待后续对准完成再停表
      // 【关键修改】：这里绝对不调用 Motors_Disable() 和 Turn_PID_Flag = 0！
      // 保持电机使能和速度环闭环运行。当目标速度为0，但车身因惯性向前滑行时，
      // 底层速度 PID (timer.c) 会测出巨大的负向误差，从而瞬间输出强力的反向
      // PWM（反接制动）， 将小车死死钉在终点线上！
    }

    // 引入局部静态计时器，刹车维持 300ms 确保完全停稳后，进入对准状态
    {
      extern volatile uint32_t ms_ticks;
      static uint32_t brake_start_time = 0;
      if (brake_start_time == 0) {
        brake_start_time = ms_ticks;
      }

      if (ms_ticks - brake_start_time > 300) {
        brake_start_time = 0; // 为下一次任务重置状态
        Task1_Time_flag = 0;  // 停止计时
        Motors_Disable();     // 关闭电机使能
        Task_1_State = 5;     // 直接进入彻底停车死锁状态，不再进行原地旋转对准
      }
    }
    break;

  case 5:
    break; // 彻底停车死锁状态

  default:
    break;
  }
}

/* ================================================================== */
/* ============================ 任务二 ============================== */
/* ================================================================== */

uint8_t Task_2_State = 0;

void Task_2(void) {
  switch (Task_2_State) {
  case 0: // 初始待机
    break;

  case 1:         // 启动任务二 (轨迹模式)
    Car_Mode = 8; // Ball_Task2_Mode
    // 重置摆杆基准位置到机械水平位，防止启动前K230的BB帧
    // 在积分分支中累积PD42S1_Motor2_Pos导致基准漂移
    extern int32_t PD42S1_Motor2_Pos;
    PD42S1_Motor2_Pos = 22300;
    // 清除摆杆电机可能的堵转保护状态并关闭堵转保护，
    // 防止PID快速来回调整时触发堵转失能导致电机突然不动
    extern uint8_t g_smd_target_uart;
    g_smd_target_uart = 2;
    smd_remove_clog_protect(1);
    smd_clear_sta(1);
    smd_set_clog_pro(1, 0);
    
    extern uint8_t Task2_Restart_Flag;
    extern uint8_t Task2_Trajectory_Enabled;
    Task2_Trajectory_Enabled = 1;   // 启用轨迹模式
    Task2_Restart_Flag = 1;         // 强制timer.c重置状态机
    
    extern void BLE_send_String(unsigned char *str);
    BLE_send_String((unsigned char *)"Task 2 Trajectory: 0->+5->-5cm\r\n");
    Task_2_State = 2;
    break;

  case 2: // 执行中
    // 状态机在 timer.c 的 Control_Ball_Task2_Mode 中自动运行
    break;

  default:
    break;
  }
}

/* ================================================================== */
/* ============================ 任务三 ============================== */
/* ================================================================== */

uint8_t Task_3_State = 0;
uint8_t Task3_Time_flag = 0;
float Task3_Time_Sec = 0.0f;
static uint32_t t3_start_time = 0;

void Task_3(void) {
  switch (Task_3_State) {
  case 0: // 初始待机
    Task3_Ramp_Active = 0; // 确保斜坡停止
    Motors_Disable();
    Basic_Speed = 0.0f;
    Turn_PID_Flag = 0;
    t3_start_time = 0;
    break;

  case 1: // 启动初始化
    // 任务启动时重置累计路程与计时基准
    Measure_Distance = 0;
    Motor1_Lucheng = 0;
    Motor2_Lucheng = 0;
    Task3_Time_Sec = 0.0f;
    t3_start_time = ms_ticks;

    // --- 开启小球平衡 ---
    Car_Mode = 8; // Ball_Task2_Mode (纯定点平衡)
    extern int32_t PD42S1_Motor2_Pos;
    PD42S1_Motor2_Pos = 22300;
    extern uint8_t g_smd_target_uart;
    g_smd_target_uart = 2;
    smd_remove_clog_protect(1);
    smd_clear_sta(1);
    smd_set_clog_pro(1, 0);

    extern uint8_t Task2_Trajectory_Enabled;
    Task2_Trajectory_Enabled = 0; // 纯定点平衡，关闭轨迹模式

    // 【关键防护】强制关闭测试模式标志，防止 Control_Run_Mode 中的
    // Tracking_Test_Flag 分支覆盖 Basic_Speed，导致 S 曲线失效直接跳目标速度
    Test_Speed_Mode = 0;
    Tracking_Test_Flag = 0;

    // 启动 cosine S 曲线斜坡 (在 timer.c 的 10ms 中断中精确执行)
    // 速度由 timer.c 根据 Task3_Ramp_Start 时间戳自动计算:
    //   0~accel_time秒: 起步速度 -> 巡航速度 (cosine S曲线加速)
    //   5秒~5+decel_time秒: 巡航速度 -> 0 (cosine S曲线减速)
    Task3_Ramp_Start = ms_ticks;
    Task3_Ramp_Active = 1;

    Motors_Enable();
    PID_Reset(&pid_Turn);

    Turn_PID_Flag = 1;     // 开启灰度循迹 (任务三赛题核心)
    Distance_PID_Flag = 0;
    Gyro_PID_Flag = 0;
    Angle_PID_Flag = 0;

    Basic_Speed = Task3_Cruise_Speed; // 设为目标速度, S曲线由 timer.c 管理
    Task_3_State = 2; // 进入行驶
    break;

  case 2: // 行驶状态 (5 秒后进入减速阶段)
    Task3_Time_flag = 1;
    Task3_Time_Sec = (ms_ticks - t3_start_time) / 1000.0f;

    // 循迹 5 秒后进入减速阶段 (timer.c 自动切换为减速 S 曲线)
    if (ms_ticks - t3_start_time >= 5000) {
      Task_3_State = 3;
    }
    break;

  case 3: // 减速阶段 (9.5 秒强制完全停稳，即 4.5 秒刹车窗口)
  {
    Task3_Time_Sec = (ms_ticks - t3_start_time) / 1000.0f;
    extern float Soft_Basic_Speed;

    // 9.5 秒强制停车：5+4.5=9.5s，无论 S 曲线是否走完，都必须完全静止
    if (ms_ticks - t3_start_time >= 9500) {
      Task3_Ramp_Active = 0; // 停止 S 曲线斜坡
      Turn_PID_Flag = 0;   // 关闭循迹，防止原地打转掉球
      Task3_Time_flag = 0; // 停止计时
      Motors_Disable();    // 切断电机使能
      Task_3_State = 4;    // 彻底停车死锁状态
    }
    // S 曲线速度已衰减到接近 0 时，可提前进入死锁
    else if (Soft_Basic_Speed > -2.0f && Soft_Basic_Speed < 2.0f) {
      Task3_Ramp_Active = 0;
      Turn_PID_Flag = 0;
      Task3_Time_flag = 0;
      Motors_Disable();
      Task_3_State = 4;
    }
  } break;

  case 4:
    Task3_Ramp_Active = 0; // 确保斜坡停止
    break; // 停车死锁状态

  default:
    break;
  }
}

/* ================================================================== */
/* ============================ 任务四 ============================== */
/* ================================================================== */
// 任务四: 与任务三相同的循迹+小球动平衡+S曲线起步刹车
// 区别: 跑一圈后停止, 采用陀螺仪角度触发刹车 (默认提前90度开始停车)

uint8_t Task_4_State = 0;
uint8_t Task4_Time_flag = 0;
float Task4_Time_Sec = 0.0f;
static uint32_t t4_start_time = 0;
static float t4_start_yaw = 0.0f;

void Task_4(void) {
  switch (Task_4_State) {
  case 0: // 初始待机
    Task4_Ramp_Active = 0;
    Task4_Brake_Triggered = 0;
    Motors_Disable();
    Basic_Speed = 0.0f;
    Turn_PID_Flag = 0;
    t4_start_time = 0;
    break;

  case 1: // 启动初始化
    // 任务启动时重置累计路程与计时基准
    Measure_Distance = 0;
    Motor1_Lucheng = 0;
    Motor2_Lucheng = 0;
    Task4_Time_Sec = 0.0f;
    t4_start_time = ms_ticks;

    // 记录起始航向角 (用于跑一圈的角度判定)
    if (JY61P_Data != NULL) {
      t4_start_yaw = JY61P_Data->total_z;
    } else {
      t4_start_yaw = 0.0f;
    }

    // --- 开启小球平衡 ---
    Car_Mode = 8; // Ball_Task2_Mode (纯定点平衡)
    extern int32_t PD42S1_Motor2_Pos;
    PD42S1_Motor2_Pos = 22300;
    extern uint8_t g_smd_target_uart;
    g_smd_target_uart = 2;
    smd_remove_clog_protect(1);
    smd_clear_sta(1);
    smd_set_clog_pro(1, 0);

    extern uint8_t Task2_Trajectory_Enabled;
    Task2_Trajectory_Enabled = 0; // 纯定点平衡，关闭轨迹模式

    // 【关键防护】强制关闭测试模式标志，防止覆盖 Basic_Speed
    Test_Speed_Mode = 0;
    Tracking_Test_Flag = 0;

    // 启动 cosine S 曲线斜坡 (加速阶段由 timer.c 精确执行)
    Task4_Brake_Triggered = 0;
    Task4_Ramp_Start = ms_ticks;
    Task4_Ramp_Active = 1;

    Motors_Enable();
    PID_Reset(&pid_Turn);

    Turn_PID_Flag = 1;     // 开启灰度循迹
    Distance_PID_Flag = 0;
    Gyro_PID_Flag = 0;
    Angle_PID_Flag = 0;

    Basic_Speed = Task4_Cruise_Speed; // S曲线由 timer.c 管理
    Task_4_State = 2; // 进入行驶
    break;

  case 2: // 行驶状态 (角度触发刹车)
    Task4_Time_flag = 1;
    Task4_Time_Sec = (ms_ticks - t4_start_time) / 1000.0f;

    // 计算航向角差 (与任务一相同的角度闭环逻辑)
    if (JY61P_Data != NULL) {
      float diff = JY61P_Data->total_z - t4_start_yaw;
      if (diff < 0.0f)
        diff = -diff;
      Debug_Yaw_Diff = diff;

      // 角度达到刹车阈值 (默认257度=347-90, 提前90度开始停车)
      if (diff >= Task4_Brake_Angle && !Task4_Brake_Triggered) {
        Task4_Brake_Triggered = 1;
        Task4_Brake_Start = ms_ticks;
        Task_4_State = 3; // 进入减速阶段
      }
    }
    break;

  case 3: // 减速阶段 (cosine S曲线减速到0)
  {
    Task4_Time_Sec = (ms_ticks - t4_start_time) / 1000.0f;
    extern float Soft_Basic_Speed;
    float brake_elapsed = (ms_ticks - Task4_Brake_Start) / 1000.0f;

    // 刹车时间到 或 S曲线速度已接近0，完全停车
    if (brake_elapsed >= Task4_Decel_Coeff ||
        (Soft_Basic_Speed > -2.0f && Soft_Basic_Speed < 2.0f)) {
      Task4_Ramp_Active = 0;
      Task4_Brake_Triggered = 0;
      Turn_PID_Flag = 0;
      Task4_Time_flag = 0;
      Motors_Disable();
      Task_4_State = 4; // 彻底停车死锁状态
    }
  } break;

  case 4:
    Task4_Ramp_Active = 0;
    Task4_Brake_Triggered = 0;
    break; // 停车死锁状态

  default:
    break;
  }
}
